#![forbid(unsafe_code)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![cfg_attr(docsrs, feature(doc_cfg_hide))]
#![cfg_attr(docsrs, doc(cfg_hide(docsrs)))]

use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use thiserror::Error;

mod lowlevel;

pub type ChipId = u8;
pub const CHIP_ID: ChipId = 0x60;

mod private {
    pub trait Sealed {}

    impl Sealed for super::Initialized {}
    impl Sealed for super::Calibrated {}
}

pub trait State: private::Sealed {}

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Initialized;

impl State for Initialized {}

#[derive(Debug)]
struct Calibration {
    temperature1: u16,
    temperature2: i16,
    temperature3: i16,

    pressure1: u16,
    pressure2: i16,
    pressure3: i16,
    pressure4: i16,
    pressure5: i16,
    pressure6: i16,
    pressure7: i16,
    pressure8: i16,
    pressure9: i16,

    humidity1: u8,
    humidity2: i16,
    humidity3: u8,
    humidity4: i16,
    humidity5: i16,
    humidity6: i8,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for Calibration {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Calibration {{ \
            dig_T1: {=u16}, dig_T2: {=i16}, dig_T3: {=i16}, \
        dig_P1: {=u16}, dig_P2: {=i16}, dig_P3: {=i16}, dig_P4: {=i16}, dig_P5: {=i16}, dig_P6: {=i16}, dig_P7: {=i16}, dig_P8: {=i16}, dig_P9: {=i16}, \
        dig_H1: {=u8}, dig_H2: {=i16}, dig_H3: {=u8}, dig_H4: {=i16}, dig_H5: {=i16}, dig_H6: {=i8} \
        }}",
            self.temperature1,
            self.temperature2,
            self.temperature3,
            self.pressure1,
            self.pressure2,
            self.pressure3,
            self.pressure4,
            self.pressure5,
            self.pressure6,
            self.pressure7,
            self.pressure8,
            self.pressure9,
            self.humidity1,
            self.humidity2,
            self.humidity3,
            self.humidity4,
            self.humidity5,
            self.humidity6,
        )
    }
}

impl From<[u8; lowlevel::REGISTER_CALIB_LENGTH]> for Calibration {
    fn from(data: [u8; lowlevel::REGISTER_CALIB_LENGTH]) -> Self {
        Self {
            temperature1: u16::from_le_bytes([data[0], data[1]]),
            temperature2: i16::from_le_bytes([data[2], data[3]]),
            temperature3: i16::from_le_bytes([data[4], data[5]]),

            pressure1: u16::from_le_bytes([data[6], data[7]]),
            pressure2: i16::from_le_bytes([data[8], data[9]]),
            pressure3: i16::from_le_bytes([data[10], data[11]]),
            pressure4: i16::from_le_bytes([data[12], data[13]]),
            pressure5: i16::from_le_bytes([data[14], data[15]]),
            pressure6: i16::from_le_bytes([data[16], data[17]]),
            pressure7: i16::from_le_bytes([data[18], data[19]]),
            pressure8: i16::from_le_bytes([data[20], data[21]]),
            pressure9: i16::from_le_bytes([data[22], data[23]]),

            humidity1: data[25],
            humidity2: i16::from_le_bytes([data[26], data[27]]),
            humidity3: data[28],
            humidity4: i16::from(data[29]) << 4 | i16::from(data[30]) & 0xf,
            humidity5: ((i16::from(data[30]) & 0xf0) >> 4) | (i16::from(data[31]) << 4),
            humidity6: data[32] as i8,
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
struct FineTemperature(i32);

impl Calibration {
    fn compensate_temperature(&self, adc_t: lowlevel::AdcTemperature) -> FineTemperature {
        let adc_t = adc_t.0 as i32;

        let var1 = (((adc_t >> 3) - (i32::from(self.temperature1) << 1))
            * i32::from(self.temperature2))
            >> 11;
        let var2 = (((((adc_t >> 4) - i32::from(self.temperature1))
            * ((adc_t >> 4) - i32::from(self.temperature1)))
            >> 12)
            * i32::from(self.temperature3))
            >> 14;

        FineTemperature(var1 + var2)
    }

    fn fine_temperature_to_temperature(fine_temperature: FineTemperature) -> i32 {
        (fine_temperature.0 * 5 + 128) >> 8
    }

    fn compensate_pressure(&self, adc_p: lowlevel::AdcPressure, t_fine: FineTemperature) -> u32 {
        let adc_p = adc_p.0;
        let t_fine = t_fine.0;

        let var1 = i64::from(t_fine) - 128_000;
        let var2 = var1 * var1 * i64::from(self.pressure6);
        let var2 = var2 + ((var1 * i64::from(self.pressure5)) << 17);
        let var2 = var2 + (i64::from(self.pressure4) << 35);
        let var1 = ((var1 * var1 * i64::from(self.pressure3)) >> 8)
            + ((var1 * i64::from(self.pressure2)) << 12);
        let var1 = ((((1_i64) << 47) + var1) * i64::from(self.pressure1)) >> 33;

        if var1 == 0 {
            // division by zero
            0
        } else {
            let var4 = 1_048_576 - i64::from(adc_p);
            let var4 = (((var4 << 31) - var2) * 3125) / var1;
            let var1 = (i64::from(self.pressure9) * (var4 >> 13) * (var4 >> 13)) >> 25;
            let var2 = (i64::from(self.pressure8) * var4) >> 19;
            let var5 = ((var4 + var1 + var2) >> 8) + (i64::from(self.pressure7) << 4);

            let p = var5;
            let pressure = p as u32;

            pressure
        }
    }

    fn compensate_humidity(&self, adc_h: lowlevel::AdcHumidity, t_fine: FineTemperature) -> u32 {
        let adc_h = i32::from(adc_h.0);
        let t_fine = t_fine.0;

        let v_x1_u32r: i32 = t_fine - 76_800_i32;
        let v_x1_u32r: i32 = ((((adc_h << 14)
            - (i32::from(self.humidity4) << 20)
            - (i32::from(self.humidity5) * v_x1_u32r))
            + (16_384_i32))
            >> 15)
            * (((((((v_x1_u32r * i32::from(self.humidity6)) >> 10)
                * (((v_x1_u32r * i32::from(self.humidity3)) >> 11) + (32_768_i32)))
                >> 10)
                + (2_097_152_i32))
                * i32::from(self.humidity2)
                + 8192_i32)
                >> 14);
        let v_x1_u32r: i32 = v_x1_u32r
            - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * i32::from(self.humidity1)) >> 4);
        let v_x1_u32r = if v_x1_u32r < 0 { 0 } else { v_x1_u32r };
        let v_x1_u32r = if v_x1_u32r > 419_430_400 {
            419_430_400
        } else {
            v_x1_u32r
        };

        let humidity = v_x1_u32r >> 12;
        let humidity = humidity as u32;

        humidity
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Calibrated {
    calibration: Calibration,
}

impl State for Calibrated {}

#[derive(Debug)]
pub struct Bme280<I2C, S: State> {
    i2c: I2C,
    address: SevenBitAddress,
    state: S,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl<I2C> defmt::Format for Bme280<I2C, Initialized> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Bme280 {{ address: {=u8:#X} }}", self.address,)
    }
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl<I2C> defmt::Format for Bme280<I2C, Calibrated> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Bme280 {{ address: {=u8:#X}, calibration: {} }}",
            self.address,
            self.state.calibration
        )
    }
}

#[derive(Error, Debug, PartialEq)]
pub enum ChipIdError<E: embedded_hal_async::i2c::Error> {
    #[error("I²C kind {}", .0.kind())]
    I2c(E),

    #[error("wrong chip id: {0} (expected {expected})", expected = CHIP_ID)]
    WrongChipId(u8),
}

impl<E> From<E> for ChipIdError<E>
where
    E: embedded_hal_async::i2c::Error,
{
    fn from(error: E) -> Self {
        Self::I2c(error)
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum HumidityOversampling {
    #[default]
    Skip,
    X1,
    X2,
    X4,
    X8,
    X16,
}

impl From<lowlevel::HumidityOversampling> for HumidityOversampling {
    fn from(humidity_oversampling: lowlevel::HumidityOversampling) -> Self {
        match humidity_oversampling {
            lowlevel::HumidityOversampling::Skip => Self::Skip,
            lowlevel::HumidityOversampling::X1 => Self::X1,
            lowlevel::HumidityOversampling::X2 => Self::X2,
            lowlevel::HumidityOversampling::X4 => Self::X4,
            lowlevel::HumidityOversampling::X8 => Self::X8,
            lowlevel::HumidityOversampling::X16 => Self::X16,
        }
    }
}

impl From<HumidityOversampling> for lowlevel::HumidityOversampling {
    fn from(humidity_oversampling: HumidityOversampling) -> Self {
        match humidity_oversampling {
            HumidityOversampling::Skip => Self::Skip,
            HumidityOversampling::X1 => Self::X1,
            HumidityOversampling::X2 => Self::X2,
            HumidityOversampling::X4 => Self::X4,
            HumidityOversampling::X8 => Self::X8,
            HumidityOversampling::X16 => Self::X16,
        }
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum TemperatureOversampling {
    #[default]
    Skip,
    X1,
    X2,
    X4,
    X8,
    X16,
}

impl From<lowlevel::TemperatureOversampling> for TemperatureOversampling {
    fn from(temperature_oversampling: lowlevel::TemperatureOversampling) -> Self {
        match temperature_oversampling {
            lowlevel::TemperatureOversampling::Skip => Self::Skip,
            lowlevel::TemperatureOversampling::X1 => Self::X1,
            lowlevel::TemperatureOversampling::X2 => Self::X2,
            lowlevel::TemperatureOversampling::X4 => Self::X4,
            lowlevel::TemperatureOversampling::X8 => Self::X8,
            lowlevel::TemperatureOversampling::X16 => Self::X16,
        }
    }
}

impl From<TemperatureOversampling> for lowlevel::TemperatureOversampling {
    fn from(temperature_oversampling: TemperatureOversampling) -> Self {
        match temperature_oversampling {
            TemperatureOversampling::Skip => Self::Skip,
            TemperatureOversampling::X1 => Self::X1,
            TemperatureOversampling::X2 => Self::X2,
            TemperatureOversampling::X4 => Self::X4,
            TemperatureOversampling::X8 => Self::X8,
            TemperatureOversampling::X16 => Self::X16,
        }
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum PressureOversampling {
    #[default]
    Skip,
    X1,
    X2,
    X4,
    X8,
    X16,
}

impl From<lowlevel::PressureOversampling> for PressureOversampling {
    fn from(pressure_oversampling: lowlevel::PressureOversampling) -> Self {
        match pressure_oversampling {
            lowlevel::PressureOversampling::Skip => Self::Skip,
            lowlevel::PressureOversampling::X1 => Self::X1,
            lowlevel::PressureOversampling::X2 => Self::X2,
            lowlevel::PressureOversampling::X4 => Self::X4,
            lowlevel::PressureOversampling::X8 => Self::X8,
            lowlevel::PressureOversampling::X16 => Self::X16,
        }
    }
}

impl From<PressureOversampling> for lowlevel::PressureOversampling {
    fn from(pressure_oversampling: PressureOversampling) -> Self {
        match pressure_oversampling {
            PressureOversampling::Skip => Self::Skip,
            PressureOversampling::X1 => Self::X1,
            PressureOversampling::X2 => Self::X2,
            PressureOversampling::X4 => Self::X4,
            PressureOversampling::X8 => Self::X8,
            PressureOversampling::X16 => Self::X16,
        }
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Mode {
    #[default]
    Sleep,
    Forced,
    Normal,
}

impl From<lowlevel::Mode> for Mode {
    fn from(mode: lowlevel::Mode) -> Self {
        match mode {
            lowlevel::Mode::Sleep => Self::Sleep,
            lowlevel::Mode::Forced => Self::Forced,
            lowlevel::Mode::Normal => Self::Normal,
        }
    }
}

impl From<Mode> for lowlevel::Mode {
    fn from(mode: Mode) -> Self {
        match mode {
            Mode::Sleep => Self::Sleep,
            Mode::Forced => Self::Forced,
            Mode::Normal => Self::Normal,
        }
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Control {
    pub humidity_oversampling: HumidityOversampling,
    pub temperature_oversampling: TemperatureOversampling,
    pub pressure_oversampling: PressureOversampling,
    pub mode: Mode,
}

impl From<(lowlevel::HumidityControl, lowlevel::MeasurementControl)> for Control {
    fn from(
        (humidity_control, measurement_control): (
            lowlevel::HumidityControl,
            lowlevel::MeasurementControl,
        ),
    ) -> Self {
        let humidity_oversampling = humidity_control
            .humidity_oversampling()
            // other bit patterns map to ×16
            .map_or(HumidityOversampling::X16, |ho| ho.into());

        let temperature_oversampling = measurement_control
            .temperature_oversampling()
            // other bit patterns map to ×16
            .map_or(TemperatureOversampling::X16, |to| to.into());

        let pressure_oversampling = measurement_control
            .pressure_oversampling()
            // other bit patterns map to ×16
            .map_or(PressureOversampling::X16, |po| po.into());

        let mode = measurement_control
            .mode()
            // two bit patterns map to Forced
            .map_or(Mode::Forced, |m| m.into());

        Self {
            humidity_oversampling,
            temperature_oversampling,
            pressure_oversampling,
            mode,
        }
    }
}

impl From<Control> for (lowlevel::HumidityControl, lowlevel::MeasurementControl) {
    fn from(control: Control) -> Self {
        let humidity_control = lowlevel::HumidityControl::builder()
            .with_humidity_oversampling(control.humidity_oversampling.into())
            .build();
        let measurement_control = lowlevel::MeasurementControl::builder()
            .with_temperature_oversampling(control.temperature_oversampling.into())
            .with_pressure_oversampling(control.pressure_oversampling.into())
            .with_mode(control.mode.into())
            .build();

        (humidity_control, measurement_control)
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum StandbyTime {
    #[default]
    Millis0_5,
    Millis62_5,
    Millis125,
    Millis250,
    Millis500,
    Millis1000,
    Millis10,
    Millis20,
}

impl From<lowlevel::StandbyTime> for StandbyTime {
    fn from(standby_time: lowlevel::StandbyTime) -> Self {
        match standby_time {
            lowlevel::StandbyTime::Ms0_5 => Self::Millis0_5,
            lowlevel::StandbyTime::Ms62_5 => Self::Millis62_5,
            lowlevel::StandbyTime::Ms125 => Self::Millis125,
            lowlevel::StandbyTime::Ms250 => Self::Millis250,
            lowlevel::StandbyTime::Ms500 => Self::Millis500,
            lowlevel::StandbyTime::Ms1000 => Self::Millis1000,
            lowlevel::StandbyTime::Ms10 => Self::Millis10,
            lowlevel::StandbyTime::Ms20 => Self::Millis20,
        }
    }
}

impl From<StandbyTime> for lowlevel::StandbyTime {
    fn from(standby_time: StandbyTime) -> Self {
        match standby_time {
            StandbyTime::Millis0_5 => Self::Ms0_5,
            StandbyTime::Millis62_5 => Self::Ms62_5,
            StandbyTime::Millis125 => Self::Ms125,
            StandbyTime::Millis250 => Self::Ms250,
            StandbyTime::Millis500 => Self::Ms500,
            StandbyTime::Millis1000 => Self::Ms1000,
            StandbyTime::Millis10 => Self::Ms10,
            StandbyTime::Millis20 => Self::Ms20,
        }
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Filter {
    #[default]
    Off,
    X2,
    X4,
    X8,
    X16,
}

impl From<lowlevel::Filter> for Filter {
    fn from(filter: lowlevel::Filter) -> Self {
        match filter {
            lowlevel::Filter::Off => Self::Off,
            lowlevel::Filter::X2 => Self::X2,
            lowlevel::Filter::X4 => Self::X4,
            lowlevel::Filter::X8 => Self::X8,
            lowlevel::Filter::X16 => Self::X16,
        }
    }
}

impl From<Filter> for lowlevel::Filter {
    fn from(filter: Filter) -> Self {
        match filter {
            Filter::Off => Self::Off,
            Filter::X2 => Self::X2,
            Filter::X4 => Self::X4,
            Filter::X8 => Self::X8,
            Filter::X16 => Self::X16,
        }
    }
}

#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Config {
    pub standby_time: StandbyTime,
    pub filter: Filter,
}

impl From<lowlevel::Config> for Config {
    fn from(config: lowlevel::Config) -> Self {
        let standby_time = config.standby_time().into();

        let filter = config
            .filter()
            // other bit patterns map to 16
            .map_or(Filter::X16, |f| f.into());

        Self {
            standby_time,
            filter,
        }
    }
}

impl From<Config> for lowlevel::Config {
    fn from(config: Config) -> Self {
        lowlevel::Config::builder()
            .with_standby_time(config.standby_time.into())
            .with_filter(config.filter.into())
            .with_spi_3wire_enabled(false)
            .build()
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum MeasurementStatus {
    Standby,
    Measuring,
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum UpdateStatus {
    Standby,
    Copying,
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Status {
    pub measurement: MeasurementStatus,
    pub update: UpdateStatus,
}

impl From<lowlevel::Status> for Status {
    fn from(status: lowlevel::Status) -> Self {
        let measurement = if status.measuring() {
            MeasurementStatus::Measuring
        } else {
            MeasurementStatus::Standby
        };

        let update = if status.im_update() {
            UpdateStatus::Copying
        } else {
            UpdateStatus::Standby
        };

        Self {
            measurement,
            update,
        }
    }
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Measurement {
    pub pressure: Option<u32>,
    pub temperature: Option<i32>,
    pub humidity: Option<u32>,
}

/// Operations that are valid in the `Initialized` state only
impl<I2C, E> Bme280<I2C, Initialized>
where
    I2C: I2c<Error = E>,
    E: embedded_hal_async::i2c::Error,
{
    pub async fn new_with_address(
        i2c: I2C,
        address: SevenBitAddress,
    ) -> Result<Bme280<I2C, Initialized>, ChipIdError<E>> {
        let mut this = Bme280 {
            i2c,
            address,
            state: Initialized,
        };

        let chip_id = this.read_chip_id().await?;
        if CHIP_ID == chip_id {
            this.reset().await?;
            Ok(this)
        } else {
            Err(ChipIdError::WrongChipId(chip_id))
        }
    }

    pub async fn calibrate(mut self) -> Result<Bme280<I2C, Calibrated>, E> {
        let calibration = self.read_calib().await?;

        let this = Bme280 {
            i2c: self.i2c,
            address: self.address,
            state: Calibrated {
                calibration: calibration.into(),
            },
        };

        #[cfg(feature = "defmt-03")]
        defmt::debug!("calibration: {}", this.state.calibration);

        Ok(this)
    }
}

/// Operations that are valid in the `Calibrated` state only
impl<I2C, E> Bme280<I2C, Calibrated>
where
    I2C: I2c<Error = E>,
    E: embedded_hal_async::i2c::Error,
{
    // TODO: alternative that takes fine_temperature as a parameter so pressure and humidity can be measured with skipped temperature
    pub async fn measure(&mut self) -> Result<Measurement, E> {
        let lowlevel::RawMeasurement {
            pressure: raw_pressure,
            temperature: raw_temperature,
            humidity: raw_humidity,
        } = self.read_raw_measurement().await?;

        let measurement = if let Some(raw_temperature) = raw_temperature {
            let calibration = &self.state.calibration;

            let fine_temperature = calibration.compensate_temperature(raw_temperature);
            let temperature = Some(Calibration::fine_temperature_to_temperature(
                fine_temperature,
            ));
            let pressure = raw_pressure.map(|raw_pressure| {
                calibration.compensate_pressure(raw_pressure, fine_temperature)
            });
            let humidity = raw_humidity.map(|raw_humidity| {
                calibration.compensate_humidity(raw_humidity, fine_temperature)
            });

            Measurement {
                pressure,
                temperature,
                humidity,
            }
        } else {
            Measurement {
                pressure: None,
                temperature: None,
                humidity: None,
            }
        };

        #[cfg(feature = "defmt-03")]
        defmt::debug!("compensated measurement: {}", measurement);

        Ok(measurement)
    }
}

/// Operations that are valid in any state
impl<I2C, E, S> Bme280<I2C, S>
where
    I2C: I2c<Error = E>,
    E: embedded_hal_async::i2c::Error,
    S: State,
{
    pub async fn reset(&mut self) -> Result<(), E> {
        self.write_reset().await?;

        Ok(())
    }

    pub async fn control(&mut self) -> Result<Control, E> {
        let ctrl_hum = self.read_ctrl_hum().await?;
        let ctrl_meas = self.read_ctrl_meas().await?;

        let control = (ctrl_hum, ctrl_meas).into();
        Ok(control)
    }

    pub async fn set_control(&mut self, control: Control) -> Result<(), E> {
        let (ctrl_hum, ctrl_meas) = control.into();

        self.write_ctrl_hum(ctrl_hum).await?;
        self.write_ctrl_meas(ctrl_meas).await?;

        Ok(())
    }

    pub async fn config(&mut self) -> Result<Config, E> {
        let config = self.read_config().await?;

        let config = config.into();
        Ok(config)
    }

    pub async fn set_config(&mut self, config: Config) -> Result<(), E> {
        let config = config.into();

        // > Writes to the “config” register in normal mode may be ignored.
        // TODO: Set device into sleep mode if needed, then write config, then set into normal mode again if needed

        self.write_config(config).await?;

        Ok(())
    }

    pub async fn status(&mut self) -> Result<Status, E> {
        let status = self.read_status().await?;

        let status = status.into();
        Ok(status)
    }

    pub fn address(&self) -> SevenBitAddress {
        self.address
    }

    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::{lowlevel::*, *};
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[futures_test::test]
    async fn test_new_with_address_ok() -> Result<(), Box<dyn std::error::Error>> {
        const I2C_ADDRESS: SevenBitAddress = 0x42;

        let expectations = [
            I2cTransaction::write_read(I2C_ADDRESS, vec![REGISTER_ID_ADDRESS], vec![CHIP_ID]),
            I2cTransaction::write(
                I2C_ADDRESS,
                vec![REGISTER_RESET_ADDRESS, REGISTER_RESET_MAGIC],
            ),
        ];
        let i2c = I2cMock::new(&expectations);

        let bme280 = Bme280::new_with_address(i2c, I2C_ADDRESS).await?;

        bme280.release().done();

        Ok(())
    }

    #[futures_test::test]
    async fn test_new_with_address_err_chipid() -> Result<(), Box<dyn std::error::Error>> {
        const I2C_ADDRESS: SevenBitAddress = 0x42;
        const WRONG_CHIP_ID: u8 = 0x42;

        let expectations = [I2cTransaction::write_read(
            I2C_ADDRESS,
            vec![REGISTER_ID_ADDRESS],
            vec![WRONG_CHIP_ID],
        )];
        let mut i2c = I2cMock::new(&expectations);

        let bme280 = Bme280::new_with_address(i2c.clone(), I2C_ADDRESS).await;
        assert_eq!(bme280.unwrap_err(), ChipIdError::WrongChipId(WRONG_CHIP_ID));

        i2c.done();

        Ok(())
    }

    #[futures_test::test]
    async fn test_new_with_address_err_i2c() -> Result<(), Box<dyn std::error::Error>> {
        use embedded_hal_async::i2c::ErrorKind;

        const I2C_ADDRESS: SevenBitAddress = 0x42;
        const I2C_ERROR: ErrorKind = ErrorKind::Other;

        let expectations =
            [
                I2cTransaction::write_read(I2C_ADDRESS, vec![REGISTER_ID_ADDRESS], vec![CHIP_ID])
                    .with_error(I2C_ERROR),
            ];
        let mut i2c = I2cMock::new(&expectations);

        let bme280 = Bme280::new_with_address(i2c.clone(), I2C_ADDRESS).await;
        assert_eq!(bme280.unwrap_err(), ChipIdError::I2c(I2C_ERROR));

        i2c.done();

        Ok(())
    }
}
