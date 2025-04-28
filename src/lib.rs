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

#[derive(Debug)]
pub struct Bme280<I2c> {
    i2c: I2c,
    address: SevenBitAddress,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl<I2C> defmt::Format for Bme280<I2C> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "BME280 {{ address {=u8:#X} }}", self.address,)
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

impl<I2C, E> Bme280<I2C>
where
    I2C: I2c<Error = E>,
    E: embedded_hal_async::i2c::Error,
{
    pub async fn new_with_address(
        i2c: I2C,
        address: SevenBitAddress,
    ) -> Result<Self, ChipIdError<E>> {
        let mut this = Self { i2c, address };

        let chip_id = this.read_chip_id().await?;
        if CHIP_ID == chip_id {
            Ok(this)
        } else {
            Err(ChipIdError::WrongChipId(chip_id))
        }
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

        let expectations = [I2cTransaction::write_read(
            I2C_ADDRESS,
            vec![REGISTER_ID_ADDRESS],
            vec![CHIP_ID],
        )];
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
