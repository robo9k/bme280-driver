use bitbybit::{bitenum, bitfield};
use embedded_hal_async::i2c::{I2c, SevenBitAddress};

use crate::{Bme280, ChipId, State};

pub(crate) const REGISTER_ID_ADDRESS: SevenBitAddress = 0xD0;
pub(crate) const REGISTER_RESET_ADDRESS: SevenBitAddress = 0xE0;
pub(crate) const REGISTER_CTRL_HUM_ADDRESS: SevenBitAddress = 0xF2;
pub(crate) const REGISTER_STATUS_ADDRESS: SevenBitAddress = 0xF3;
pub(crate) const REGISTER_CTRL_MEAS_ADDRESS: SevenBitAddress = 0xF4;
pub(crate) const REGISTER_CONFIG_ADDRESS: SevenBitAddress = 0xF5;
pub(crate) const REGISTER_CALIB26_CALIB41_ADDRESS: SevenBitAddress = 0xE1;
pub(crate) const REGISTER_CALIB00_CALIB25_ADDRESS: SevenBitAddress = 0x88;
pub(crate) const REGISTER_RAW_MEASUREMENT_ADDRESS: SevenBitAddress = 0xF7;

pub(crate) const REGISTER_CALIB26_CALIB41_LENGTH: usize = 7;
pub(crate) const REGISTER_CALIB00_CALIB25_LENGTH: usize = 26;
pub(crate) const REGISTER_CALIB_LENGTH: usize =
    REGISTER_CALIB00_CALIB25_LENGTH + REGISTER_CALIB26_CALIB41_LENGTH;

pub(crate) const REGISTER_RAW_MEASUREMENT_LENGTH: usize = 8;

pub(crate) const REGISTER_HUM_SKIPPED: u16 = 0x8000;
pub(crate) const REGISTER_TEMP_SKIPPED: u32 = 0x80000;
pub(crate) const REGISTER_PRESS_SKIPPED: u32 = 0x80000;

pub(crate) const REGISTER_RESET_MAGIC: SevenBitAddress = 0xB6;

#[bitenum(u3, exhaustive = false)]
pub(crate) enum HumidityOversampling {
    Skip = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

#[bitfield(u8, default = 0x00)]
pub(crate) struct HumidityControl {
    #[bits(0..=2, rw)]
    humidity_oversampling: Option<HumidityOversampling>,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for HumidityControl {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ctrl_hum {{ osrs_h {=0..3} }}", self.raw_value,)
    }
}

#[bitenum(u3, exhaustive = false)]
pub(crate) enum TemperatureOversampling {
    Skip = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

#[bitenum(u3, exhaustive = false)]
pub(crate) enum PressureOversampling {
    Skip = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

#[bitenum(u2, exhaustive = false)]
pub(crate) enum Mode {
    Sleep = 0b00,
    Forced = 0b01,
    Normal = 0b11,
}

#[bitfield(u8, default = 0x00)]
pub(crate) struct MeasurementControl {
    #[bits(5..=7, rw)]
    temperature_oversampling: Option<TemperatureOversampling>,

    #[bits(2..=4, rw)]
    pressure_oversampling: Option<PressureOversampling>,

    #[bits(0..=1, rw)]
    mode: Option<Mode>,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for MeasurementControl {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "ctrl_meas {{ osrs_t {0=5..8} osrs_p {0=2..5} mode {0=0..2} }}",
            self.raw_value,
        )
    }
}

#[bitenum(u3, exhaustive = true)]
pub(crate) enum StandbyTime {
    Ms0_5 = 0b000,
    Ms62_5 = 0b001,
    Ms125 = 0b010,
    Ms250 = 0b011,
    Ms500 = 0b100,
    Ms1000 = 0b101,
    Ms10 = 0b110,
    Ms20 = 0b111,
}

#[bitenum(u3, exhaustive = false)]
pub(crate) enum Filter {
    Off = 0b000,
    X2 = 0b001,
    X4 = 0b010,
    X8 = 0b011,
    X16 = 0b100,
}

#[bitfield(u8, default = 0x00)]
pub(crate) struct Config {
    #[bits(5..=7, rw)]
    standby_time: StandbyTime,

    #[bits(2..=4, rw)]
    filter: Option<Filter>,

    #[bit(10, rw)]
    spi_3wire_enabled: bool,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for Config {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "config {{ t_sb {0=5..8} filter {0=2..5} spi3w_en {0=0..1} }}",
            self.raw_value,
        )
    }
}

#[bitfield(u8, default = 0x00)]
pub(crate) struct Status {
    #[bit(3, rw)]
    measuring: bool,

    #[bit(0, rw)]
    im_update: bool,
}

pub(crate) struct AdcPressure(pub(crate) u32);

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for AdcPressure {
    fn format(&self, f: defmt::Formatter) {
        self.0.format(f)
    }
}

pub(crate) struct AdcTemperature(pub(crate) u32);

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for AdcTemperature {
    fn format(&self, f: defmt::Formatter) {
        self.0.format(f)
    }
}

pub(crate) struct AdcHumidity(pub(crate) u16);

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for AdcHumidity {
    fn format(&self, f: defmt::Formatter) {
        self.0.format(f)
    }
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub(crate) struct RawMeasurement {
    pub(crate) pressure: Option<AdcPressure>,
    pub(crate) temperature: Option<AdcTemperature>,
    pub(crate) humidity: Option<AdcHumidity>,
}

impl<I2C, E, S> Bme280<I2C, S>
where
    I2C: I2c<Error = E>,
    E: embedded_hal_async::i2c::Error,
    S: State,
{
    pub(crate) async fn read_chip_id(&mut self) -> Result<ChipId, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_ID_ADDRESS], &mut data)
            .await?;
        Ok(data[0])
    }

    pub(crate) async fn write_reset(&mut self) -> Result<(), E> {
        self.i2c
            .write(
                self.address,
                &[REGISTER_RESET_ADDRESS, REGISTER_RESET_MAGIC],
            )
            .await?;
        Ok(())
    }

    pub(crate) async fn read_ctrl_hum(&mut self) -> Result<HumidityControl, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CTRL_HUM_ADDRESS], &mut data)
            .await?;
        let ctrl_hum = HumidityControl::new_with_raw_value(data[0]);
        Ok(ctrl_hum)
    }

    pub(crate) async fn write_ctrl_hum(&mut self, ctrl_hum: HumidityControl) -> Result<(), E> {
        self.i2c
            .write(
                self.address,
                &[REGISTER_CTRL_HUM_ADDRESS, ctrl_hum.raw_value()],
            )
            .await?;
        Ok(())
    }

    pub(crate) async fn read_ctrl_meas(&mut self) -> Result<MeasurementControl, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CTRL_MEAS_ADDRESS], &mut data)
            .await?;
        let ctrl_meas = MeasurementControl::new_with_raw_value(data[0]);
        Ok(ctrl_meas)
    }

    pub(crate) async fn write_ctrl_meas(&mut self, ctrl_meas: MeasurementControl) -> Result<(), E> {
        self.i2c
            .write(
                self.address,
                &[REGISTER_CTRL_MEAS_ADDRESS, ctrl_meas.raw_value()],
            )
            .await?;
        Ok(())
    }

    pub(crate) async fn read_config(&mut self) -> Result<Config, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CONFIG_ADDRESS], &mut data)
            .await?;
        let config = Config::new_with_raw_value(data[0]);
        Ok(config)
    }

    pub(crate) async fn write_config(&mut self, config: Config) -> Result<(), E> {
        self.i2c
            .write(self.address, &[REGISTER_CONFIG_ADDRESS, config.raw_value()])
            .await?;
        Ok(())
    }

    pub(crate) async fn read_status(&mut self) -> Result<Status, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_STATUS_ADDRESS], &mut data)
            .await?;
        let status = Status::new_with_raw_value(data[0]);
        Ok(status)
    }

    pub(crate) async fn read_calib(&mut self) -> Result<[u8; REGISTER_CALIB_LENGTH], E> {
        let mut data: [u8; REGISTER_CALIB_LENGTH] = [0; REGISTER_CALIB_LENGTH];

        self.i2c
            .write_read(
                self.address,
                &[REGISTER_CALIB00_CALIB25_ADDRESS],
                &mut data[0..REGISTER_CALIB00_CALIB25_LENGTH],
            )
            .await?;
        self.i2c
            .write_read(
                self.address,
                &[REGISTER_CALIB26_CALIB41_ADDRESS],
                &mut data[REGISTER_CALIB00_CALIB25_LENGTH..REGISTER_CALIB_LENGTH],
            )
            .await?;

        Ok(data)
    }

    pub(crate) async fn read_raw_measurement(&mut self) -> Result<RawMeasurement, E> {
        let mut data: [u8; REGISTER_RAW_MEASUREMENT_LENGTH] = [0; REGISTER_RAW_MEASUREMENT_LENGTH];

        self.i2c
            .write_read(self.address, &[REGISTER_RAW_MEASUREMENT_ADDRESS], &mut data)
            .await?;

        let adc_p: u32 =
            (u32::from(data[0]) << 12) | (u32::from(data[1]) << 4) | (u32::from(data[2]) >> 4);
        #[cfg(feature = "defmt-03")]
        defmt::trace!("adc_p {=u32}", adc_p);

        let adc_t: u32 =
            (u32::from(data[3]) << 12) | (u32::from(data[4]) << 4) | (u32::from(data[5]) >> 4);
        #[cfg(feature = "defmt-03")]
        defmt::trace!("adc_t {=u32}", adc_t);

        let adc_h: u16 = (u16::from(data[6]) << 8) | u16::from(data[7]);
        #[cfg(feature = "defmt-03")]
        defmt::trace!("adc_p {=u16}", adc_h);

        let raw_measurement = RawMeasurement {
            pressure: if adc_p == REGISTER_PRESS_SKIPPED {
                None
            } else {
                Some(AdcPressure(adc_p))
            },
            temperature: if adc_t == REGISTER_TEMP_SKIPPED {
                None
            } else {
                Some(AdcTemperature(adc_t))
            },
            humidity: if adc_h == REGISTER_HUM_SKIPPED {
                None
            } else {
                Some(AdcHumidity(adc_h))
            },
        };
        #[cfg(feature = "defmt-03")]
        defmt::debug!("raw_measurement {}", raw_measurement);
        Ok(raw_measurement)
    }
}
