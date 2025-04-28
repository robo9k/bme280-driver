use bitbybit::{bitenum, bitfield};
use embedded_hal_async::i2c::{I2c, SevenBitAddress};

use crate::{Bme280, ChipId};

const REGISTER_ID_ADDRESS: SevenBitAddress = 0xD0;
const REGISTER_CTRL_HUM_ADDRESS: SevenBitAddress = 0xF2;
const REGISTER_STATUS_ADDRESS: SevenBitAddress = 0xF3;
const REGISTER_CTRL_MEAS_ADDRESS: SevenBitAddress = 0xF4;
const REGISTER_CONFIG_ADDRESS: SevenBitAddress = 0xF5;

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

impl<I2C, E> Bme280<I2C>
where
    I2C: I2c<Error = E>,
    E: embedded_hal_async::i2c::Error,
{
    pub(crate) async fn read_chip_id(&mut self) -> Result<ChipId, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_ID_ADDRESS], &mut data)
            .await?;
        Ok(data[0])
    }

    pub(crate) async fn read_ctrl_hum(&mut self) -> Result<HumidityControl, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CTRL_HUM_ADDRESS], &mut data)
            .await?;
        let ctrl_hum = HumidityControl::new_with_raw_value(data[0]);
        Ok(ctrl_hum)
    }

    pub(crate) async fn read_ctrl_meas(&mut self) -> Result<MeasurementControl, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CTRL_MEAS_ADDRESS], &mut data)
            .await?;
        let ctrl_meas = MeasurementControl::new_with_raw_value(data[0]);
        Ok(ctrl_meas)
    }

    pub(crate) async fn read_config(&mut self) -> Result<Config, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CONFIG_ADDRESS], &mut data)
            .await?;
        let config = Config::new_with_raw_value(data[0]);
        Ok(config)
    }

    pub(crate) async fn read_status(&mut self) -> Result<Status, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_STATUS_ADDRESS], &mut data)
            .await?;
        let status = Status::new_with_raw_value(data[0]);
        Ok(status)
    }
}
