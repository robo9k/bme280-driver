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

#[derive(Default)]
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

#[derive(Default)]
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

#[derive(Default)]
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

#[derive(Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Mode {
    #[default]
    Sleep,
    Forced,
    Normal,
}

#[derive(Default)]
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
            .map_or(HumidityOversampling::X16, |ho| match ho {
                lowlevel::HumidityOversampling::Skip => HumidityOversampling::Skip,
                lowlevel::HumidityOversampling::X1 => HumidityOversampling::X1,
                lowlevel::HumidityOversampling::X2 => HumidityOversampling::X2,
                lowlevel::HumidityOversampling::X4 => HumidityOversampling::X4,
                lowlevel::HumidityOversampling::X8 => HumidityOversampling::X8,
                lowlevel::HumidityOversampling::X16 => HumidityOversampling::X16,
            });

        let temperature_oversampling = measurement_control
            .temperature_oversampling()
            // other bit patterns map to ×16
            .map_or(TemperatureOversampling::X16, |to| match to {
                lowlevel::TemperatureOversampling::Skip => TemperatureOversampling::Skip,
                lowlevel::TemperatureOversampling::X1 => TemperatureOversampling::X1,
                lowlevel::TemperatureOversampling::X2 => TemperatureOversampling::X2,
                lowlevel::TemperatureOversampling::X4 => TemperatureOversampling::X4,
                lowlevel::TemperatureOversampling::X8 => TemperatureOversampling::X8,
                lowlevel::TemperatureOversampling::X16 => TemperatureOversampling::X16,
            });

        let pressure_oversampling = measurement_control
            .pressure_oversampling()
            // other bit patterns map to ×16
            .map_or(PressureOversampling::X16, |po| match po {
                lowlevel::PressureOversampling::Skip => PressureOversampling::Skip,
                lowlevel::PressureOversampling::X1 => PressureOversampling::X1,
                lowlevel::PressureOversampling::X2 => PressureOversampling::X2,
                lowlevel::PressureOversampling::X4 => PressureOversampling::X4,
                lowlevel::PressureOversampling::X8 => PressureOversampling::X8,
                lowlevel::PressureOversampling::X16 => PressureOversampling::X16,
            });

        let mode = measurement_control
            .mode()
            // two bit patterns map to Forced
            .map_or(Mode::Forced, |m| match m {
                lowlevel::Mode::Sleep => Mode::Sleep,
                lowlevel::Mode::Forced => Mode::Forced,
                lowlevel::Mode::Normal => Mode::Normal,
            });

        Self {
            humidity_oversampling,
            temperature_oversampling,
            pressure_oversampling,
            mode,
        }
    }
}

#[derive(Default)]
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

#[derive(Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Filter {
    #[default]
    Off,
    X2,
    X4,
    X8,
    X16,
}

#[derive(Default)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct Config {
    pub standby_time: StandbyTime,
    pub filter: Filter,
}

impl From<lowlevel::Config> for Config {
    fn from(config: lowlevel::Config) -> Self {
        let standby_time = match config.standby_time() {
            lowlevel::StandbyTime::Ms0_5 => StandbyTime::Millis0_5,
            lowlevel::StandbyTime::Ms62_5 => StandbyTime::Millis62_5,
            lowlevel::StandbyTime::Ms125 => StandbyTime::Millis125,
            lowlevel::StandbyTime::Ms250 => StandbyTime::Millis250,
            lowlevel::StandbyTime::Ms500 => StandbyTime::Millis500,
            lowlevel::StandbyTime::Ms1000 => StandbyTime::Millis1000,
            lowlevel::StandbyTime::Ms10 => StandbyTime::Millis10,
            lowlevel::StandbyTime::Ms20 => StandbyTime::Millis20,
        };

        let filter = config
            .filter()
            // other bit patterns map to 16
            .map_or(Filter::X16, |f| match f {
                lowlevel::Filter::Off => Filter::Off,
                lowlevel::Filter::X2 => Filter::X2,
                lowlevel::Filter::X4 => Filter::X4,
                lowlevel::Filter::X8 => Filter::X8,
                lowlevel::Filter::X16 => Filter::X16,
            });

        Self {
            standby_time,
            filter,
        }
    }
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum MeasurementStatus {
    Standby,
    Measuring,
}

#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum UpdateStatus {
    Standby,
    Copying,
}

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

    pub async fn config(&mut self) -> Result<Config, E> {
        let config = self.read_config().await?;

        let config = config.into();
        Ok(config)
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
    use super::*;
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
