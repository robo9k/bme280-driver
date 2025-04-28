#![forbid(unsafe_code)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![cfg_attr(docsrs, feature(doc_cfg_hide))]
#![cfg_attr(docsrs, doc(cfg_hide(docsrs)))]

use bitbybit::{bitenum, bitfield};
use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use thiserror::Error;

pub type ChipId = u8;
pub const CHIP_ID: ChipId = 0x60;

const REGISTER_ID_ADDRESS: SevenBitAddress = 0xD0;
const REGISTER_CTRL_HUM_ADDRESS: SevenBitAddress = 0xF2;
const REGISTER_CTRL_MEAS_ADDRESS: SevenBitAddress = 0xF4;
const REGISTER_CONFIG_ADDRESS: SevenBitAddress = 0xF5;

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

        let chip_id = this.chip_id().await?;
        if CHIP_ID == chip_id {
            Ok(this)
        } else {
            Err(ChipIdError::WrongChipId(chip_id))
        }
    }

    async fn chip_id(&mut self) -> Result<ChipId, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_ID_ADDRESS], &mut data)
            .await?;
        Ok(data[0])
    }

    pub async fn ctrl_hum(&mut self) -> Result<HumCtrl, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CTRL_HUM_ADDRESS], &mut data)
            .await?;
        let hum_ctrl = HumCtrl::new_with_raw_value(data[0]);
        Ok(hum_ctrl)
    }

    pub async fn ctrl_meas(&mut self) -> Result<MeasCtrl, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CTRL_MEAS_ADDRESS], &mut data)
            .await?;
        let meas_ctrl = MeasCtrl::new_with_raw_value(data[0]);
        Ok(meas_ctrl)
    }

    pub async fn config(&mut self) -> Result<Config, E> {
        let mut data: [u8; 1] = [0];
        self.i2c
            .write_read(self.address, &[REGISTER_CONFIG_ADDRESS], &mut data)
            .await?;
        let config = Config::new_with_raw_value(data[0]);
        Ok(config)
    }

    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[bitenum(u3, exhaustive = false)]
enum HumOs {
    Skip = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

#[bitfield(u8, default = 0x00)]
pub struct HumCtrl {
    #[bits(0..=2, rw)]
    hum_os: Option<HumOs>,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for HumCtrl {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "ctrl_hum {{ osrs_h {=0..3} }}", self.raw_value,)
    }
}

#[bitenum(u3, exhaustive = false)]
enum TmpOs {
    Skip = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

#[bitenum(u3, exhaustive = false)]
enum PrsOs {
    Skip = 0b000,
    X1 = 0b001,
    X2 = 0b010,
    X4 = 0b011,
    X8 = 0b100,
    X16 = 0b101,
}

#[bitenum(u2, exhaustive = false)]
enum Mode {
    Sleep = 0b00,
    Forced = 0b01,
    Normal = 0b11,
}

#[bitfield(u8, default = 0x00)]
pub struct MeasCtrl {
    #[bits(5..=7, rw)]
    tmp_os: Option<TmpOs>,

    #[bits(2..=4, rw)]
    prs_os: Option<PrsOs>,

    #[bits(0..=1, rw)]
    mode: Option<Mode>,
}

#[cfg(feature = "defmt-03")]
#[cfg_attr(docsrs, doc(cfg(feature = "defmt-03")))]
impl defmt::Format for MeasCtrl {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "ctrl_meas {{ osrs_t {0=5..8} osrs_p {0=2..5} mode {0=0..2} }}",
            self.raw_value,
        )
    }
}

#[bitenum(u3, exhaustive = true)]
enum SbTime {
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
enum Filter {
    Off = 0b000,
    X2 = 0b001,
    X4 = 0b010,
    X8 = 0b011,
    X16 = 0b100,
}

#[bitfield(u8, default = 0x00)]
pub struct Config {
    #[bits(5..=7, rw)]
    t_sb: SbTime,

    #[bits(2..=4, rw)]
    filter: Option<Filter>,

    #[bit(10, rw)]
    spi3w_en: bool,
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
