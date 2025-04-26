#![forbid(unsafe_code)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![cfg_attr(docsrs, feature(doc_cfg_hide))]
#![cfg_attr(docsrs, doc(cfg_hide(docsrs)))]

use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use thiserror::Error;

pub type ChipId = u8;
pub const CHIP_ID: ChipId = 0x60;

const REGISTER_ID_ADDRESS: SevenBitAddress = 0xD0;

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
