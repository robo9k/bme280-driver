#![forbid(unsafe_code)]
#![cfg_attr(not(any(test, feature = "std")), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![cfg_attr(docsrs, feature(doc_cfg_hide))]
#![cfg_attr(docsrs, doc(cfg_hide(docsrs)))]

use embedded_hal_async::i2c::{I2c, SevenBitAddress};

pub struct Bme280<I2c> {
    i2c: I2c,
    address: SevenBitAddress,
}

impl<I2C> Bme280<I2C>
where
    I2C: I2c,
{
    // TODO: should this be fallible and check the chip_id?
    // calibration should be done in e.g. .init() and involve typestate
    pub fn new_with_address(i2c: I2C, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
