//! Cyclic redundancy check calculation unit
//!
//! Hardware CRC calculation with programmable polynomial (7, 8, 16, or 32-bit),
//! configurable initial value, input/output bit-reversal, and output XOR.
//! Feed data into the hardware with the update method and then read out the rolling CRC.
//! Defaults to the CRC-32 (Ethernet) polynomial `0x04C1_1DB7`.
//!
//! # Usage
//!
//! ```rust
//! let mut crc = dp.CRC.crc(ccdr.peripheral.CRC); // uses defaults
//!
//! crc.update(b"hello");
//! let checksum = crc.finish(); // reads CRC and resets
//!
//! // Custom configuration
//! let cfg = Config::new()
//!     .reverse_input(ReverseInput::WordReverse)
//!     .reverse_output(true)
//!     .initial_value(0xFFFF_FFFF)
//!     .output_xor(0xFFFF_FFFF);
//! crc.set_config(&cfg);
//! ```
//!
//! Also refer to examples/crc.rs
//!
use core::fmt;

use crate::stm32::{
    crc::cr::{POLYSIZE, REV_IN},
    CRC,
};

use crate::rcc::{rec, ResetEnable};

pub trait CrcExt {
    /// Enable the CRC unit.
    fn crc(self, prec: rec::Crc) -> Crc;
}

impl CrcExt for CRC {
    fn crc(self, prec: rec::Crc) -> Crc {
        prec.enable().reset();
        Crc {
            reg: self,
            output_xor: 0,
        }
    }
}

/// Hardware CRC unit
pub struct Crc {
    reg: CRC,
    output_xor: u32,
}

impl Crc {
    /// Set the unit's configuration, discarding previous state.
    pub fn set_config(&mut self, config: &Config) {
        self.output_xor = config.output_xor & config.poly.xor_mask();

        // The reference manual requires a reset (or DR read) before changing
        // polynomial value/size while a CRC calculation may be in progress.
        self.reg.cr().modify(|_, w| {
            w.polysize()
                .variant(config.poly.polysize())
                .rev_in()
                .variant(config.reverse_input.as_reg())
                .rev_out()
                .bit(config.reverse_output)
                .reset()
                .set_bit()
        });
        self.reg.pol().write(|w| w.pol().set(config.poly.pol()));
        // Writing CRC_INIT auto-reloads DR (RM0492 §18.3.2), so DR ends up
        // with the new initial value even though RESET already fired above.
        self.reg.init().write(|w| w.init().set(config.initial));
    }

    /// Write data to the CRC unit
    pub fn update(&mut self, data: &[u8]) {
        let mut words = data.chunks_exact(4);
        for word in words.by_ref() {
            let word: u32 = u32::from_be_bytes(word.try_into().unwrap());
            self.reg.dr().write(|w| w.dr().set(word));
        }

        let mut half_words = words.remainder().chunks_exact(2);
        if let Some(half_word) = half_words.next() {
            let half_word = u16::from_be_bytes(half_word.try_into().unwrap());
            self.reg.dr16().write(|w| w.dr16().set(half_word));
        }

        if let Some(byte) = half_words.remainder().first() {
            self.reg.dr8().write(|w| w.dr8().set(*byte));
        }
    }

    /// Write data to the CRC unit and read the updated CRC value in one step.
    #[must_use = "retrieving the CRC takes time, use update() if not needed"]
    pub fn update_and_read(&mut self, data: &[u8]) -> u32 {
        self.update(data);
        self.read_crc()
    }

    /// Read the CRC without resetting the unit.
    pub fn read_crc(&self) -> u32 {
        self.read_crc_no_xor() ^ self.output_xor
    }

    /// Read the state of the CRC calculation. When used as the initial value
    /// of an otherwise identical CRC config, this allows resuming calculation
    /// from the current state.
    ///
    /// This is equivalent to [`read_crc()`](Self::read_crc) in the case of an
    /// algorithm that does not apply an output XOR or reverse the output bits.
    pub fn read_state(&self) -> u32 {
        let state = self.read_crc_no_xor();
        if self.reg.cr().read().rev_out().is_reversed() {
            state.reverse_bits()
        } else {
            state
        }
    }

    /// Read the CRC without applying output XOR.
    #[inline(always)]
    fn read_crc_no_xor(&self) -> u32 {
        self.reg.dr().read().dr().bits()
    }

    /// Read the CRC and reset DR to initial value in preparation for a new CRC.
    /// This does not reset the configuration options.
    pub fn finish(&mut self) -> u32 {
        let result = self.read_crc();
        self.reset();
        result
    }

    /// Write to the independent data register
    /// This is "temporary storage location for four bytes" and is not affected by reset
    /// It is also not used in the CRC calculation
    pub fn set_idr(&mut self, value: u32) {
        self.reg.idr().write(|w| w.idr().set(value));
    }

    /// Read from the independent data register
    pub fn get_idr(&self) -> u32 {
        self.reg.idr().read().idr().bits()
    }

    /// Reset the CRC unit and set the value of the data register back to default
    /// This does not change the configuration
    pub fn reset(&mut self) {
        self.reg.cr().modify(|_, w| w.reset().set_bit());
    }
}

/// Configuration for the CRC unit
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    poly: Polynomial,
    initial: u32,
    reverse_input: ReverseInput,
    reverse_output: bool,
    output_xor: u32,
}

impl Config {
    pub const fn new() -> Self {
        Self {
            poly: Polynomial::bits32_unchecked(0x04C1_1DB7),
            initial: 0xFFFF_FFFF,
            reverse_input: ReverseInput::NoReverse,
            reverse_output: false,
            output_xor: 0,
        }
    }

    pub const fn polynomial(mut self, poly: Polynomial) -> Self {
        self.poly = poly;
        self
    }

    pub const fn polynomial_bits32(
        self,
        poly: u32,
    ) -> Result<Self, PolynomialError> {
        match Polynomial::bits32(poly) {
            Ok(poly) => Ok(self.polynomial(poly)),
            Err(e) => Err(e),
        }
    }

    pub const fn reverse_input(mut self, reverse: ReverseInput) -> Self {
        self.reverse_input = reverse;
        self
    }

    pub const fn reverse_output(mut self, reversed: bool) -> Self {
        self.reverse_output = reversed;
        self
    }

    pub const fn initial_value(mut self, value: u32) -> Self {
        self.initial = value;
        self
    }

    /// Final XOR value applied to the CRC result in software (not a hardware
    /// feature). Automatically masked to the polynomial width.
    pub const fn output_xor(mut self, output_xor: u32) -> Self {
        self.output_xor = output_xor;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Configuration value for control of the reversal of the bit order of the input data
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReverseInput {
    NoReverse,
    ByteReverse,
    HalfWordReverse,
    WordReverse,
}

impl ReverseInput {
    const fn as_reg(self) -> REV_IN {
        match self {
            Self::NoReverse => REV_IN::Normal,
            Self::ByteReverse => REV_IN::Byte,
            Self::HalfWordReverse => REV_IN::HalfWord,
            Self::WordReverse => REV_IN::Word,
        }
    }
}

/// A CRC polynomial with explicit width.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Polynomial(Poly);

impl Polynomial {
    pub const fn bits7(poly: u8) -> Result<Self, PolynomialError> {
        if poly > 0x7F {
            return Err(PolynomialError::TooLarge);
        }
        if poly.is_multiple_of(2) {
            return Err(PolynomialError::EvenPoly);
        }
        Ok(Self(Poly::B7(poly)))
    }

    pub const fn bits8(poly: u8) -> Result<Self, PolynomialError> {
        if poly.is_multiple_of(2) {
            return Err(PolynomialError::EvenPoly);
        }
        Ok(Self(Poly::B8(poly)))
    }

    pub const fn bits16(poly: u16) -> Result<Self, PolynomialError> {
        if poly.is_multiple_of(2) {
            return Err(PolynomialError::EvenPoly);
        }
        Ok(Self(Poly::B16(poly)))
    }

    pub const fn bits32(poly: u32) -> Result<Self, PolynomialError> {
        if poly.is_multiple_of(2) {
            return Err(PolynomialError::EvenPoly);
        }
        Ok(Self(Poly::B32(poly)))
    }

    pub const fn bits7_unchecked(poly: u8) -> Self {
        Self(Poly::B7(poly))
    }

    pub const fn bits8_unchecked(poly: u8) -> Self {
        Self(Poly::B8(poly))
    }

    pub const fn bits16_unchecked(poly: u16) -> Self {
        Self(Poly::B16(poly))
    }

    pub const fn bits32_unchecked(poly: u32) -> Self {
        Self(Poly::B32(poly))
    }

    const fn polysize(self) -> POLYSIZE {
        match self.0 {
            Poly::B7(_) => POLYSIZE::Polysize7,
            Poly::B8(_) => POLYSIZE::Polysize8,
            Poly::B16(_) => POLYSIZE::Polysize16,
            Poly::B32(_) => POLYSIZE::Polysize32,
        }
    }

    const fn pol(self) -> u32 {
        match self.0 {
            Poly::B7(pol) | Poly::B8(pol) => pol as u32,
            Poly::B16(pol) => pol as u32,
            Poly::B32(pol) => pol,
        }
    }

    const fn xor_mask(self) -> u32 {
        match self.0 {
            Poly::B7(_) => 0x7F,
            Poly::B8(_) => 0xFF,
            Poly::B16(_) => 0xFFFF,
            Poly::B32(_) => 0xFFFF_FFFF,
        }
    }
}

impl Default for Polynomial {
    fn default() -> Self {
        Self::bits32_unchecked(0x04C1_1DB7)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Poly {
    B7(u8),
    B8(u8),
    B16(u16),
    B32(u32),
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PolynomialError {
    EvenPoly,
    TooLarge,
}

impl fmt::Display for PolynomialError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EvenPoly => f.write_str("tried to create an even polynomial"),
            Self::TooLarge => f.write_str(
                "tried to create a 7-bit polynomial with an 8-bit number",
            ),
        }
    }
}
