//! Inter Integrated Circuit (I2C)
//!
//! # Terminology
//! This uses the updated Controller/Target terminology that replaces Master/Slave, as of v7 of the
//! I2C Spec. See https://www.nxp.com/docs/en/user-guide/UM10204.pdf
//!
//! # Usage
//!
//! ## Controller
//!
//! In the simplest case, the I2C can be initialized from the device peripheral
//! and GPIO pins:
//!
//! ```
//! let dp = ...;            // Device peripherals
//! let (scl, sda) = ...;    // GPIO pins
//!
//! let mut i2c = dp.I2C1.i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);
//! ```
//!
//! The GPIO pins should be supplied as a tuple in the following order:
//!
//! - Clock (SCL)
//! - Data (SDA)
//!
//! The driver exposes the controller functionality via the embedded-hal I2C
//! traits:
//!
//! ```
//! use embedded-hal::i2c::I2c;
//!
//! // Write, repeat start, read
//! let write = [0x11, 0x22, 0x33];
//! i2c.write(0x18, &write)?;
//!
//! // Read only
//! let mut read = [0u8; 4];
//! i2c.read(0x18, &mut read)?;
//!
//! // Write, repeat start, read
//! let write = [0x02];
//! let mut read = [0u8; 4];
//! i2c.write_read(0x18, &write, &mut read)?;
//! ```
//!
//! # Examples
//!
//! - [I2C controller simple example](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/i2c.rs)

use core::ops::{Deref, DerefMut};

use crate::rcc::{CoreClocks, ResetEnable};

use crate::stm32::{i2c1, i2c1::isr::ISRrs};

type Isr = stm32h5::R<ISRrs>;

use crate::time::Hertz;

mod hal;
mod i2c_def;

/// I2C Stop Configuration
///
/// Peripheral options for generating the STOP condition
#[derive(Copy, Clone, PartialEq, Eq)]
enum Stop {
    /// Repeat start mode: A repeat start condition will be generated at the
    /// end of the current operation when using the blocking API or when the
    /// repeat_start is called. Otherwise,this can be used to take manual
    /// control of generating stop conditions when using non-blocking APIs.
    RepeatStart,
    /// Automatic end mode: A STOP condition is automatically generated once the
    /// configured number of bytes have been transferred
    Automatic,
    /// Reload mode: this allows for reads/writes longer than 255 bytes,
    /// allowing for a reload after current operation (subsequent must be the
    /// same type of operation).
    Reload,
}

/// Direction of transfer
#[derive(Copy, Clone, PartialEq, Eq)]
enum Direction {
    /// Read operation
    Read,
    /// Write operation
    Write,
}

/// Addressing mode
#[derive(Copy, Clone, PartialEq, Eq)]
enum AddressMode {
    /// 7-bit addressing mode
    AddressMode7bit,
    /// 10-bit addressing mode
    AddressMode10bit,
}

/// I2C error
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// NACK received
    NotAcknowledge,
}

/// A trait to represent the SCL Pin of an I2C Port
pub trait PinScl<I2C> {}

/// A trait to represent the SDL Pin of an I2C Port
pub trait PinSda<I2C> {}

/// A trait to represent the collection of pins required for an I2C port
pub trait Pins<I2C> {}

impl<I2C, SCL, SDA> Pins<I2C> for (SCL, SDA)
where
    SCL: PinScl<I2C>,
    SDA: PinSda<I2C>,
{
}

pub trait Instance:
    crate::Sealed + Deref<Target = i2c1::RegisterBlock>
{
    type Rec: ResetEnable;

    #[doc(hidden)]
    fn ptr() -> *const i2c1::RegisterBlock;

    #[doc(hidden)]
    fn clock(clocks: &CoreClocks) -> Hertz;

    #[doc(hidden)]
    fn rec() -> Self::Rec;
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Inner<I2C> {
    i2c: I2C,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2c<I2C> {
    inner: Inner<I2C>,
}

impl<I2C> Deref for I2c<I2C> {
    type Target = Inner<I2C>;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<I2C> DerefMut for I2c<I2C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<I2C> Inner<I2C> {
    fn new(i2c: I2C) -> Self {
        Inner { i2c }
    }
}

pub trait I2cExt<I2C: Instance>: Sized {
    /// Create a I2c instance that is capable of Controller operation only
    fn i2c<P: Pins<I2C>>(
        self,
        _pins: P,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C>;

    /// Create a I2c instance that is capable of Controller operation only.
    /// This will not check that the pins are properly configured.
    fn i2c_unchecked(
        self,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C>;
}

impl<I2C: Instance> I2cExt<I2C> for I2C {
    fn i2c<P: Pins<I2C>>(
        self,
        _pins: P,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C> {
        I2c::new(self, frequency, rec, clocks)
    }

    fn i2c_unchecked(
        self,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C> {
        I2c::new(self, frequency, rec, clocks)
    }
}

fn calc_timing_params(ker_ck: u32, target_freq: u32) -> (u8, u8, u8, u8, u8) {
    // This timing derivation is taken directly from the stm32h7xx-hal implementation
    // (see https://github.com/stm32-rs/stm32h7xx-hal/blob/master/src/i2c.rs).
    //
    // The timing requirements for the I2C peripheral are quite complex and ST does not
    // provide clear instructions for deriving them, instead referring users from the
    // reference manual to use the STM32CubeMX tool to get the TIMINGR setting. The H7
    // implementation is well tested, so we'll stick with it.
    // Refer to RM0492 Rev 2 Sections 34.4.5 & 34.4.9 for timing details.
    //
    // t_I2CCLK = 1 / i2c_ker_ck
    // t_PRESC  = (PRESC + 1) * t_I2CCLK
    // t_SCLL   = (SCLL + 1) * t_PRESC
    // t_SCLH   = (SCLH + 1) * t_PRESC
    //
    // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
    // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
    let ratio = ker_ck / target_freq;

    // For the standard-mode configuration method, we must have a ratio of 4
    // or higher
    assert!(
        ratio >= 4,
        "i2c_ker_ck must be at least 4 times the bus frequency!"
    );

    let (presc_reg, scll, sclh, sdadel, scldel) = if target_freq > 100_000 {
        // Fast-mode (Fm) or Fast-mode Plus (Fm+)
        // here we pick SCLL + 1 = 2 * (SCLH + 1)

        // Prescaler, 96 ticks for sclh/scll. Round up then subtract 1
        let presc_reg = ((ratio - 1) / 96) as u8;
        // ratio < 2500 by pclk 250MHz max., therefore presc < 16

        // Actual precale value selected
        let presc = (presc_reg + 1) as u32;

        let sclh = ((ratio / presc) - 3) / 3;
        let scll = (2 * (sclh + 1)) - 1;

        let (sdadel, scldel) = if target_freq > 400_000 {
            // Fast-mode Plus (Fm+)
            assert!(ker_ck >= 17_000_000); // See table in datsheet

            let sdadel = ker_ck / 8_000_000 / presc;
            let scldel = ker_ck / 4_000_000 / presc - 1;

            (sdadel, scldel)
        } else {
            // Fast-mode (Fm)
            assert!(ker_ck >= 8_000_000); // See table in datsheet

            let sdadel = ker_ck / 3_000_000 / presc;
            let scldel = ker_ck / 1_000_000 / presc - 1;

            (sdadel, scldel)
        };

        (
            presc_reg,
            scll as u8,
            sclh as u8,
            sdadel as u8,
            scldel as u8,
        )
    } else {
        // Standard-mode (Sm)
        // here we pick SCLL = SCLH
        assert!(ker_ck >= 2_000_000); // See table in datsheet

        // Prescaler, 128 or 256 ticks for sclh/scll. Round up then
        // subtract 1
        let presc_reg = (ratio - 1)
            / if target_freq < 8000 {
                256
            } else if target_freq < 80_000 {
                128
            } else {
                64
            };
        let presc_reg = core::cmp::min(presc_reg, 15) as u8;

        // Actual prescale value selected
        let presc = (presc_reg + 1) as u32;

        let sclh = ((ratio / presc) - 2) / 2;
        let scll = sclh;

        // Speed check
        assert!(
            sclh < 256,
            "The I2C PCLK is too fast for this bus frequency!"
        );

        let sdadel = ker_ck / 2_000_000 / presc;
        let scldel = ker_ck / 500_000 / presc - 1;

        (
            presc_reg,
            scll as u8,
            sclh as u8,
            sdadel as u8,
            scldel as u8,
        )
    };

    // Sanity check
    assert!(presc_reg < 16);

    // Keep values within reasonable limits for fast per_ck
    let sdadel = core::cmp::max(sdadel, 1);
    let scldel = core::cmp::max(scldel, 4);

    let sdadel = core::cmp::min(sdadel, 15);
    let scldel = core::cmp::min(scldel, 15);

    (presc_reg, scll, sclh, sdadel, scldel)
}

impl<I2C: Instance> I2c<I2C> {
    /// Create and initialise a new I2C peripheral.
    ///
    /// The frequency of the I2C bus clock is specified by `frequency`.
    ///
    /// # Panics
    ///
    /// Panics if the ratio between `frequency` and the i2c_ker_ck
    /// is out of bounds. The acceptable range is [4, 8192].
    ///
    /// Panics if the `frequency` is too fast. The maximum is 1MHz.
    pub fn new(
        i2c: I2C,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> Self {
        let _ = rec.enable().reset();

        let freq: u32 = frequency.raw();

        // Maximum f_SCL for Fast-mode Plus (Fm+)
        assert!(freq <= 1_000_000);

        let i2c_ker_ck: u32 = I2C::clock(clocks).raw();

        // Clear PE bit in I2C_CR1
        i2c.cr1().modify(|_, w| w.pe().disabled());

        // Configure timing
        let (presc_reg, scll, sclh, sdadel, scldel) =
            calc_timing_params(i2c_ker_ck, freq);
        i2c.timingr().write(|w| {
            w.presc()
                .set(presc_reg)
                .scll()
                .set(scll)
                .sclh()
                .set(sclh)
                .sdadel()
                .set(sdadel)
                .scldel()
                .set(scldel)
        });

        // Enable the peripheral and analog filter
        i2c.cr1().write(|w| w.pe().enabled().anfoff().enabled());

        I2c {
            inner: Inner::new(i2c),
        }
    }

    /// Reset the peripheral
    pub fn reset(&mut self) {
        self.i2c.cr1().modify(|_, w| w.pe().disabled());
        interrupt_clear_clock_sync_delay!(self.i2c.cr1());
        while self.i2c.cr1().read().pe().is_enabled() {}
        self.i2c.cr1().modify(|_, w| w.pe().enabled());
    }

    pub fn free(mut self) -> I2C {
        self.reset();
        self.inner.i2c
    }
}

impl<I2C: Instance> Inner<I2C> {
    #[inline(always)]
    fn flush_txdr(&self) {
        // If a pending TXIS flag is set, write dummy data to TXDR
        if self.i2c.isr().read().txis().bit_is_set() {
            self.i2c.txdr().write(|w| w.txdata().set(0));
        }

        // If TXDR is not flagged as empty, write 1 to flush it
        if self.i2c.isr().read().txe().is_not_empty() {
            self.i2c.isr().write(|w| w.txe().set_bit());
        }
    }

    #[inline(always)]
    fn read_isr_and_check_errors(&self) -> Result<Isr, Error> {
        let isr = self.i2c.isr().read();
        if isr.berr().is_error() {
            self.i2c.icr().write(|w| w.berrcf().clear());
            return Err(Error::Bus);
        } else if isr.arlo().is_lost() {
            self.i2c.icr().write(|w| w.arlocf().clear());
            return Err(Error::Arbitration);
        }
        Ok(isr)
    }

    #[inline(always)]
    fn check_clear_target_nack(&self, isr: &Isr) -> Result<(), Error> {
        if isr.nackf().is_nack() {
            self.i2c.icr().write(|w| w.nackcf().clear());
            self.flush_txdr();
            Err(Error::NotAcknowledge)
        } else {
            Ok(())
        }
    }

    /// Write a single byte if possible. If data was written, Ok(true) is
    /// returned. If the peripheral is not yet ready, but no error occurred,
    /// Ok(false) is returned.
    ///
    /// If a bus error occurs it will be returned and a write will not be
    /// attempted. If a previous byte was Nack'd by the receiver, that is
    /// indicated by a NotAcknowledge error being returned.
    #[inline(always)]
    fn write_byte_if_ready(&self, data: u8) -> Result<bool, Error> {
        let isr = self.read_isr_and_check_errors()?;
        self.check_clear_target_nack(&isr)?;

        if isr.txis().is_empty() {
            self.i2c.txdr().write(|w| w.txdata().set(data));
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Blocks until data can be written and writes it.
    ///
    /// If a bus error occurs it will be returned and a write will not be
    /// attempted. If a previous byte was Nack'd by the receiver, that is
    /// indicated by a NotAcknowledge error being returned.
    fn write_byte(&mut self, data: u8) -> Result<(), Error> {
        while !self.write_byte_if_ready(data)? {}
        Ok(())
    }

    /// Read a single byte if one is available. If data was read, Ok(true) is
    /// returned and the value read is stored to `data`. If the peripheral is
    /// not yet ready, but no error occurred, Ok(false) is returned.
    ///
    /// If a bus error occurs it will be returned and a read will not be
    /// attempted. In Target mode, if the operation is stopped by the
    /// Controller, a TransferStopped error is returned.
    #[inline(always)]
    fn read_byte_if_ready(&self, data: &mut u8) -> Result<bool, Error> {
        let isr = self.read_isr_and_check_errors()?;

        if isr.rxne().is_not_empty() {
            *data = self.i2c.rxdr().read().rxdata().bits();
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Blocks until data is available and returns it
    ///
    /// If a bus error occurs it will be returned and a read will not be
    /// attempted. In Target mode, if the operation is stopped by the
    /// Controller, a TransferStopped error is returned.
    fn read_byte(&mut self) -> Result<u8, Error> {
        let mut data = 0u8;
        while !self.read_byte_if_ready(&mut data)? {}
        Ok(data)
    }
}

/// Controller methods
///
/// These infallible methods are used to begin or end parts of
/// transactions, but do __not__ read or write the data
/// registers. If you want to perform an entire transcation see the
/// [Read](I2c#impl-Read) and [Write](I2c#impl-Write)
/// implementations.
///
/// If a previous transcation is still in progress, then these
/// methods will block until that transcation is complete. A
/// previous transaction can still be "in progress" up to 50% of a
/// bus cycle after a ACK/NACK event. Otherwise these methods return
/// immediately.
impl<I2C: Instance> I2c<I2C> {
    /// Start read transaction
    ///
    /// Perform an I2C start prepare peripheral to perform subsequent operation defined by
    /// the length, direction and stop parameters.
    ///
    /// Setting stop to Stop::Automatic means that no subsequent operations will be performed
    /// after the next one. Setting it to Stop::RepeatStart indicates that a further operation will
    /// be performed and it will require a repeated start condition. Setting it to Reload will
    /// indicate that the same operation will be performed subsequently.
    ///
    /// ```
    /// Controller: ST SAD+R  ...  (SP)
    /// Target:               ...
    /// ```
    fn start(
        &self,
        addr: u16,
        address_mode: AddressMode,
        length: usize,
        direction: Direction,
        stop: Stop,
    ) {
        assert!(length < 256 && length > 0);

        // Set START condition and
        self.i2c.cr2().write(|w| {
            w.rd_wrn()
                .bit(direction == Direction::Read)
                .nbytes()
                .set(length as u8)
                .start()
                .start()
                .autoend()
                .bit(stop == Stop::Automatic)
                .reload()
                .bit(stop == Stop::Reload)
                .add10()
                .bit(address_mode == AddressMode::AddressMode10bit);
            if address_mode == AddressMode::AddressMode10bit {
                w.sadd().set(addr).head10r().complete();
            } else {
                w.sadd().set(addr << 1);
            }
            w
        });
    }

    /// Send a repeat start condition. This should only be called if the previous call to
    /// start/repeat_start indicated that a repeat start would follow (by passing Stop::RepeatStart)
    ///
    /// It is erroneous to call this function after passing anything other than Stop::RepeatStart
    /// to a previous call.
    ///
    /// See the sequence diagrams in Figures 390 & 393 of RM0492 Rev 2 for more.
    ///
    fn repeat_start(
        &self,
        addr: u16,
        address_mode: AddressMode,
        length: usize,
        direction: Direction,
        stop: Stop,
    ) {
        self.i2c.cr2().write(|w| {
            w.rd_wrn()
                .bit(direction == Direction::Read)
                .nbytes()
                .set(length as u8)
                .start()
                .start()
                .autoend()
                .bit(stop == Stop::Automatic)
                .reload()
                .bit(stop == Stop::Reload)
                .add10()
                .bit(address_mode == AddressMode::AddressMode10bit);
            if address_mode == AddressMode::AddressMode10bit {
                w.sadd().set(addr).head10r().partial();
            } else {
                w.sadd().set(addr << 1);
            }
            w
        });
    }

    /// Continue a transaction without issuing a repeat start condition. This allows for more than
    /// 255 bytes to be written or read during an operation.
    ///
    /// It is erroneous to call this function after passing anything other than Stop::Reload to a
    /// previous call of start/repeat_start
    ///
    /// See the sequence diagrams in Figures 390 & 393 of RM0492 Rev 2 for more.
    fn reload(&self, length: usize, direction: Direction, stop: Stop) {
        self.i2c.cr2().write(|w| {
            w.rd_wrn()
                .bit(direction == Direction::Read)
                .nbytes()
                .set(length as u8)
                .autoend()
                .bit(stop == Stop::Automatic)
                .reload()
                .bit(stop == Stop::Reload)
        });
    }
}

impl<I2C: Instance> I2c<I2C> {
    /// Check whether start sequence has completed.
    #[inline(always)]
    fn is_start_sequence_complete(&self) -> Result<bool, Error> {
        let isr = self.read_isr_and_check_errors()?; // just check for errors
        self.check_clear_target_nack(&isr)?;

        if self.i2c.cr2().read().start().is_no_start() {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Blocks until start sequence has completed
    fn wait_for_start_sequence_complete(&self) -> Result<(), Error> {
        while !self.is_start_sequence_complete()? {}
        Ok(())
    }

    /// Check whether an operation has completed
    #[inline(always)]
    fn is_stopped(&self) -> Result<bool, Error> {
        let isr = self.read_isr_and_check_errors()?;
        self.check_clear_target_nack(&isr)?;

        if isr.stopf().is_stop() {
            self.i2c.icr().write(|w| w.stopcf().clear());
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Blocks until the operation completes
    fn wait_for_stop(&self) -> Result<(), Error> {
        while !self.is_stopped()? {}
        Ok(())
    }

    /// Check whether peripheral is ready for reload
    #[inline(always)]
    fn is_reload_ready(&self) -> Result<bool, Error> {
        let isr = self.read_isr_and_check_errors()?;
        self.check_clear_target_nack(&isr)?;
        if isr.tcr().is_complete() {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Blocks until the peripheral is ready for reload.
    fn wait_for_reload_ready(&self) -> Result<(), Error> {
        while !self.is_reload_ready()? {}
        Ok(())
    }

    /// Check whether all bytes have been transmitted before issuing repeat
    /// start.
    #[inline(always)]
    fn is_transmit_complete(&self) -> Result<bool, Error> {
        let isr = self.read_isr_and_check_errors()?;
        self.check_clear_target_nack(&isr)?;

        if isr.tc().is_complete() {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Blocks until all bytes have been transmitted
    fn wait_for_transmit_complete(&self) -> Result<(), Error> {
        while !self.is_transmit_complete()? {}
        Ok(())
    }

    /// Write the contents of the buffer. Blocks until all data has been
    /// written, or an error occurs.
    fn write_all(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for byte in bytes {
            // Blocks until we are allowed to send data (START has been ACKed or last byte when
            // through)
            self.write_byte(*byte)?;
        }
        Ok(())
    }

    /// Fill the buffer with data read from the bus. Blocks until all data
    /// have been read or an error occurs.
    fn read_all(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        for byte in buffer {
            *byte = self.read_byte()?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::i2c::calc_timing_params;

    /// Runs a timing testcase over PCLK and I2C clock ranges
    fn i2c_timing_testcase<F>(f: F)
    where
        F: Fn(u32, u32),
    {
        let i2c_timing_tests = [
            // (i2c_clk, range of bus frequencies to test)
            (2_000_000, (4_000..=100_000)), // Min PCLK
            (8_000_000, (4_000..=400_000)), // Slowest PCLK for fast mode
            (16_000_000, (4_000..=400_000)), // Default H7 PCLK = 16MHz
            (32_000_000, (4_000..=1_000_000)),
            (79_876_135, (10_000..=1_000_000)),
            (100_000_000, (13_000..=1_000_000)),
            (120_000_000, (15_000..=1_000_000)), // Max PCLK
        ];

        for (clock, freq_range) in i2c_timing_tests.iter() {
            for freq in freq_range.clone().step_by(1_000) {
                f(*clock, freq)
            }
        }
    }

    #[test]
    /// Test the SCL frequency is within the expected range
    fn i2c_frequency() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, scll, sclh, _, _) =
                calc_timing_params(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;

            // Estimate minimum sync times. Analog filter on, 2 i2c_clk cycles
            let t_af_min = 50e-9_f32; // Analog filter 50ns. From H7 Datasheet
            let t_sync1 = t_af_min + 2. * t_i2c_clk;
            let t_sync2 = t_af_min + 2. * t_i2c_clk;

            // See RM0492 Rev 2 Section 34.4.9
            let t_high_low = sclh as f32 + 1. + scll as f32 + 1.;
            let t_scl = t_sync1 + t_sync2 + (t_high_low * presc * t_i2c_clk);
            let f_scl = 1. / t_scl;

            let error = (freq - f_scl) / freq;
            println!(
                "Set SCL = {} Actual = {} Error {:.1}%",
                freq,
                f_scl,
                100. * error
            );

            // We must generate a bus frequency less than or equal to that
            // specified. Tolerate a 2% error
            assert!(f_scl <= 1.02 * freq);

            // But it should not be too much less than specified
            assert!(f_scl > 0.8 * freq);
        });
    }

    #[test]
    /// Test that the low period of SCL is greater than the minimum specification
    fn i2c_scl_low() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, scll, _, _, _) = calc_timing_params(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_scll = (scll as f32 + 1.) * presc * t_i2c_clk;

            // From I2C Specification Table 10
            //
            // UM10204 rev 6.: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
            let t_scll_minimum = match freq {
                x if x <= 100_000. => 4.7e-6, // Standard mode (Sm)
                x if x <= 400_000. => 1.3e-6, // Fast mode (Fm)
                _ => 0.5e-6,                  // Fast mode Plus (Fm+)
            };

            println!("Target {} Hz; SCLL {}", freq, scll);
            println!(
                "T SCL LOW {:.2e}; MINIMUM {:.2e}",
                t_scll, t_scll_minimum
            );
            assert!(t_scll >= t_scll_minimum);
        });
    }

    #[test]
    /// Test the SDADEL value is greater than the minimum specification
    fn i2c_sdadel_minimum() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, _, _, sdadel, _) =
                calc_timing_params(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_sdadel = (sdadel as f32) * presc * t_i2c_clk;

            // From I2C Specification Table 10
            //
            // UM10204 rev 6.: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
            let t_fall_max = match freq {
                x if x <= 100_000. => 300e-9, // Standard mode (Sm)
                x if x <= 400_000. => 300e-9, // Fast mode (Fm)
                _ => 120e-9,                  // Fast mode Plus (Fm+)
            };

            let t_af_min = 50e-9_f32; // Analog filter min 50ns. From H7 Datasheet
            let hddat_min = 0.;

            // From RM0492 Rev 2 Section 34.4.5
            //
            // tSDADEL >= {tf + tHD;DAT(min) - tAF(min) - [(DNF + 3) x tI2CCLK]}
            let t_sdadel_minimim =
                t_fall_max + hddat_min - t_af_min - (3. * t_i2c_clk);

            println!("Target {} Hz; SDADEL {}", freq, sdadel);
            println!(
                "T SDA DELAY {:.2e} MINIMUM {:.2e}",
                t_sdadel, t_sdadel_minimim
            );
            assert!(sdadel <= 15);
            assert!(t_sdadel >= t_sdadel_minimim);
        });
    }

    #[test]
    /// Test the SDADEL value is less than the maximum specification
    fn i2c_sdadel_maximum() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, _, _, sdadel, _) =
                calc_timing_params(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_sdadel = (sdadel as f32) * presc * t_i2c_clk;

            let t_hddat_max = match freq {
                x if x <= 100_000. => 3.45e-6, // Standard mode (Sm)
                x if x <= 400_000. => 0.9e-6,  // Fast mode (Fm)
                _ => 0.45e-6,                  // Fast mode Plus (Fm+)
            };
            let t_af_max = 80e-9_f32; // Analog filter max 80ns. From H7 Datasheet

            // From RM0492 Rev 2 Section 34.4.5
            //
            // tSDADEL <= {tHD;DAT(max) - tAF(max) - [(DNF + 4) x tI2CCLK]}
            let t_sdadel_maximum = t_hddat_max - t_af_max - (4. * t_i2c_clk);

            println!("Target {} Hz; SDADEL {}", freq, sdadel);
            println!(
                "T SDA DELAY {:.2e} MAXIMUM {:.2e}",
                t_sdadel, t_sdadel_maximum
            );
            assert!(sdadel <= 15);
            assert!(t_sdadel <= t_sdadel_maximum);
        });
    }

    #[test]
    /// Test the SCLDEL value is greater than the minimum specification
    fn i2c_scldel_minimum() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, _, _, _, scldel) =
                calc_timing_params(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_scldel = (scldel as f32) * presc * t_i2c_clk;

            // From I2C Specification Table 10
            //
            // UM10204 rev 6.: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
            let t_rise_max = match freq {
                x if x <= 100_000. => 1000e-9, // Standard mode (Sm)
                x if x <= 400_000. => 300e-9,  // Fast mode (Fm)
                _ => 120e-9,                   // Fast mode Plus (Fm+)
            };
            let t_sudat_min = match freq {
                x if x <= 100_000. => 250e-9, // Standard mode (Sm)
                x if x <= 400_000. => 100e-9, // Fast mode (Fm)
                _ => 50e-9,                   // Fast mode Plus (Fm+)
            };

            // From RM0492 Rev 2 Section 34.4.5
            //
            // tSCLDEL >= tr + tSU;DAT(min)
            let t_scldel_minimum = t_rise_max + t_sudat_min;

            println!("Target {} Hz; SCLDEL {}", freq, scldel);
            println!(
                "T SCL DELAY {:.2e} MINIMUM {:.2e}",
                t_scldel, t_scldel_minimum
            );
            assert!(scldel <= 15);
            assert!(t_scldel >= t_scldel_minimum);
        });
    }
}
