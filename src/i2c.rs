//! Inter Integrated Circuit (I2C)
//!
//! This module provides I2C functionality as both a Controller and Target, and
//! supports multi-controller topologies through role-switching between
//! Controller and Target. The Controller implementation is exposed via the I2c
//! struct, while the Target implementation is exposed via the I2cTarget
//! struct.
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
//! ## Target
//! In the simplest case, the I2cTarget can be initialized from the device peripheral
//! and GPIO pins:
//!
//! ```
//! let dp = ...;            // Device peripherals
//! let (scl, sda) = ...;    // GPIO pins
//! let own_addr = ...;      // Primary address for target operation
//!
//! let mut i2c_target = dp.I2C1.i2c_target_only(
//!     (scl, sda),
//!     own_addr,
//!     ccdr.peripheral.I2C1,
//! );
//! ```
//!
//! The target (or controller when role switching is allowed) must be
//! instructed to respond to specific TargetListenEvents using the
//! `enable_target_event` function:
//!
//! ```
//! i2c_target.enable_target_event(TargetListenEvent::PrimaryAddress);
//! ```
//!
//! Use the `get_target_event_nb` (non-blocking) or `wait_for_event` (blocking)
//! functions to get target events to which to respond. Then use the provided
//! `read`/`write`` (blocking) or `read_nb`/`write_nb`(non-blocking) functions
//! to respond to the controller:
//!
//! ```
//! let mut buffer = [0u8; 10];
//! match i2c.wait_for_event().unwrap() {
//!     TargetEvent::Read { address: _ } => i2c.write(...),
//!     TargetEvent::Write { address: _ } => i2c.read(&mut buffer),
//!     TargetEvent::Stop => Ok(0),
//! };
//! ```
//!
//! ### Manual ACK control in receive mode
//!
//! To gain control over ACK'ing received data when operating as a target, convert to
//! manual ACK control with:
//!
//! ```
//! let mut i2c_target_manual_ack = i2c_target.with_manual_ack_control();
//! ```
//!
//! This provides the methods `I2cTarget::ack_transfer` and
//! `I2cTarget::nack_transfer` to end a transfer of an expected number of bytes
//! with an ACK or NACK, respectively:
//!
//! ```
//! let mut buf = [0u8; 10];
//! i2c_target_manual_ack.read(&mut buf)?;  // Read 10 bytes, ACK each byte except the last.
//! // Check something
//! if good {
//!     i2c_target_manual_ack.ack_transfer()
//! } else {
//!     i2c_target_manual_ack.nack_transfer()
//! }
//! ```
//!
//! ## Switching between target and controller
//!
//! Use the provided initialization functions to create an I2C driver that can switch
//! between controller and target operation:
//!
//! ```
//! let dp = ...;            // Device peripherals
//! let (scl, sda) = ...;    // GPIO pins
//! let own_addr = ...;      // Primary address for target operation
//!
//! let mut i2c = dp.I2C1.i2c_controller_target(
//!     (scl, sda),
//!     own_addr,
//!     ccdr.peripheral.I2C1,
//! );
//! ```
//!
//! To switch operating modes, use the provided functions:
//!
//! ```
//! let i2c_target = i2c.to_target();
//! ...
//! let i2c = i2c_target.to_controller();
//! ```
//!
//! These functions are only available on driver instances created with the
//! above initialization function.
//!
//! # Examples
//!
//! - [I2C controller simple example](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/i2c.rs)
//! - [I2C Target simple example](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/i2c_target.rs)
//! - [I2C Target with manual ACK control](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/i2c_target_manual_ack.rs)

use core::iter;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};

use crate::rcc::{CoreClocks, ResetEnable};

use crate::stm32::{i2c1, i2c1::isr::ISRrs};

type Isr = stm32h5::R<ISRrs>;

use crate::time::Hertz;

pub mod config;
pub use config::TargetConfig;

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
pub enum AddressMode {
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
    /// Target operation only:
    /// Indicates that a stop or repeat start was received while reading, or
    /// while explicitly waiting for a controller read or write event.
    TransferStopped,
    /// Target operation only:
    /// While waiting for a controller read event, a write event was received.
    ControllerExpectedWrite,
    /// Target operation only:
    /// While waiting for a controller write event, a read event was received.
    ControllerExpectedRead,
}

/// Target Event.
///
/// This encapsulates a transaction event that occurs when listening in Target
/// operation. A Read event indicates that the Controller wants to read
/// data from the Target and the target must write to the bus.
/// A Write event indicates that the Controller wants to write data to the
/// Target and the target must read data from the bus.
/// A Stop event indicates that a Stop condition was received and the transaction has been completed.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TargetEvent {
    /// The controller initiated an I2C Read operation, requiring that the Target must write to the
    /// bus.
    Read { address: u16 },
    /// The controller initiated an I2C Write operation, requiring that the Target must read from
    /// the bus.
    Write { address: u16 },
    /// A Stop condition was received, ending the transaction.
    Stop,
}

/// Target Listen Event
///
/// Indicates what listen events are responded to. A target can respond to one
/// of or all of a general call address, the primary address configured or a
/// secondary address.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum TargetListenEvent {
    /// The General Call address (0x0) is reserved for specific broadcasts in the I2C
    /// standard. Enabling the general call event will enable the target to receive
    /// General Call broadcasts from the bus controller
    GeneralCall,
    /// Primary address of the target. A single address specific to this target. Can
    /// be 7- or 10-bit.
    PrimaryAddress,
    /// The Secondary address event is a match against a 7-bit address range governed
    /// by an associated mask. See the `Config` for more.
    SecondaryAddress,
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

/// Marker struct for an I2c/I2cTarget that can switch between Controller and Target operation
pub struct SwitchRole;

/// Marker struct for an I2c/I2cTarget that cannot switch between Controller and Target operation
pub struct SingleRole;

/// Marker struct for I2cTarget implementation to indicate that it allows for manual ACK control
pub struct ManualAck;

/// Marker struct for I2cTarget implementation to indicate that it automatically handles all ACK'ing
pub struct AutoAck;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2c<I2C, R = SingleRole> {
    inner: Inner<I2C>,
    _role: PhantomData<R>,
}

impl<I2C, R> Deref for I2c<I2C, R> {
    type Target = Inner<I2C>;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<I2C, R> DerefMut for I2c<I2C, R> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

pub struct I2cTarget<I2C, A = AutoAck, R = SingleRole> {
    inner: Inner<I2C>,
    _role: PhantomData<R>,
    _ack: PhantomData<A>,
}

impl<I2C, A, R> Deref for I2cTarget<I2C, A, R> {
    type Target = Inner<I2C>;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<I2C, A, R> DerefMut for I2cTarget<I2C, A, R> {
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
    ) -> I2c<I2C, SingleRole>;

    /// Create a I2c instance that is capable of Controller operation only.
    /// This will not check that the pins are properly configured.
    fn i2c_unchecked(
        self,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SingleRole>;

    /// Create an I2cTarget instance capable of Target operation only
    fn i2c_target_only<P: Pins<I2C>>(
        self,
        _pins: P,
        target_config: impl Into<TargetConfig>,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2cTarget<I2C, AutoAck, SingleRole>;

    /// Create an I2cTarget instance capable of Target operation only.
    /// This will not check that the pins are properly configured.
    fn i2c_target_only_unchecked<P: Pins<I2C>>(
        self,
        target_config: impl Into<TargetConfig>,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2cTarget<I2C, AutoAck, SingleRole>;

    /// Create an I2c instance that can switch roles to a I2cTarget to perform
    /// Target operations
    fn i2c_controller_target<P: Pins<I2C>>(
        self,
        _pins: P,
        frequency: Hertz,
        target_config: TargetConfig,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SwitchRole>;

    /// Create an I2c instance that can switch roles to a I2cTarget to perform
    /// Target operations.
    /// This will not check that the pins are properly configured.
    fn i2c_controller_target_unchecked(
        self,
        frequency: Hertz,
        target_config: TargetConfig,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SwitchRole>;
}

impl<I2C: Instance> I2cExt<I2C> for I2C {
    fn i2c<P: Pins<I2C>>(
        self,
        _pins: P,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SingleRole> {
        I2c::new(self, frequency, None::<TargetConfig>, rec, clocks)
    }

    fn i2c_unchecked(
        self,
        frequency: Hertz,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SingleRole> {
        I2c::new(self, frequency, None::<TargetConfig>, rec, clocks)
    }

    fn i2c_target_only<P: Pins<I2C>>(
        self,
        _pins: P,
        config: impl Into<TargetConfig>,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2cTarget<I2C, AutoAck, SingleRole> {
        I2cTarget::new(self, config, rec, clocks)
    }

    fn i2c_target_only_unchecked<P: Pins<I2C>>(
        self,
        target_config: impl Into<TargetConfig>,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2cTarget<I2C, AutoAck, SingleRole> {
        I2cTarget::new(self, target_config, rec, clocks)
    }

    fn i2c_controller_target<P: Pins<I2C>>(
        self,
        _pins: P,
        frequency: Hertz,
        target_config: TargetConfig,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SwitchRole> {
        I2c::new(self, frequency, Some(target_config), rec, clocks)
    }

    fn i2c_controller_target_unchecked(
        self,
        frequency: Hertz,
        target_config: TargetConfig,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C, SwitchRole> {
        I2c::new(self, frequency, Some(target_config), rec, clocks)
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
            assert!(ker_ck >= 17_000_000); // See table in datasheet

            let sdadel = ker_ck / 8_000_000 / presc;
            let scldel = ker_ck / 4_000_000 / presc - 1;

            (sdadel, scldel)
        } else {
            // Fast-mode (Fm)
            assert!(ker_ck >= 8_000_000); // See table in datasheet

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

fn configure_target_addresses<I2C: Instance>(i2c: &I2C, config: TargetConfig) {
    i2c.oar1().write(|w| match config.own_address_mode {
        AddressMode::AddressMode7bit => {
            w.oa1().set(config.own_address << 1).oa1mode().bit7()
        }
        AddressMode::AddressMode10bit => {
            w.oa1().set(config.own_address).oa1mode().bit10()
        }
    });

    if let Some(secondary_address) = config.secondary_address {
        i2c.oar2().write(|w| {
            w.oa2().set(secondary_address);
            if let Some(mask) = config.secondary_address_mask_bits {
                w.oa2msk().set(mask + 1); // The address is shifted up by one, so increment the mask bits too
            }
            w
        });
    }
}

impl<I2C: Instance, R> I2c<I2C, R> {
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
        config: Option<impl Into<TargetConfig>>,
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

        if let Some(config) = config {
            let config = config.into();
            configure_target_addresses(&i2c, config);
        }

        // Enable the peripheral and analog filter
        i2c.cr1().write(|w| w.pe().enabled().anfoff().enabled());

        I2c {
            inner: Inner::new(i2c),
            _role: PhantomData,
        }
    }

    pub fn free(self) -> I2C {
        let _ = I2C::rec().reset().disable();
        self.inner.i2c
    }
}

/// Target implementation
impl<I2C: Instance, A, R> I2cTarget<I2C, A, R> {
    fn new(
        i2c: I2C,
        target_config: impl Into<TargetConfig>,
        rec: I2C::Rec,
        clocks: &CoreClocks,
    ) -> Self {
        let config = target_config.into();

        let _ = rec.enable().reset();

        // Clear PE bit in I2C_CR1
        i2c.cr1().modify(|_, w| w.pe().disabled());

        let i2c_ker_ck: u32 = I2C::clock(clocks).raw();

        // Configure timing parameters for target mode
        let (presc_reg, _, _, sdadel, scldel) =
            calc_timing_params(i2c_ker_ck, config.bus_frequency_hz);
        i2c.timingr().write(|w| {
            w.presc()
                .set(presc_reg)
                .sdadel()
                .set(sdadel)
                .scldel()
                .set(scldel)
        });

        configure_target_addresses(&i2c, config);

        // Enable the peripheral and Analog Noise Filter
        i2c.cr1().modify(|_, w| w.pe().enabled().anfoff().enabled());

        Self {
            inner: Inner::new(i2c),
            _role: PhantomData,
            _ack: PhantomData,
        }
    }

    /// Releases the I2C peripheral
    pub fn free(self) -> I2C {
        let _ = I2C::rec().reset().disable();
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
    /// attempted. If a previous byte was NACK'd by the receiver, that is
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
    /// attempted. If a previous byte was NACK'd by the receiver, that is
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
        } else if isr.stopf().is_stop() || isr.addr().is_match() {
            // This is only relevant to Target operation, when the controller stops the read
            // operation with a Stop or Restart condition.
            Err(Error::TransferStopped)
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

    fn enable_target_event(&mut self, event: TargetListenEvent) {
        match event {
            TargetListenEvent::GeneralCall => {
                self.i2c.cr1().modify(|_, w| w.gcen().enabled());
            }
            TargetListenEvent::PrimaryAddress => {
                self.i2c.oar1().modify(|_, w| w.oa1en().enabled());
            }
            TargetListenEvent::SecondaryAddress => {
                self.i2c.oar2().modify(|_, w| w.oa2en().enabled());
            }
        }
    }

    fn disable_target_event(&mut self, event: TargetListenEvent) {
        match event {
            TargetListenEvent::GeneralCall => {
                self.i2c.cr1().modify(|_, w| w.gcen().disabled());
            }
            TargetListenEvent::PrimaryAddress => {
                self.i2c.oar1().modify(|_, w| w.oa1en().disabled());
            }
            TargetListenEvent::SecondaryAddress => {
                self.i2c.oar2().modify(|_, w| w.oa2en().disabled());
            }
        }
    }

    /// Reset the peripheral
    pub fn reset(&mut self) {
        self.i2c.cr1().modify(|_, w| w.pe().disabled());
        interrupt_clear_clock_sync_delay!(self.i2c.cr1());
        while self.i2c.cr1().read().pe().is_enabled() {}
        self.i2c.cr1().modify(|_, w| w.pe().enabled());
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
impl<I2C: Instance, R> I2c<I2C, R> {
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
        assert!(
            length <= u8::MAX as usize,
            "I2C max transaction size = {} bytes",
            u8::MAX
        );

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
        assert!(
            length <= u8::MAX as usize,
            "I2C max transaction size = {} bytes",
            u8::MAX
        );

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
        assert!(
            length <= u8::MAX as usize,
            "I2C max transfer size per reload = {}",
            u8::MAX
        );

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

impl<I2C: Instance, R> I2c<I2C, R> {
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

/// I2C Target common blocking operations
impl<I2C: Instance, A, R> I2cTarget<I2C, A, R> {
    /// While operating with automatic ACK control, this will indicate to the
    /// peripheral that the next byte received should be NACK'd (the last
    /// received was already ACK'd). The peripheral only NACKs if a byte is
    /// received. If a control signal is received, the peripheral will respond
    /// correctly.
    ///
    /// With manual ACK control, this will NACK the last received byte.
    fn nack(&mut self) {
        self.i2c.cr2().modify(|_, w| w.nack().nack());
    }

    fn read_all(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        assert!(!buffer.is_empty());

        for (i, byte) in buffer.iter_mut().enumerate() {
            match self.read_byte() {
                Ok(data) => {
                    *byte = data;
                }
                Err(Error::TransferStopped) => return Ok(i),
                Err(error) => return Err(error),
            };
        }
        Ok(buffer.len())
    }

    fn write_buf(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        assert!(!bytes.is_empty());

        for (i, data) in bytes.iter().enumerate() {
            match self.write_byte(*data) {
                Ok(()) => {}
                // If we receive a NACK it will be on the FIFO write subsequent to the last byte
                // actually written to the bus
                Err(Error::NotAcknowledge) => return Ok(i - 1),
                Err(error) => return Err(error),
            }
        }
        Ok(bytes.len())
    }

    fn write_buf_fill_zeroes(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        assert!(!bytes.is_empty());

        // The controller can try to read more data than the target has to write, so we write
        // zeroes until the controller stops the transaction.
        // This creates an iterator that will return zeroes when the buffer is exhausted.
        for (i, data) in bytes.iter().chain(iter::repeat(&0)).enumerate() {
            match self.write_byte(*data) {
                Ok(()) => {}
                Err(Error::NotAcknowledge) => {
                    // If we receive a NACK it will be on the FIFO write subsequent to the last byte
                    // actually written to the bus. If we start writing zeroes out, we only want to
                    // indicate how many bytes from the buffer we wrote.
                    return Ok(core::cmp::min(i - 1, bytes.len()));
                }
                Err(error) => return Err(error),
            }
        }
        unreachable!();
    }

    fn get_address(&self, addcode: u8) -> u16 {
        const I2C_RESERVED_ADDR_MASK: u8 = 0x7C;
        const I2C_10BIT_HEADER_CODE: u8 = 0x78;

        if (addcode & I2C_RESERVED_ADDR_MASK) == I2C_10BIT_HEADER_CODE {
            // No need to check if we're using 7 bits to perform a shift because we wouldn't
            // have received a 10-bit address unless we were listening for it
            self.i2c.oar1().read().oa1().bits()
        } else {
            addcode as u16
        }
    }
}

trait TargetAckMode {
    fn get_target_event(&mut self) -> Result<Option<TargetEvent>, Error>;
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error>;
    fn write(&mut self, bytes: &[u8]) -> Result<usize, Error>;
}

impl<I2C: Instance, R> TargetAckMode for I2cTarget<I2C, AutoAck, R> {
    fn get_target_event(&mut self) -> Result<Option<TargetEvent>, Error> {
        let isr = self.read_isr_and_check_errors()?;

        if isr.addr().is_match() {
            if isr.txe().is_not_empty() {
                // Flush the contents of TXDR without writing it to the bus. Doing so ensures that
                // spurious data is not written to the bus and that the clock remains stretched when
                // the ADDR flag is cleared and until data is written to TXDR (in the case of a
                // target write operation)
                self.i2c.isr().write(|w| w.txe().set_bit());
            }
            self.i2c.icr().write(|w| w.addrcf().clear());

            let address = self.get_address(isr.addcode().bits());

            if isr.dir().is_read() {
                Ok(Some(TargetEvent::Read { address }))
            } else {
                Ok(Some(TargetEvent::Write { address }))
            }
        } else if isr.stopf().is_stop() {
            self.i2c.icr().write(|w| w.stopcf().clear());
            Ok(Some(TargetEvent::Stop))
        } else {
            Ok(None)
        }
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        let size = self.read_all(buffer)?;

        // NACK any subsequent bytes
        self.nack();

        Ok(size)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        self.write_buf_fill_zeroes(bytes)
    }
}

impl<I2C: Instance, R> TargetAckMode for I2cTarget<I2C, ManualAck, R> {
    fn get_target_event(&mut self) -> Result<Option<TargetEvent>, Error> {
        let isr = self.read_isr_and_check_errors()?;

        if isr.addr().is_match() {
            if isr.txe().is_not_empty() {
                // Flush the contents of TXDR without writing it to the bus. Doing so ensures that
                // spurious data is not written to the bus and that the clock remains stretched when
                // the ADDR flag is cleared and until data is written to TXDR (in the case of a
                // target write operation)
                self.i2c.isr().write(|w| w.txe().set_bit());
            }
            // Reset SBC, reload, and nbytes. They're used differently for a read or write, but
            // make sure they're in a known state before starting the next operation.
            self.i2c.cr1().modify(|_, w| w.sbc().disabled());
            self.i2c
                .cr2()
                .modify(|_, w| w.reload().completed().nbytes().set(0));

            let address = self.get_address(isr.addcode().bits());

            if isr.dir().is_read() {
                self.i2c.icr().write(|w| w.addrcf().clear());
                Ok(Some(TargetEvent::Read { address }))
            } else {
                // Manual ACK control uses slave byte control mode when reading data from the
                // controller, so set it up here.
                self.i2c.cr1().modify(|_, w| w.sbc().enabled());
                self.i2c.cr2().modify(|_, w| w.reload().not_completed());
                Ok(Some(TargetEvent::Write { address }))
            }
        } else if isr.stopf().is_stop() {
            self.i2c.icr().write(|w| w.stopcf().clear());
            Ok(Some(TargetEvent::Stop))
        } else {
            Ok(None)
        }
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        let mut bytes_read = 0;
        // In manual ACK'ing mode we use Slave Byte Control (SBC) which requires using the NBYTES
        // field to indicate how many bytes to ACK before a manual ACK is required. The NBYTES field
        // is 8 bits wide, so if the transaction is larger than 255 bytes, we need to manually
        // restart the transfer by setting NBYTES again.
        for (i, buf) in buffer.chunks_mut(u8::MAX as usize).enumerate() {
            if i == 0 {
                self.start_transfer(buf.len());
            } else {
                self.restart_transfer(buf.len())
            }
            let count = self.read_all(buf)?;
            bytes_read += count;
            if count < buf.len() {
                break;
            }
        }

        Ok(bytes_read)
    }

    fn write(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        let mut bytes_written = 0;
        for (i, buf) in bytes.chunks(u8::MAX as usize).enumerate() {
            // In manual ACK'ing mode we use Slave Byte Control (SBC) which requires using the
            // NBYTES field to indicate how many TXIS events are generated. We need to set this
            // up for every 255 bytes because the NBYTES field is only 8 bits wide.
            if i == 0 {
                self.start_transfer(buf.len());
            } else {
                self.restart_transfer(buf.len())
            }
            let count = if bytes_written + buf.len() == bytes.len() {
                // This is the last chunk so write out zeroes if we reach the end of the buffer
                self.write_buf_fill_zeroes(buf)?
            } else {
                // Only write the contents of the buffer and move on to the next chunk
                self.write_buf(buf)?
            };
            bytes_written += count;
            if count < buf.len() {
                break;
            }
        }
        Ok(bytes_written)
    }
}

impl<I2C: Instance, R> I2cTarget<I2C, ManualAck, R> {
    /// This will start a transfer operation in manual ACK'ing mode.
    ///
    /// When performing a read operation the caller must pass the number
    /// of bytes that will be read before a manual ACK will be performed. All
    /// bytes prior to that will be ACK'd automatically.
    ///
    /// When performing a write operation this determines how many TXIS events
    /// will be generated for event handling (typically via interrupts).
    fn start_transfer(&mut self, expected_bytes: usize) {
        self.i2c
            .cr2()
            .modify(|_, w| w.nbytes().set(expected_bytes as u8));
        self.i2c.icr().write(|w| w.addrcf().clear());
    }

    /// End a transfer in manual ACK'ing mode. This should only be called after
    /// the expected number of bytes have been read (as set up with
    /// I2cTarget::start_transfer), otherwise the bus might hang.
    fn end_transfer(&mut self) {
        self.i2c.cr2().modify(|_, w| w.reload().completed());
    }

    /// Restart a transfer in manual ACK'ing mode.
    ///
    /// When performing a read operation, this allows partial reads of a
    /// transaction with manual ACK'ing together with I2cTarget::start_transfer.
    /// To handle a single byte between each ACK, set expected_bytes to 1
    /// before each read.
    ///
    /// When performing a write operation this determines how many TXIS events
    /// will be generated for event handling (typically via interrupts)
    fn restart_transfer(&mut self, expected_bytes: usize) {
        self.i2c
            .cr2()
            .modify(|_, w| w.nbytes().set(expected_bytes as u8));
    }

    /// End the transfer by ACK'ing the last byte in a read transfer after
    /// using the I2cTarget::read function in manual ACK'ing mode. This
    /// should only be called when the controller is not expected to send any
    /// more bytes, otherwise the bus will hang.
    pub fn ack_transfer(&mut self) {
        self.end_transfer();
    }

    /// End the transfer by NACK'ing the last received byte
    pub fn nack_transfer(&mut self) {
        self.nack();
        self.end_transfer();
    }
}

// We don't need to to expose TargetAckMode publically. All the public methods that
// wrap the allowable implementations are exposed below.
#[allow(private_bounds)]
impl<I2C, A, R> I2cTarget<I2C, A, R>
where
    Self: TargetAckMode,
{
    /// Blocks until this device to be addressed or for a transaction to be
    /// stopped. When the device is addressed, a `TargetEvent::Read` or
    /// `TargetEvent::Write` event will be returned. If the transaction was
    /// stopped, `TargetEvent::Stop` will be returned.
    ///
    /// If an error occurs while waiting for an event, this will be returned.
    pub fn wait_for_event(&mut self) -> Result<TargetEvent, Error> {
        loop {
            if let Some(event) = self.get_target_event()? {
                return Ok(event);
            }
        }
    }

    /// Perform a blocking read.
    ///
    /// If no error occurs, this will read until the buffer is full, or until
    /// the transfer is stopped by the controller, whichever comes first.
    ///
    /// The function will return the number of bytes received wrapped in the
    /// Ok result, or an error if one occurred.
    ///
    /// ## Auto-ACK'ing mode
    /// When the buffer is full the controller will be NACK'd for any
    /// subsequent byte received.
    ///
    /// ## Manual ACK'ing mode:
    /// If the buffer is filled before the controller stops the transaction,
    /// the final byte will not be ACK'd, but the function will return. The
    /// I2cTarget::ack_transfer function must be called in this case in order
    /// to complete the transaction.
    ///
    /// Note: this function should not be called to read less than the expected
    /// number of bytes in the total transaction. This could potentially leave
    /// the bus in a bad state.
    pub fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        TargetAckMode::read(self, buffer)
    }

    /// Perform a blocking write to the bus. This will write the
    /// contents of the buffer, followed by zeroes if the controller keeps
    /// clocking the bus. If the controller NACKs at any point the function
    /// will return with an Ok result.
    ///
    /// The function will return the number of bytes written wrapped in the
    /// Ok result, or an error if one occurred.
    pub fn write(&mut self, bytes: &[u8]) -> Result<usize, Error> {
        TargetAckMode::write(self, bytes)
    }

    /// Waits for a controller write event and reads the data written by the
    /// controller to the buffer provided. This will read until the buffer is
    /// full, or until the transfer is stopped by the controller, whichever
    /// comes first.
    ///
    /// If the a bus error occurs or a stop condition is received, an error
    /// will be returned, otherwise the number of bytes received wrapped in an
    /// Ok result will be returned
    ///
    /// ## Auto ACK'ing mode
    /// This will read until the buffer is full, at which point the controller
    /// will be NACK'd for any subsequent bytes received.
    ///
    /// ## Manual ACK'ing mode
    /// If the buffer is filled before the controller stops the transaction,
    /// the final byte will not be ACK'd. The I2cTarget::ack_transfer function
    /// must be called in this case in order to complete the transaction.
    ///
    /// ## Note:
    /// Any one of the configured primary or secondary addresses will be
    /// matched against. This method is not be suitable for use when
    /// different operations must be performed depending upon the received
    /// address.
    pub fn wait_for_target_read(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Error> {
        match self.wait_for_event()? {
            TargetEvent::Read { address: _ } => {
                Err(Error::ControllerExpectedWrite)
            }
            TargetEvent::Write { address: _ } => self.read(buffer),
            TargetEvent::Stop => Err(Error::TransferStopped),
        }
    }

    /// Waits for a controller read event and writes the contents of the
    /// buffer, followed by zeroes if the controller keeps clocking the bus. If
    /// the controller NACKs at any point the write will be terminated and the
    /// function will return with an Ok result indicating the number of bytes
    /// written before the NACK occurred.
    ///
    /// If a bus error occurs or a stop condition is received, the function
    /// will return an error, otherwise it will return the number of bytes
    /// written wrapped in an Ok result.
    pub fn wait_for_target_write(
        &mut self,
        bytes: &[u8],
    ) -> Result<usize, Error> {
        match self.wait_for_event()? {
            TargetEvent::Read { address: _ } => self.write(bytes),
            TargetEvent::Write { address: _ } => {
                Err(Error::ControllerExpectedRead)
            }
            TargetEvent::Stop => Err(Error::TransferStopped),
        }
    }

    /// Waits for a transaction to be stopped
    ///
    /// If a controller write or read event is received, an error is returned,
    /// otherwise an Ok result is returned when the stop condition is received.
    pub fn wait_for_stop(&mut self) -> Result<(), Error> {
        match self.wait_for_event()? {
            TargetEvent::Read { address: _ } => {
                Err(Error::ControllerExpectedWrite)
            }
            TargetEvent::Write { address: _ } => {
                Err(Error::ControllerExpectedRead)
            }
            TargetEvent::Stop => Ok(()),
        }
    }
}

impl<I2C, R> I2cTarget<I2C, AutoAck, R> {
    /// Convert this target to manual ACK control mode
    pub fn with_manual_ack_control(self) -> I2cTarget<I2C, ManualAck, R> {
        I2cTarget {
            inner: self.inner,
            _role: PhantomData,
            _ack: PhantomData,
        }
    }
}

impl<I2C, R> I2cTarget<I2C, ManualAck, R> {
    /// Convert this target to automatic ACK control mode
    pub fn with_automatic_ack_control(self) -> I2cTarget<I2C, AutoAck, R> {
        I2cTarget {
            inner: self.inner,
            _role: PhantomData,
            _ack: PhantomData,
        }
    }
}

pub trait Targetable {
    /// Enable the specified TargetListenEvent. This is required to start
    /// listening for transactions addressed to the device. It will not Ack
    /// any transactions addressed to it until one of these events is enabled.
    /// ie. to listen for transactions addressed to the primary address
    /// provided during Target configuration do:
    /// ```
    /// i2c.enable_target_event(TargetListenEvent::PrimaryAddress)
    /// ```
    fn enable_listen_event(&mut self, event: TargetListenEvent);

    /// Disable the specified TargetListenEvent. This can be used to stop
    /// listening for transactions. The peripheral will not Ack any messages
    /// addressed to it if all events are disabled.
    fn disable_listen_event(&mut self, event: TargetListenEvent);

    /// Configure target after initialization: allows the target addresses to be
    /// dynamically configured. This will disable any target events previously
    /// enabled.
    fn configure_target(&mut self, config: impl Into<TargetConfig>);
}

impl<I2C: Instance> Targetable for I2c<I2C, SwitchRole> {
    fn enable_listen_event(&mut self, event: TargetListenEvent) {
        self.inner.enable_target_event(event)
    }

    fn disable_listen_event(&mut self, event: TargetListenEvent) {
        self.inner.disable_target_event(event)
    }

    fn configure_target(&mut self, config: impl Into<TargetConfig>) {
        configure_target_addresses(&self.i2c, config.into())
    }
}

impl<I2C: Instance, R> Targetable for I2cTarget<I2C, R> {
    fn enable_listen_event(&mut self, event: TargetListenEvent) {
        self.inner.enable_target_event(event)
    }

    fn disable_listen_event(&mut self, event: TargetListenEvent) {
        self.inner.disable_target_event(event)
    }

    fn configure_target(&mut self, config: impl Into<TargetConfig>) {
        configure_target_addresses(&self.i2c, config.into())
    }
}

impl<I2C> I2c<I2C, SwitchRole> {
    /// Convert a controller implementation to a target. This is only possible
    /// if the I2c was created with a target configuration.
    pub fn to_target(self) -> I2cTarget<I2C, SwitchRole> {
        I2cTarget {
            inner: self.inner,
            _role: PhantomData,
            _ack: PhantomData,
        }
    }
}

impl<I2C> I2cTarget<I2C, SwitchRole> {
    /// Convert a target implementation to a controller. This is only possible
    /// for a I2cTarget created from an I2c instance.
    pub fn to_controller(self) -> I2c<I2C, SwitchRole> {
        I2c {
            inner: self.inner,
            _role: PhantomData,
        }
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
