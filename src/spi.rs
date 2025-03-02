//! Serial Peripheral Interface (SPI)
//!
//! This module provides functionality for SPI as both a Master. It
//! supports full-duplex, half duplex, simplex transmitter and simplex receiver
//! modes.
//!
//! It supports both blocking and non-blocking usage. For blocking usage
//! as a Master `Spi` implements the [embedded-hal][embedded-hal] traits ([`SpiBus`][spi_bus]
//! and [`SpiDevice`][spi_device]). `Spi` also implements the [`FullDuplex`][full_duplex] interface for
//! non-blocking operation
//!
//! # Usage
//!
//! In the simplest case, SPI can be initialised from the device peripheral
//! and the GPIO pins.
//!
//! ```
//! use stm32h5xx_hal::spi;
//!
//! let dp = ...;                   // Device peripherals
//! let (sck, miso, mosi) = ...;    // GPIO pins
//!
//! let spi = dp.SPI1.spi((sck, miso, mosi), spi::MODE_0, 1.MHz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! The GPIO pins should be supplied as a tuple in the following order:
//!
//! - Serial Clock (SCK)
//! - Master In Slave Out (MISO)
//! - Master Out Slave In (MOSI)
//!
//! If one of the pins is not required, explicitly pass one of the
//! filler types instead:
//!
//! ```
//! let spi = dp.SPI1.spi((sck, spi::NoMiso, mosi), spi::MODE_0, 1.MHz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! To use hardware control of CS pins, additionally provide a CS pin during
//! initialization, and specify hardware CS mode in the config struct:
//!
//! ```
//! use stm32h5xx_hal::spi;
//!
//! let dp = ...;                       // Device peripherals
//! let (sck, miso, mosi, cs) = ...;    // GPIO pins
//!
//! let spi = dp.SPI1.spi(
//!     (sck, miso, mosi, cs),
//!     spi::Config::new(spi::MODE_0)
//!         .hardware_cs(spi::HardwareCSMode::FrameTransaction),
//!     1.MHz(),
//!     ccdr.peripheral.SPI1,
//!     &ccdr.clocks);
//!```
//!
//!
//! Use the (`SpiBus`)[spi_bus] or (`SpiDevice`)[spi_device] (with hardware control of CS) APIs provided by
//! [embedded-hal](embedded-hal):
//!
//! ```
//! use stm32h5xx_hal::spi;
//! use embedded-hal::spi::SpiBus;
//!
//! let dp = ...;                   // Device peripherals
//! let (sck, miso, mosi) = ...;    // GPIO pins
//!
//! let spi = dp.SPI1.spi((sck, miso, mosi), spi::MODE_0, 1.MHz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//!
//! // Transmit only
//! spi.write(&[0x11u8, 0x22, 0x33])?;
//!
//! // Receive only
//! let read = &mut [0u8; 3];
//! spi.read(read)?;
//!
//! // Full duplex simultaneous transmit & receive
//! spi.transfer(read, &[0x11u8, 0x22, 0x33])?;
//! ```
//!
//! ## Clocks
//!
//! The bitrate calculation is based upon the clock currently assigned
//! in the RCC CCIP register. The default assignments are:
//!
//! - SPI1, SPI2, SPI3: __PLL1 Q CK__
//!
//!
//! ## Word Sizes
//!
//! The word size used by the SPI controller must be indicated to the compiler.
//! This can be done either using an explicit type annotation, or with a type
//! hint. The supported word sizes are 8, 16 or 32 bits (u8, u16, u32).
//!
//! For example, an explict type annotation:
//! ```
//! let _: spi:Spi<_, u8> = dp.SPI1.spi((sck, spi::NoMiso, mosi), spi::MODE_0, 1.MHz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//!
//! # Examples
//!
//! - [SPI Master](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/spi.rs)
//! - [SPI Master with Frame Transactions](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/spi_send_frames.rs)
//!
//! [embedded_hal]: https://docs.rs/embedded-hal/1.0.0-rc.1/embedded_hal/spi/index.html
//! [spi_bus]: https://docs.rs/embedded-hal/1.0.0-rc.1/embedded_hal/spi/trait.SpiBus.html
//! [spi_device]: https://docs.rs/embedded-hal/1.0.0-rc.1/embedded_hal/spi/trait.SpiDevice.html

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::ptr;

pub use embedded_hal::spi::{
    Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};

use crate::rcc::{CoreClocks, ResetEnable};
use crate::stm32::spi1;

use crate::time::Hertz;
use spi1::{cfg1::MBR, cfg2::LSBFRST, cfg2::SSIOP};

mod config;
mod hal;
mod spi_def;

pub use config::{
    CommunicationMode, Config, Endianness, HardwareCS, HardwareCSMode,
};

/// SPI error
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Underrun occurred
    Underrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    /// Calling this method is not valid in this state
    TransferAlreadyComplete,
    /// Can't start a transaction because one is already started
    TransactionAlreadyStarted,
    /// A buffer is too big to be processed
    BufferTooBig { max_size: usize },
    /// Caller makes invalid call (e.g. write in SimplexReceiver mode, or read in
    /// SimplexTransmitter)
    InvalidOperation,
}

pub trait Pins<SPI> {
    /// States whether or not the Hardware Chip Select is present in this set of pins
    const HCS_PRESENT: bool;
}

pub trait PinSck<SPI> {}

pub trait PinMiso<SPI> {}

pub trait PinMosi<SPI> {}

pub trait PinHCS<SPI> {}

impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
{
    const HCS_PRESENT: bool = false;
}

impl<SPI, SCK, MISO, MOSI, HCS> Pins<SPI> for (SCK, MISO, MOSI, HCS)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
    HCS: PinHCS<SPI>,
{
    const HCS_PRESENT: bool = true;
}

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;

/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;

/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

#[derive(Debug)]
pub struct Inner<SPI, W: Word> {
    spi: SPI,
    _word: PhantomData<W>,
}

/// Spi in Master mode
#[derive(Debug)]
pub struct Spi<SPI, W: Word = u8> {
    inner: Inner<SPI, W>,
    _word: PhantomData<W>,
}

impl<SPI, W: Word> Deref for Spi<SPI, W> {
    type Target = Inner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI, W: Word> DerefMut for Spi<SPI, W> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

// Implemented by all SPI instances
pub trait Instance:
    crate::Sealed + Deref<Target = spi1::RegisterBlock> + Sized
{
    type Rec: ResetEnable;

    #[doc(hidden)]
    fn ptr() -> *const spi1::RegisterBlock;

    #[doc(hidden)]
    fn clock(clocks: &CoreClocks) -> Hertz;

    #[doc(hidden)]
    fn rec() -> Self::Rec;
}

pub trait Word: Copy + Default + 'static + crate::Sealed {
    const BITS: u8;
}

macro_rules! word {
    ($type:ty) => {
        impl Word for $type {
            const BITS: u8 = <$type>::BITS as u8;
        }
        impl crate::Sealed for $type {}
    };
}

word!(u32);
word!(u16);
word!(u8);

/// Marker trait to indicate what word sizes each SPI peripheral supports
pub trait SupportedWordSize<W>: crate::Sealed {}

pub trait SpiExt<SPI: Instance + SupportedWordSize<W>, W: Word = u8> {
    fn spi<PINS, CONFIG>(
        self,
        _pins: PINS,
        config: CONFIG,
        freq: Hertz,
        rec: SPI::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, W>
    where
        PINS: Pins<SPI>,
        CONFIG: Into<Config>;

    fn spi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        freq: Hertz,
        rec: SPI::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, W>
    where
        CONFIG: Into<Config>;
}

impl<SPI: Instance + SupportedWordSize<W>, W: Word> SpiExt<SPI, W> for SPI {
    fn spi<PINS, CONFIG>(
        self,
        _pins: PINS,
        config: CONFIG,
        freq: Hertz,
        rec: SPI::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, W>
    where
        PINS: Pins<SPI>,
        CONFIG: Into<Config>,
    {
        let config = config.into();
        assert_eq!(
            config.hardware_cs.enabled(),
            PINS::HCS_PRESENT,
            "If the hardware cs is enabled in the config, an HCS pin must be present in the given pins"
        );
        Spi::<SPI, W>::new(self, config, freq, rec, clocks)
    }

    fn spi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        freq: Hertz,
        rec: SPI::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, W>
    where
        CONFIG: Into<Config>,
    {
        Spi::<SPI, W>::new(self, config, freq, rec, clocks)
    }
}

fn calc_mbr(spi_ker_ck: u32, spi_freq: u32) -> MBR {
    match spi_ker_ck.div_ceil(spi_freq) {
        1..=2 => MBR::Div2,
        3..=4 => MBR::Div4,
        5..=8 => MBR::Div8,
        9..=16 => MBR::Div16,
        17..=32 => MBR::Div32,
        33..=64 => MBR::Div64,
        65..=128 => MBR::Div128,
        _ => MBR::Div256,
    }
}

impl<SPI: Instance, W: Word> Spi<SPI, W> {
    fn new(
        spi: SPI,
        config: impl Into<Config>,
        freq: Hertz,
        rec: SPI::Rec,
        clocks: &CoreClocks,
    ) -> Self {
        let config: Config = config.into();
        rec.enable();

        let spi = Spi {
            inner: Inner::new(spi),
            _word: PhantomData,
        };
        spi.init(config, freq, SPI::clock(clocks))
    }

    fn init(self, config: Config, freq: Hertz, clock: Hertz) -> Self {
        let spi_freq = freq.raw();
        let spi_ker_ck = clock.raw();
        let mbr = calc_mbr(spi_ker_ck, spi_freq);
        self.spi.cfg1().modify(|_, w| {
            w.mbr()
                .variant(mbr) // master baud rate
                .dsize()
                .set(W::BITS - 1)
        });

        // Select master mode
        self.spi.cr1().write(|w| w.ssi().slave_not_selected());

        // Calculate the CS->transaction cycle delay bits.
        let (assertion_delay, inter_word_delay) = {
            let mut assertion_delay: u32 =
                (config.hardware_cs.assertion_delay * spi_freq as f32) as u32;
            let mut inter_word_delay: u32 =
                (config.inter_word_delay * spi_freq as f32) as u32;

            // If a delay is specified as non-zero, add 1 to the delay cycles
            // before truncation to an integer to ensure that we have at least as
            // many cycles as required.
            if config.hardware_cs.assertion_delay > 0.0_f32 {
                assertion_delay += 1;
            }
            if config.inter_word_delay > 0.0_f32 {
                inter_word_delay += 1;
            }

            // If CS suspends while data is inactive, we also require an
            // "inter-data" delay.
            if matches!(
                config.hardware_cs.mode,
                HardwareCSMode::WordTransaction
            ) {
                inter_word_delay = inter_word_delay.max(1);
            }

            (
                assertion_delay.min(0xF) as u8,
                inter_word_delay.min(0xF) as u8,
            )
        };

        let cs_polarity = match config.hardware_cs.polarity {
            Polarity::IdleHigh => SSIOP::ActiveLow,
            Polarity::IdleLow => SSIOP::ActiveHigh,
        };

        let endianness = match config.endianness {
            Endianness::LsbFirst => LSBFRST::Lsbfirst,
            Endianness::MsbFirst => LSBFRST::Msbfirst,
        };

        self.spi.cfg2().write(|w| {
            w.cpha()
                .bit(config.mode.phase == Phase::CaptureOnSecondTransition)
                .cpol()
                .bit(config.mode.polarity == Polarity::IdleHigh)
                .master()
                .master()
                .lsbfrst()
                .variant(endianness)
                .ssom()
                .bit(config.hardware_cs.interleaved_cs())
                .ssm()
                .bit(!config.hardware_cs.enabled())
                .ssoe()
                .bit(config.hardware_cs.enabled())
                .mssi()
                .set(assertion_delay)
                .midi()
                .set(inter_word_delay)
                .ioswp()
                .bit(config.swap_miso_mosi)
                .comm()
                .variant(config.communication_mode.into())
                .ssiop()
                .variant(cs_polarity)
        });

        self
    }
}

macro_rules! check_status_error {
    ($sr:expr) => {
        if $sr.ovr().is_overrun() {
            Err(Error::Overrun)
        } else if $sr.udr().is_underrun() {
            Err(Error::Underrun)
        } else if $sr.modf().is_fault() {
            Err(Error::ModeFault)
        } else if $sr.crce().is_error() {
            Err(Error::Crc)
        } else {
            Ok(())
        }
    };
}

impl<SPI: Instance, W: Word> Inner<SPI, W> {
    fn new(spi: SPI) -> Self {
        Self {
            spi,
            _word: PhantomData,
        }
    }

    /// Enable SPI
    fn enable(&mut self) {
        self.spi.cr1().modify(|_, w| w.spe().enabled());
    }

    /// Enable SPI
    fn disable(&mut self) {
        self.spi.cr1().modify(|_, w| w.spe().disabled());
    }

    /// Read the SPI communication mode
    fn communication_mode(&self) -> CommunicationMode {
        self.spi.cfg2().read().comm().variant().into()
    }

    /// Set SPI to transmit mode in half duplex operation
    /// Only valid in half duplex operation. This is provided for non-blocking calls to be able to
    /// change direction of communication. Blocking calls already handle this
    pub fn set_dir_transmitter(&self) {
        assert!(matches!(
            self.communication_mode(),
            CommunicationMode::HalfDuplex
        ));

        self.spi.cr1().modify(|_, w| w.hddir().transmitter());
    }

    /// Set SPI to receive mode in half duplex operation
    /// Only valid in half duplex operation. This is provided for non-blocking calls to be able to
    /// change direction of communication. Blocking calls already handle this
    pub fn set_dir_receiver(&self) {
        assert!(matches!(
            self.communication_mode(),
            CommunicationMode::HalfDuplex
        ));

        self.spi.cr1().modify(|_, w| w.hddir().receiver());
    }

    /// Set the word size for a transaction. Can only be changed when the peripheral is disabled
    #[inline(always)]
    pub fn set_word_size(&self, word_size: usize) {
        // Do not allow a word size greater than W, and ensure that it is greater than the minimum
        // word size for the peripheral
        assert!(word_size <= (W::BITS as usize) && word_size >= 4);
        self.spi
            .cfg1()
            .modify(|_, w| w.dsize().set((word_size as u8) - 1));
    }

    /// Clears the MODF flag, which indicates that a
    /// mode fault has occurred.
    #[inline(always)]
    fn clear_modf(&mut self) {
        self.spi.ifcr().write(|w| w.modfc().clear());
        interrupt_clear_clock_sync_delay!(self.spi.sr());
    }

    /// Read a single word from the receive data register
    #[inline(always)]
    fn read_data_reg(&mut self) -> W {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
        // reading a half-word)
        unsafe { ptr::read_volatile(self.spi.rxdr() as *const _ as *const W) }
    }

    #[inline(always)]
    fn write_data_reg(&mut self, data: W) {
        // NOTE(write_volatile/read_volatile) write/read only 1 word
        unsafe {
            let txdr = self.spi.txdr() as *const _ as *const UnsafeCell<W>;
            ptr::write_volatile(UnsafeCell::raw_get(txdr), data);
        }
    }

    #[inline(always)]
    fn read_if_ready(&mut self, word: &mut W) -> Result<bool, Error> {
        let sr = self.spi.sr().read();

        check_status_error!(sr)?;

        if sr.rxp().is_not_empty() {
            *word = self.read_data_reg();
            Ok(true)
        } else {
            Ok(false)
        }
    }

    #[inline(always)]
    fn write_if_ready(&mut self, word: W) -> Result<bool, Error> {
        let sr = self.spi.sr().read();

        check_status_error!(sr)?;

        if sr.txp().is_not_full() {
            self.write_data_reg(word);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Fill the TX FIFO. Will write to the TX fifo until it is full. The
    /// amount of words written are returned wrapped in the Ok value.
    fn fill_tx_fifo(&mut self, words: &[W]) -> Result<usize, Error> {
        for (i, word) in words.iter().enumerate() {
            if !self.write_if_ready(*word)? {
                return Ok(i);
            }
        }
        Ok(words.len())
    }

    /// Flush the TX Fifo. Attempts to write `len` zeroes to the TX FIFO.
    /// Returns the number written, wrapped in an Ok result, unless an error
    /// occurs.
    fn flush_tx_fifo(&mut self, len: usize) -> Result<usize, Error> {
        for i in 0..len {
            if !self.write_if_ready(W::default())? {
                return Ok(i);
            }
        }
        Ok(len)
    }

    /// Reads as many words as are available in the RX FIFO.  The amount of
    /// words written are returned wrapped in the Ok value, unless an error
    /// occurs.
    fn drain_rx_fifo(&mut self, words: &mut [W]) -> Result<usize, Error> {
        for (i, word) in words.iter_mut().enumerate() {
            if !self.read_if_ready(word)? {
                return Ok(i);
            }
        }
        Ok(words.len())
    }

    /// Flush the RX Fifo. Attempts to read `len` bytes from the RX FIFO.
    /// Returns the number read, wrapped in an Ok result, unless an error
    /// occurs.
    fn flush_rx_fifo(&mut self, len: usize) -> Result<usize, Error> {
        for i in 0..len {
            let mut dummy = W::default();
            if !self.read_if_ready(&mut dummy)? {
                return Ok(i);
            }
        }
        Ok(len)
    }

    fn empty_rx_fifo(&mut self) -> Result<(), Error> {
        let mut dummy = W::default();
        while self.read_if_ready(&mut dummy)? {}
        Ok(())
    }

    fn write_words(&mut self, words: &[W]) -> Result<(), Error> {
        let mut i = 0;
        i += self.fill_tx_fifo(words)?;

        // Continue filling write FIFO and emptying read FIFO
        while i < words.len() {
            self.empty_rx_fifo()?;
            i += self.fill_tx_fifo(&words[i..])?;
        }

        // Final flush of read buffer
        self.empty_rx_fifo()
    }

    fn read_words(&mut self, words: &mut [W]) -> Result<(), Error> {
        let mut flushed = self.flush_tx_fifo(words.len())?;

        let mut i = 0;
        while i < words.len() {
            i += self.drain_rx_fifo(&mut words[i..])?;
            flushed += self.flush_tx_fifo(words.len() - flushed)?;
        }

        Ok(())
    }

    fn transfer_words(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Error> {
        let len = core::cmp::max(read.len(), write.len());
        let mut write_idx = 0;
        let mut read_idx = 0;
        while read_idx < read.len() || write_idx < write.len() {
            if write_idx < write.len() {
                write_idx += self.fill_tx_fifo(&write[write_idx..])?;
            }

            if write_idx >= write.len() && write_idx < len {
                write_idx += self.flush_tx_fifo(len - write_idx)?;
            }

            if read_idx < read.len() {
                read_idx += self.drain_rx_fifo(&mut read[read_idx..])?;
            }

            if read_idx >= read.len() && read_idx < len {
                read_idx += self.flush_rx_fifo(len - read_idx)?;
            }
        }

        Ok(())
    }

    fn transfer_words_inplace(&mut self, words: &mut [W]) -> Result<(), Error> {
        let mut write_idx = 0;
        let mut read_idx = 0;
        write_idx += self.fill_tx_fifo(&words[write_idx..])?;

        while write_idx < words.len() || read_idx < words.len() {
            if read_idx < words.len() {
                read_idx += self.drain_rx_fifo(&mut words[read_idx..])?;
            }
            if write_idx < words.len() {
                write_idx += self.fill_tx_fifo(&words[write_idx..])?;
            }
        }
        Ok(())
    }
}

impl<SPI: Instance, W: Word> Spi<SPI, W> {
    /// Sets up a frame transaction with the given amount of data words.
    ///
    /// If this is called when a transaction has already started,
    /// then an error is returned with [Error::TransactionAlreadyStarted].
    fn setup_transaction(&mut self, length: usize) {
        assert!(
            length <= u16::MAX as usize,
            "Buffer too big! Max transaction size is {}",
            u16::MAX
        );

        self.spi.cr2().write(|w| w.tsize().set(length as u16));

        // Re-enable
        self.clear_modf();
        self.enable();
        self.spi.cr1().modify(|_, w| w.cstart().started());
    }

    /// Checks if the current transaction is complete and disables the
    /// peripheral if it is, returning true. If it isn't, returns false.
    fn end_transaction_if_done(&mut self) -> bool {
        if self.spi.sr().read().txc().is_ongoing() {
            return false;
        }

        // Errata ES0561 Rev 3 2.13.2: Disabling the peripheral can cut short the last clock pulse
        // which can cause transmission failures. This spin loop causes a long enough delay for most
        // clock frequencies such that the last clock is output correctly.
        for _i in 0..30 {
            cortex_m::asm::nop()
        }

        self.spi
            .ifcr()
            .write(|w| w.txtfc().clear().eotc().clear().suspc().clear());

        self.disable();
        true
    }

    /// Ends the current transaction. Waits for all data to be transmitted.
    /// This must always be called when all data has been sent to
    /// properly terminate the transaction and reset the SPI peripheral.
    fn end_transaction(&mut self) {
        // Result is only () or WouldBlock. Discard result.
        while !self.end_transaction_if_done() {}
    }

    fn abort_transaction(&mut self) {
        self.disable();
    }

    /// Write-only transfer
    fn write(&mut self, words: &[W]) -> Result<(), Error> {
        let communication_mode = self.communication_mode();
        if communication_mode == CommunicationMode::SimplexReceiver {
            return Err(Error::InvalidOperation);
        }

        if words.is_empty() {
            return Ok(());
        }

        if communication_mode == CommunicationMode::HalfDuplex {
            self.set_dir_transmitter();
        }

        self.setup_transaction(words.len());

        match self.write_words(words) {
            Ok(()) => {
                self.end_transaction();
                Ok(())
            }
            Err(error) => {
                self.abort_transaction();
                Err(error)
            }
        }
    }

    /// Read-only transfer
    fn read(&mut self, words: &mut [W]) -> Result<(), Error> {
        let communication_mode = self.communication_mode();
        if communication_mode == CommunicationMode::SimplexTransmitter {
            return Err(Error::InvalidOperation);
        }

        if words.is_empty() {
            return Ok(());
        }

        if communication_mode == CommunicationMode::HalfDuplex {
            self.set_dir_receiver();
        }

        self.setup_transaction(words.len());

        match self.read_words(words) {
            Ok(()) => {
                self.end_transaction();
                Ok(())
            }
            Err(error) => {
                self.abort_transaction();
                Err(error)
            }
        }
    }

    /// Full duplex transfer
    fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        if self.communication_mode() != CommunicationMode::FullDuplex {
            return Err(Error::InvalidOperation);
        }

        if read.is_empty() && write.is_empty() {
            return Ok(());
        }

        let len = core::cmp::max(read.len(), write.len());
        self.setup_transaction(len);

        match self.transfer_words(read, write) {
            Ok(()) => {
                self.end_transaction();
                Ok(())
            }
            Err(error) => {
                self.abort_transaction();
                return Err(error);
            }
        }
    }

    /// Full duplex transfer with single buffer for transmit and receive
    fn transfer_inplace(&mut self, words: &mut [W]) -> Result<(), Error> {
        if self.communication_mode() != CommunicationMode::FullDuplex {
            return Err(Error::InvalidOperation);
        }

        if words.is_empty() {
            return Ok(());
        }

        self.setup_transaction(words.len());

        match self.transfer_words_inplace(words) {
            Ok(()) => {
                self.end_transaction();
                Ok(())
            }
            Err(error) => {
                self.abort_transaction();
                Err(error)
            }
        }
    }

    /// Deconstructs the SPI peripheral and returns the component parts.
    pub fn free(self) -> SPI {
        self.inner.spi
    }

    pub fn inner(&self) -> &SPI {
        &self.spi
    }

    pub fn inner_mut(&mut self) -> &mut SPI {
        &mut self.spi
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calc_mbr() {
        let result = calc_mbr(100_000_000, 50_000_000);
        assert_eq!(result, MBR::Div2);

        let result = calc_mbr(50_000_000, 1_000_000);
        assert_eq!(result, MBR::Div64);

        let result = calc_mbr(32_000_000, 1_000_000);
        assert_eq!(result, MBR::Div32);
    }
}
