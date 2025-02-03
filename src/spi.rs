//! Serial Peripheral Interface (SPI)
//!
//! This module provides functionality for SPI as a Master. It supports full-duplex, half duplex,
//! simplex transmitter and simplex receiver modes.
//!
//! It supports both blocking and non-blocking usage. For blocking usage
//! as a Master `Spi` implements the [embedded-hal][embedded-hal] traits ([`SpiBus`][spi_bus]
//! and [`SpiDevice`][spi_device]). The driver also provides a
//! [NonBlocking](nonblocking::NonBlocking) trait that defines the non-blocking API. This must be
//! explicitly used to access the non-blocking API.
//!
//! # Usage
//!
//! ## Initialization
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
//! ## Blocking API
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
//! ## Non-blocking API
//! To use the non-blocking API, the [`nonblocking::NonBlocking`] trait must be used. Then,
//! transactions need to be started via one of the start_nonblocking_XXX methods. This will return
//! a [`transaction::Transaction`] type that can be passed to
//! [`nonblocking::NonBlocking::transfer_nonblocking()`].
//!
//! ```
//! use stm32h5xx_hal::spi::{self, nonblocking::NonBlocking};
//!
//! let dp = ...;                   // Device peripherals
//! let (sck, miso, mosi) = ...;    // GPIO pins
//!
//! let mut spi = dp.SPI1.spi((sck, miso, mosi), spi::MODE_0, 1.MHz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//!
//! let words = [0x11u8, 0x22, 0x33];
//! let mut write = spi.start_nonblocking_write(&words)?;
//!
//! while let Some(t) = spi.transfer_nonblocking(write)? {
//!     write = t;
//! }
//! ```
//!
//! ## Clocks
//!
//! The bitrate calculation is based upon the clock currently assigned
//! in the RCC CCIP register. The default assignments are:
//!
//! - SPI1, SPI2, SPI3: PLL1Q
//! - SPI4, SPI6: PCLK2
//! - SPI5: PCLK3
//!
//! ## Word Sizes
//!
//! The word size used by the SPI controller must be indicated to the compiler.
//! This can be done either using an explicit type annotation, or with a type
//! hint. The supported word sizes are 8, 16 or 32 bits (u8, u16, u32). However,
//! 32-bit access are not supported by all peripherals.
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
use core::ops::Deref;
use core::ptr;

pub use embedded_hal::spi::{
    Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};

use crate::gpdma::Error as DmaError;
use crate::rcc::{CoreClocks, ResetEnable};
use crate::stm32::spi1;

use crate::time::Hertz;
use spi1::{cfg1::MBR, cfg2::LSBFRST, cfg2::SSIOP};

mod config;
pub mod dma;
mod hal;
pub mod nonblocking;
mod spi_def;
mod transaction;

pub use config::{
    CommunicationMode, Config, Endianness, HardwareCS, HardwareCSMode,
};

use transaction::Op;
pub use transaction::{Read, Transaction, Transfer, TransferInplace, Write};

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
    /// A DMA error occurred during processing
    DmaError(DmaError)
}

impl From<DmaError> for Error {
    fn from(error: DmaError) -> Self {
        Error::DmaError(error)
    }
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

    #[doc(hidden)]
    fn tx_dma_request() -> u8;

    #[doc(hidden)]
    fn rx_dma_request() -> u8;
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

    fn spi(&mut self) -> &mut SPI {
        &mut self.inner.spi
    }

    fn init(mut self, config: Config, freq: Hertz, clock: Hertz) -> Self {
        let spi_freq = freq.raw();
        let spi_ker_ck = clock.raw();
        let mbr = calc_mbr(spi_ker_ck, spi_freq);
        self.spi().cfg1().modify(|_, w| {
            w.mbr()
                .variant(mbr) // master baud rate
                .dsize()
                .set(W::BITS - 1)
        });

        // Select master mode
        self.spi().cr1().write(|w| w.ssi().slave_not_selected());

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

        self.spi().cfg2().write(|w| {
            w.afcntr()
                .controlled()
                .cpha()
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

    /// Determine if SPI is a transmitter
    fn is_transmitter(&self) -> bool {
        match self.communication_mode() {
            CommunicationMode::FullDuplex => true,
            CommunicationMode::HalfDuplex => self.is_half_duplex_transmitter(),
            CommunicationMode::SimplexTransmitter => true,
            CommunicationMode::SimplexReceiver => false,
        }
    }

    fn is_receiver(&self) -> bool {
        match self.communication_mode() {
            CommunicationMode::FullDuplex => true,
            CommunicationMode::HalfDuplex => !self.is_half_duplex_transmitter(),
            CommunicationMode::SimplexTransmitter => false,
            CommunicationMode::SimplexReceiver => true,
        }
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

    fn is_half_duplex_transmitter(&self) -> bool {
        self.spi.cr1().read().hddir().is_transmitter()
    }

    /// Set the word size for a transaction. Can only be changed when the peripheral is disabled
    #[inline(always)]
    fn set_word_size(&self, word_size: usize) {
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
        let _ = self.spi.sr().read();
        let _ = self.spi.sr().read();
    }

    /// Disable DMA for both Rx and Tx
    #[inline]
    pub fn enable_tx_dma(&self) {
        self.spi.cfg1().modify(|_, w| w.txdmaen().disabled());
    }

    /// Disable DMA for both Rx and Tx
    #[inline]
    pub fn enable_rx_dma(&self) {
        self.spi.cfg1().modify(|_, w| w.rxdmaen().disabled());
    }

    /// Disable DMA for both Rx and Tx
    #[inline]
    pub fn disable_dma(&self) {
        self.spi.cfg1().modify(|_, w| w.rxdmaen().disabled().txdmaen().disabled());
    }

    #[inline]
    pub fn start_transfer(&self) {
        self.spi.cr1().modify(|_, w| w.cstart().started());
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
    #[inline(always)]
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
    #[inline(always)]
    fn flush_tx_fifo(&mut self, len: usize) -> Result<usize, Error> {
        // If the device is not a transmitter, no bytes can be written/flushed, so just return
        // immediately.
        if !self.is_transmitter() {
            return Ok(len);
        }
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
    #[inline(always)]
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
    #[inline(always)]
    fn discard_rx_fifo(&mut self, len: usize) -> Result<usize, Error> {
        // If the device is not a receiver, no bytes will be received or available to read/discard,
        // so just return immediately.
        if !self.is_receiver() {
            return Ok(len);
        }
        for i in 0..len {
            let mut dummy = W::default();
            if !self.read_if_ready(&mut dummy)? {
                return Ok(i);
            }
        }
        Ok(len)
    }

    /// Write operation for a partial transaction. For use in multi-operation transactions via
    /// embedded-hal's SpiDevice trait.
    fn write_partial(&mut self, words: &[W]) -> Result<(), Error> {
        let mut write = Transaction::<Write<W>, W>::write(words);
        while !write.is_complete() {
            self.transfer_words_nonblocking(&mut write)?;
        }

        Ok(())
    }

    /// Read operation for a partial transaction. For use in multi-operation transactions via
    /// embedded-hal's SpiDevice trait.
    fn read_partial(&mut self, words: &mut [W]) -> Result<(), Error> {
        let mut read = Transaction::<Read<W>, W>::read(words);
        while !read.is_complete() {
            self.transfer_words_nonblocking(&mut read)?;
        }

        Ok(())
    }

    /// Duplex transfer for a partial transaction. For use in multi-operation transactions via
    /// embedded-hal's SpiDevice trait.
    fn transfer_partial(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Error> {
        let mut transfer = Transaction::<Transfer<W>, W>::transfer(write, read);
        while !transfer.is_complete() {
            self.transfer_words_nonblocking(&mut transfer)?;
        }

        Ok(())
    }

    /// Duplex transfer in place for a partial transaction. For use in multi-operation transactions
    /// via embedded-hal's SpiDevice trait.
    fn transfer_inplace_partial(
        &mut self,
        words: &mut [W],
    ) -> Result<(), Error> {
        let mut transfer =
            Transaction::<TransferInplace<W>, W>::transfer_inplace(words);
        while !transfer.is_complete() {
            self.transfer_words_nonblocking(&mut transfer)?;
        }
        Ok(())
    }

    fn write_words_nonblocking<OP: Op<W>>(
        &mut self,
        transaction: &mut Transaction<OP, W>,
    ) -> Result<(), Error> {
        // Don't attempt to write out more than the size of the transaction
        let count = self.fill_tx_fifo(transaction.write_buf())?;
        transaction.advance_write_idx(count);
        Ok(())
    }

    fn read_words_nonblocking<OP: Op<W>>(
        &mut self,
        transaction: &mut Transaction<OP, W>,
    ) -> Result<(), Error> {
        let count = self.drain_rx_fifo(transaction.read_buf())?;
        transaction.advance_read_idx(count);
        Ok(())
    }

    /// Transfers words in a non-blocking manner. This function drives all SPI read/write/transfer
    /// operations in this driver.
    ///
    /// This will write data to the TX FIFO if there is data to write, and read data from the RX
    /// FIFO if there is data to read. If there is data to read, but not to write (ie. read length
    /// is greater than write length) then it will write dummy bytes (zeroes) to the TX FIFO to
    /// clock in the remaining read data. If there is data to write, but not to read, the contents
    /// of the RX FIFO will be discarded.
    #[inline(always)]
    fn transfer_words_nonblocking<OP: Op<W>>(
        &mut self,
        transaction: &mut Transaction<OP, W>,
    ) -> Result<(), Error> {
        if !transaction.is_write_complete() {
            self.write_words_nonblocking(transaction)?;
        }

        // Keep writing to SPI if data to read is longer than data to write
        let flush_remainder = transaction.tx_flush_remainder();
        if transaction.is_write_complete() && flush_remainder > 0 {
            let count = self.flush_tx_fifo(flush_remainder)?;
            transaction.advance_write_idx(count)
        }

        if !transaction.is_read_complete() {
            self.read_words_nonblocking(transaction)?;
        }

        // Keep emptying Rx FIFO if data to write is longer than data to read
        let discard_remainder = transaction.rx_remainder_to_discard();
        if transaction.is_read_complete() && discard_remainder > 0 {
            let count = self.discard_rx_fifo(discard_remainder)?;
            transaction.advance_read_idx(count);
        }

        Ok(())
    }
}

impl<SPI: Instance, W: Word> Spi<SPI, W> {
    /// Set the word size for a transaction. Can only be changed when the peripheral is disabled
    /// Must be less than or equal to the maximum size denoted by the const generic size parameter
    /// (W)
    pub fn set_word_size(&mut self, word_size: usize) {
        self.inner.set_word_size(word_size);
    }

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

        self.spi().cr2().write(|w| w.tsize().set(length as u16));

        // Re-enable
        self.inner.clear_modf();
        self.inner.enable();
        self.spi().cr1().modify(|_, w| w.cstart().started());
    }

    /// Checks if the current transaction is complete and disables the
    /// peripheral if it is, returning true. If it isn't, returns false.
    fn end_transaction_if_done(&mut self) -> bool {
        let sr = self.spi().sr().read();
        let is_complete = if self.inner.is_transmitter() {
            sr.eot().is_completed() && sr.txc().is_completed()
        } else {
            sr.eot().is_completed()
        };

        if !is_complete {
            return false;
        }

        // Errata ES0561 Rev 3 2.13.2: Disabling the peripheral can cut short the last clock pulse
        // which can cause transmission failures. This spin loop causes a long enough delay for most
        // clock frequencies such that the last clock is output correctly.
        for _i in 0..30 {
            cortex_m::asm::nop()
        }

        self.spi()
            .ifcr()
            .write(|w| w.txtfc().clear().eotc().clear().suspc().clear());

        self.inner.disable();
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
        self.inner.disable();
    }

    /// Deconstructs the SPI peripheral and returns the component parts.
    pub fn free(self) -> SPI {
        self.inner.spi
    }

    pub fn inner(&self) -> &SPI {
        &self.inner.spi
    }

    pub fn inner_mut(&mut self) -> &mut SPI {
        &mut self.inner.spi
    }
}

/// Non-blocking SPI operations
impl<SPI: Instance, W: Word> Spi<SPI, W> {
    fn transfer_nonblocking_internal<OP: Op<W>>(
        &mut self,
        mut transaction: Transaction<OP, W>,
    ) -> Result<Option<Transaction<OP, W>>, Error> {
        self.inner
            .transfer_words_nonblocking(&mut transaction)
            .inspect_err(|_| self.abort_transaction())?;

        if transaction.is_complete() && self.end_transaction_if_done() {
            Ok(None)
        } else {
            Ok(Some(transaction))
        }
    }

    fn start_write<'a>(
        &mut self,
        words: &'a [W],
    ) -> Result<Transaction<Write<'a, W>, W>, Error> {
        assert!(
            !words.is_empty(),
            "Write buffer should not be non-zero length"
        );
        let communication_mode = self.inner.communication_mode();
        if communication_mode == CommunicationMode::SimplexReceiver {
            return Err(Error::InvalidOperation);
        }

        if communication_mode == CommunicationMode::HalfDuplex {
            self.inner.set_dir_transmitter();
        }

        self.setup_transaction(words.len());

        Ok(Transaction::<Write<'a, W>, W>::write(words))
    }

    fn start_read<'a>(
        &mut self,
        buf: &'a mut [W],
    ) -> Result<Transaction<Read<'a, W>, W>, Error> {
        assert!(!buf.is_empty(), "Read buffer should not be non-zero length");
        let communication_mode = self.inner.communication_mode();
        if communication_mode == CommunicationMode::SimplexTransmitter {
            return Err(Error::InvalidOperation);
        }

        if communication_mode == CommunicationMode::HalfDuplex {
            self.inner.set_dir_receiver();
        }

        self.setup_transaction(buf.len());

        Ok(Transaction::<Read<'a, W>, W>::read(buf))
    }

    fn start_transfer<'a>(
        &mut self,
        read: &'a mut [W],
        write: &'a [W],
    ) -> Result<Transaction<Transfer<'a, W>, W>, Error> {
        assert!(
            !read.is_empty() && !write.is_empty(),
            "Transfer buffers should not be of zero length"
        );
        if self.inner.communication_mode() != CommunicationMode::FullDuplex {
            return Err(Error::InvalidOperation);
        }

        self.setup_transaction(core::cmp::max(read.len(), write.len()));

        Ok(Transaction::<Transfer<'a, W>, W>::transfer(write, read))
    }

    fn start_transfer_inplace<'a>(
        &mut self,
        words: &'a mut [W],
    ) -> Result<Transaction<TransferInplace<'a, W>, W>, Error> {
        assert!(
            !words.is_empty(),
            "Transfer buffer should not be of zero length"
        );
        if self.inner.communication_mode() != CommunicationMode::FullDuplex {
            return Err(Error::InvalidOperation);
        }

        self.setup_transaction(words.len());

        Ok(Transaction::<TransferInplace<'a, W>, W>::transfer_inplace(
            words,
        ))
    }
}

// Blocking SPI operations
impl<SPI: Instance, W: Word> Spi<SPI, W> {
    fn transfer_internal<OP: Op<W>>(
        &mut self,
        mut transaction: Transaction<OP, W>,
    ) -> Result<(), Error> {
        while let Some(t) = self.transfer_nonblocking_internal(transaction)? {
            transaction = t;
        }
        Ok(())
    }

    /// Write-only transfer
    fn write(&mut self, words: &[W]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }

        let transaction = self.start_write(words)?;
        self.transfer_internal(transaction)
    }

    /// Read-only transfer
    fn read(&mut self, words: &mut [W]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }
        let transaction = self.start_read(words)?;
        self.transfer_internal(transaction)
    }

    /// Full duplex transfer
    fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        if read.is_empty() && write.is_empty() {
            return Ok(());
        }

        let transaction = self.start_transfer(read, write)?;
        self.transfer_internal(transaction)
    }

    /// Full duplex transfer with single buffer for transmit and receive
    fn transfer_inplace(&mut self, words: &mut [W]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }

        let transaction = self.start_transfer_inplace(words)?;
        self.transfer_internal(transaction)
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
