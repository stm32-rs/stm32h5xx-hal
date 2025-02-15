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
//! ## Blocking usage
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
//! ## Non-blocking usage:
//!
//! Transactions need to be setup and ended properly in non-blocking mode. Use
//! `Spi::setup_transaction` to tell the driver how many words must be
//! transferred and to enable the transceiver. Use `Spi::end_transaction` to clean
//! up once all bytes have been transferred. If this is not called, subsequent
//! transactions may not work. If an error occurs during the transaction, call
//! `Spi::abort_transaction` to clean up and clear the errors and allow the
//! peripheral to be used again.
//!
//! To transfer data, the (`FullDuplex`)[full_duplex] APIs from
//! [embedded-hal](embedded-hal) can be used to read/write data:
//!
//! ```
//! use stm32h5xx_hal::spi;
//! use embedded_hal_nb::spi::FullDuplex;
//! use nb::block;
//!
//! let dp = ...;                   // Device peripherals
//! let (sck, miso, mosi) = ...;    // GPIO pins
//!
//! let mut spi = dp.SPI1.spi((sck, miso, mosi), spi::MODE_0, 1.MHz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//!
//! spi.setup_transaction(NonZeroUsize::new(3).unwrap())?;
//!
//! for word in &[0x11u8, 0x22, 0x33] {
//!    block!(FullDuplex::write(&mut spi, *word)).unwrap();
//! }
//! spi.end_transaction();
//! ```
//!
//! Alternatively, you can use Spi::read_nb, Spi::write_nb, and
//! Spi::transfer_nb (with spi::NonBlockingTransfer) functions:
//!
//! Write-only transaction:
//! ```
//! spi.setup_transaction(NonZeroUsize::new(3).unwrap())?;
//!
//! let write = &[0x11, 0x22, 0x33];
//! let mut count = 0;
//! while count < write.len() {
//!     count += spi.write_nb(write[count..])?;
//! }
//!
//! block!(spi.end_transaction_nb())?;
//! ```
//!
//! Read only transaction:
//! ```
//! spi.setup_transaction(NonZeroUsize::new(3).unwrap())?;
//!
//! let read = &mut [0u8; 3];
//! let mut count = 0;
//! while count < read.len() {
//!     count += spi.read_nb(&mut read[count..])?;
//! }
//!
//! block!(spi.end_transaction_nb())?;
//! ```
//!
//! Full-duplex transfer with different buffer sizes:
//! ```
//! let write = &[0x11, 0x22, 0x33];
//! let read = &mut [0u8; 8];
//! let transfer = NonBlockingTransfer::new(read, write);
//!
//! // No need to call setup_transaction as self.transfer.nb handles that
//! block!(self.transfer_nb(&mut transfer))?;
//!
//! block!(spi.end_transaction_nb())?;
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
//! - [Blocking SPI Master](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/spi.rs)
//! - [SPI Master with Frame Transactions](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/spi_send_frames.rs)
//!
//! [embedded_hal]: https://docs.rs/embedded-hal/1.0.0-rc.1/embedded_hal/spi/index.html
//! [spi_bus]: https://docs.rs/embedded-hal/1.0.0-rc.1/embedded_hal/spi/trait.SpiBus.html
//! [spi_device]: https://docs.rs/embedded-hal/1.0.0-rc.1/embedded_hal/spi/trait.SpiDevice.html
//! [full_duplex]: https://docs.rs/embedded-hal-nb/1.0.0-rc.1/embedded_hal_nb/spi/trait.FullDuplex.html

use core::cell::UnsafeCell;
use core::convert::From;
use core::marker::PhantomData;
use core::num::NonZeroUsize;
use core::ops::{Deref, DerefMut};
use core::ptr;

use crate::gpio::{self, Alternate};

pub use embedded_hal::spi::{
    Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32;
use crate::stm32::rcc::ccipr3;
use crate::stm32::spi1;
#[cfg(any(feature = "h523_h533", feature = "h56x_h573"))]
use crate::stm32::SPI4;
use crate::stm32::{SPI1, SPI2, SPI3};
#[cfg(feature = "h56x_h573")]
use crate::stm32::{SPI5, SPI6};

use crate::time::Hertz;
use spi1::{cfg1::MBR, cfg2::COMM, cfg2::LSBFRST, cfg2::SSIOP};

mod hal;

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

/// Specifies the communication mode of the SPI interface.
#[derive(Copy, Clone, PartialEq)]
pub enum CommunicationMode {
    /// Both RX and TX are used on separate wires. This is the default communications mode
    FullDuplex,

    /// RX and TX are used on a single wire.
    HalfDuplex,

    /// Only the SPI TX functionality is used on a single wire.
    SimplexTransmitter,

    /// Only the SPI RX functionality is used on a single wire.
    SimplexReceiver,
}

impl From<COMM> for CommunicationMode {
    fn from(value: COMM) -> Self {
        match value {
            COMM::Transmitter => CommunicationMode::SimplexTransmitter,
            COMM::Receiver => CommunicationMode::SimplexReceiver,
            COMM::FullDuplex => CommunicationMode::FullDuplex,
            COMM::HalfDuplex => CommunicationMode::HalfDuplex,
        }
    }
}

impl From<CommunicationMode> for COMM {
    fn from(value: CommunicationMode) -> Self {
        match value {
            CommunicationMode::SimplexTransmitter => COMM::Transmitter,
            CommunicationMode::SimplexReceiver => COMM::Receiver,
            CommunicationMode::FullDuplex => COMM::FullDuplex,
            CommunicationMode::HalfDuplex => COMM::HalfDuplex,
        }
    }
}

/// The endianness with which to send the data
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endianness {
    /// Least significant bit first
    LsbFirst,
    /// Most significant bit first. MSB first is the default SPI behavior.
    MsbFirst,
}

/// A structure for specifying SPI configuration.
///
/// This structure uses builder semantics to generate the configuration.
///
/// `Example`
/// ```
/// use embedded_hal::spi::Mode;
///
/// let config = Config::new(Mode::MODE_0)
///     .manage_cs()
/// ```
#[derive(Copy, Clone)]
pub struct Config {
    mode: Mode,
    swap_miso_mosi: bool,
    hardware_cs: HardwareCS,
    inter_word_delay: f32,
    communication_mode: CommunicationMode,
    endianness: Endianness,
}

impl Config {
    /// Create a default configuration for the SPI interface.
    ///
    /// Arguments:
    /// * `mode` - The SPI mode to configure.
    pub fn new(mode: Mode) -> Self {
        Config {
            mode,
            swap_miso_mosi: false,
            hardware_cs: HardwareCS {
                mode: HardwareCSMode::Disabled,
                assertion_delay: 0.0,
                polarity: Polarity::IdleHigh,
            },
            inter_word_delay: 0.0,
            communication_mode: CommunicationMode::FullDuplex,
            endianness: Endianness::MsbFirst,
        }
    }

    /// Specify that the SPI MISO/MOSI lines are swapped.
    ///
    /// Note:
    /// * This function updates the HAL peripheral to treat the pin provided in the MISO parameter
    ///   as the MOSI pin and the pin provided in the MOSI parameter as the MISO pin.
    #[must_use]
    pub fn swap_mosi_miso(mut self) -> Self {
        self.swap_miso_mosi = true;
        self
    }

    /// Specify the behaviour of the hardware chip select.
    ///
    /// This also affects the way data is sent using [HardwareCSMode].
    /// By default the hardware cs is disabled.
    #[must_use]
    pub fn hardware_cs<CS>(mut self, hardware_cs: CS) -> Self
    where
        CS: Into<HardwareCS>,
    {
        self.hardware_cs = hardware_cs.into();
        self
    }

    /// Specify the time in seconds that should be idled between every data word being sent.
    ///
    /// Note:
    /// * This value is converted to a number of spi peripheral clock ticks and at most 15 of those.
    #[must_use]
    pub fn inter_word_delay(mut self, inter_word_delay: f32) -> Self {
        self.inter_word_delay = inter_word_delay;
        self
    }

    /// Select the communication mode of the SPI bus.
    #[must_use]
    pub fn communication_mode(mut self, mode: CommunicationMode) -> Self {
        self.communication_mode = mode;
        self
    }

    // /// Select endianness is used for data transfers
    pub fn endianness(mut self, endianness: Endianness) -> Self {
        self.endianness = endianness;
        self
    }
}

impl From<Mode> for Config {
    fn from(mode: Mode) -> Self {
        Self::new(mode)
    }
}

/// Object containing the settings for the hardware chip select pin
#[derive(Clone, Copy)]
pub struct HardwareCS {
    /// The value that determines the behaviour of the hardware chip select pin.
    pub mode: HardwareCSMode,
    /// The delay between CS assertion and the beginning of the SPI transaction in seconds.
    ///
    /// Note:
    /// * This value introduces a delay on SCK from the initiation of the transaction. The delay
    ///   is specified as a number of SCK cycles, so the actual delay may vary.
    pub assertion_delay: f32,
    /// The polarity of the CS pin.
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HardwareCSMode {
    /// Handling the CS is left for the user to do in software
    Disabled,
    /// The CS will assert and de-assert for each word being sent
    WordTransaction,
    /// The CS will assert and only de-assert after the whole frame is sent.
    ///
    /// When this mode is active, the blocking embedded hal interface automatically
    /// sets up the frames so it will be one frame per call.
    ///
    /// Note:
    /// * If using the non-blocking APIs, this mode does require some maintenance. Before sending,
    ///   you must setup the frame with [Spi::setup_transaction]. After everything has been sent,
    ///   you must also clean it up with [Spi::end_transaction].
    FrameTransaction,
}

impl HardwareCS {
    pub fn new(mode: HardwareCSMode) -> Self {
        HardwareCS {
            mode,
            assertion_delay: 0.0,
            polarity: Polarity::IdleHigh,
        }
    }

    pub fn assertion_delay(mut self, delay: f32) -> Self {
        self.assertion_delay = delay;
        self
    }

    pub fn polarity(mut self, polarity: Polarity) -> Self {
        self.polarity = polarity;
        self
    }

    fn enabled(&self) -> bool {
        !matches!(self.mode, HardwareCSMode::Disabled)
    }

    fn interleaved_cs(&self) -> bool {
        matches!(self.mode, HardwareCSMode::WordTransaction { .. })
    }
}

impl From<HardwareCSMode> for HardwareCS {
    fn from(value: HardwareCSMode) -> Self {
        HardwareCS::new(value)
    }
}

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;

/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;

/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

macro_rules! pins {
    ($($SPIX:ty:
       SCK: [$($( #[ $pmeta1:meta ] )* $SCK:ty),*]
       MISO: [$($( #[ $pmeta2:meta ] )* $MISO:ty),*]
       MOSI: [$($( #[ $pmeta3:meta ] )* $MOSI:ty),*]
       HCS: [$($( #[ $pmeta4:meta ] )* $HCS:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl PinSck<$SPIX> for $SCK {}
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl PinMiso<$SPIX> for $MISO {}
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl PinMosi<$SPIX> for $MOSI {}
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl PinHCS<$SPIX> for $HCS {}
            )*
        )+
    }
}

#[cfg(feature = "rm0492")]
pins! {
    SPI1:
        SCK: [
            NoSck,
            gpio::PA2<Alternate<4>>,
            gpio::PA5<Alternate<5>>,
            gpio::PA8<Alternate<12>>,
            gpio::PB3<Alternate<5>>,
            gpio::PC0<Alternate<5>>,
            gpio::PC5<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PA0<Alternate<12>>,
            gpio::PA3<Alternate<4>>,
            gpio::PA6<Alternate<5>>,
            gpio::PA9<Alternate<4>>,
            gpio::PB4<Alternate<5>>,
            gpio::PC2<Alternate<4>>,
            gpio::PC10<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA4<Alternate<4>>,
            gpio::PA7<Alternate<5>>,
            gpio::PB5<Alternate<5>>,
            gpio::PC3<Alternate<4>>,
            gpio::PC7<Alternate<5>>
        ]
        HCS: [
            gpio::PA1<Alternate<4>>,
            gpio::PA4<Alternate<5>>,
            gpio::PA15<Alternate<5>>,
            gpio::PB8<Alternate<12>>,
            gpio::PC1<Alternate<4>>,
            gpio::PC8<Alternate<5>>
        ]
    SPI2:
        SCK: [
            NoSck,
            gpio::PA5<Alternate<7>>,
            gpio::PA9<Alternate<5>>,
            gpio::PA12<Alternate<5>>,
            gpio::PB2<Alternate<6>>,
            gpio::PB10<Alternate<5>>,
            gpio::PB13<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PA7<Alternate<11>>,
            gpio::PA15<Alternate<7>>,
            gpio::PB5<Alternate<6>>,
            gpio::PB14<Alternate<5>>,
            gpio::PC2<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA8<Alternate<6>>,
            gpio::PB1<Alternate<6>>,
            gpio::PB15<Alternate<5>>,
            gpio::PC1<Alternate<5>>,
            gpio::PC3<Alternate<5>>
        ]
        HCS: [
            gpio::PA3<Alternate<5>>,
            gpio::PA8<Alternate<11>>,
            gpio::PA11<Alternate<5>>,
            gpio::PB4<Alternate<7>>,
            gpio::PB12<Alternate<5>>
        ]
    SPI3:
        SCK: [
            NoSck,
            gpio::PA1<Alternate<6>>,
            gpio::PA15<Alternate<10>>,
            gpio::PB3<Alternate<6>>,
            gpio::PB7<Alternate<6>>,
            gpio::PC10<Alternate<6>>
        ]
        MISO: [
            NoMiso,
            gpio::PA2<Alternate<6>>,
            gpio::PA4<Alternate<10>>,
            gpio::PB4<Alternate<6>>,
            gpio::PB15<Alternate<6>>,
            gpio::PC11<Alternate<6>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA3<Alternate<6>>,
            gpio::PA5<Alternate<10>>,
            gpio::PB2<Alternate<7>>,
            gpio::PB5<Alternate<7>>,
            gpio::PC12<Alternate<6>>,
            gpio::PA9<Alternate<10>>
        ]
        HCS: [
            gpio::PA4<Alternate<6>>,
            gpio::PA15<Alternate<6>>,
            gpio::PB10<Alternate<6>>,
            gpio::PA0<Alternate<10>>,
            gpio::PD2<Alternate<6>>
        ]
}

// Note: pin data is taken from stm32h56x, stm32h573, stm32h523 and stm32h533 datasheets
#[cfg(any(feature = "h523_h533", feature = "h56x_h573"))]
pins! {
    SPI1:
        SCK: [
            NoSck,
            gpio::PA5<Alternate<5>>,
            gpio::PB3<Alternate<5>>,
            gpio::PG11<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PA6<Alternate<5>>,
            gpio::PB4<Alternate<5>>,
            gpio::PG9<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA7<Alternate<5>>,
            gpio::PB5<Alternate<5>>,
            #[cfg(feature = "h523_h533")]
            gpio::PB15<Alternate<6>>,
            gpio::PD7<Alternate<5>>
        ]
        HCS: [
            gpio::PA4<Alternate<5>>,
            gpio::PA15<Alternate<5>>,
            gpio::PG10<Alternate<5>>
        ]
    SPI2:
        SCK: [
            NoSck,
            gpio::PA9<Alternate<5>>,
            gpio::PA12<Alternate<5>>,
            gpio::PB10<Alternate<5>>,
            gpio::PB13<Alternate<5>>,
            gpio::PD3<Alternate<5>>,
            #[cfg(feature = "h56x_h573")]
            gpio::PI1<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PB14<Alternate<5>>,
            gpio::PC2<Alternate<5>>,
            #[cfg(feature = "h56x_h573")]
            gpio::PI2<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PB15<Alternate<5>>,
            gpio::PC1<Alternate<5>>,
            gpio::PC3<Alternate<5>>,
            gpio::PG1<Alternate<7>>,
            #[cfg(feature = "h56x_h573")]
            gpio::PI3<Alternate<5>>
        ]
        HCS: [
            gpio::PA3<Alternate<5>>,
            gpio::PA11<Alternate<5>>,
            #[cfg(feature = "h523_h533")]
            gpio::PB1<Alternate<5>>,
            gpio::PB4<Alternate<7>>,
            gpio::PB9<Alternate<5>>,
            gpio::PB12<Alternate<5>>,
            #[cfg(feature = "h56x_h573")]
            gpio::PI0<Alternate<5>>
        ]
    SPI3:
        SCK: [
            NoSck,
            #[cfg(feature = "h523_h533")]
            gpio::PB1<Alternate<4>>,
            gpio::PB3<Alternate<6>>,
            #[cfg(feature = "h523_h533")]
            gpio::PB9<Alternate<6>>,
            gpio::PC10<Alternate<6>>
        ]
        MISO: [
            NoMiso,
            #[cfg(feature = "h523_h533")]
            gpio::PB0<Alternate<5>>,
            gpio::PB4<Alternate<6>>,
            gpio::PC11<Alternate<6>>,
            #[cfg(feature = "h523_h533")]
            gpio::PD7<Alternate<6>>
        ]
        MOSI: [
            NoMosi,
            #[cfg(feature = "h523_h533")]
            gpio::PA3<Alternate<6>>,
            #[cfg(feature = "h523_h533")]
            gpio::PA4<Alternate<4>>,
            gpio::PB2<Alternate<7>>,
            gpio::PB5<Alternate<7>>,
            gpio::PC12<Alternate<6>>,
            gpio::PD6<Alternate<5>>,
            #[cfg(feature = "h523_h533")]
            gpio::PG8<Alternate<5>>
        ]
        HCS: [
            gpio::PA4<Alternate<6>>,
            gpio::PA15<Alternate<6>>,
            #[cfg(feature = "h523_h533")]
            gpio::PB8<Alternate<6>>
        ]
    SPI4:
        SCK: [
            NoSck,
            #[cfg(feature = "h523_h533")]
            gpio::PA0<Alternate<5>>,
            #[cfg(feature = "h523_h533")]
            gpio::PC5<Alternate<6>>,
            gpio::PE2<Alternate<5>>,
            gpio::PE12<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            #[cfg(feature = "h523_h533")]
            gpio::PC0<Alternate<6>>,
            #[cfg(feature = "h523_h533")]
            gpio::PB7<Alternate<5>>,
            gpio::PE5<Alternate<5>>,
            gpio::PE13<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            #[cfg(feature = "h523_h533")]
            gpio::PA8<Alternate<6>>,
            #[cfg(feature = "stm32h523")]
            gpio::PC1<Alternate<6>>,
            gpio::PE6<Alternate<5>>,
            gpio::PE14<Alternate<5>>
        ]
        HCS: [
            gpio::PE4<Alternate<5>>,
            gpio::PE11<Alternate<5>>
        ]

}

#[cfg(feature = "h56x_h573")]
pins! {
    SPI5:
        SCK: [
            NoSck,
            gpio::PF7<Alternate<5>>,
            gpio::PH6<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PF8<Alternate<5>>,
            gpio::PH7<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PF9<Alternate<5>>,
            gpio::PF11<Alternate<5>>,
            gpio::PH8<Alternate<5>>
        ]
        HCS: [
            gpio::PF6<Alternate<5>>,
            gpio::PH5<Alternate<5>>,
            gpio::PH9<Alternate<5>>
        ]
    SPI6:
        SCK: [
            NoSck,
            gpio::PA5<Alternate<8>>,
            gpio::PB3<Alternate<8>>,
            gpio::PC12<Alternate<5>>,
            gpio::PG13<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PA6<Alternate<8>>,
            gpio::PB4<Alternate<8>>,
            gpio::PG12<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA7<Alternate<8>>,
            gpio::PB5<Alternate<8>>,
            gpio::PG14<Alternate<5>>
        ]
        HCS: [
            gpio::PA0<Alternate<5>>,
            gpio::PA4<Alternate<8>>,
            gpio::PA15<Alternate<7>>,
            gpio::PG8<Alternate<5>>
        ]
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Event {
    /// New data has been received
    Rxp,
    /// Data can be sent
    Txp,
    /// An error occurred
    Error,
    /// End of transaction
    Eot,
}

#[derive(Debug)]
pub struct Inner<SPI, W: FrameSize<SPI>> {
    spi: SPI,
    _word: PhantomData<W>,
}

/// Spi in Master mode
#[derive(Debug)]
pub struct Spi<SPI, W: FrameSize<SPI> = u8> {
    inner: Inner<SPI, W>,
    _word: PhantomData<W>,
}

impl<SPI, W: FrameSize<SPI>> Deref for Spi<SPI, W> {
    type Target = Inner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI, W: FrameSize<SPI>> DerefMut for Spi<SPI, W> {
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

// Implemented by all SPI instances
macro_rules! instance {
    ($SPIX:ident: $Spi:ident, $SPICLKSEL:ident) => { paste::item! {
        pub type $Spi<W = u8> = Spi<stm32::$SPIX, W>;

        impl Instance for $SPIX {
            type Rec = rec::$Spi;

            fn ptr() -> *const spi1::RegisterBlock {
                <$SPIX>::ptr() as *const _
            }

            fn clock(clocks: &CoreClocks) -> Hertz {
                let ccipr3 = unsafe { (*stm32::RCC::ptr()).ccipr3().read() };

                match ccipr3.[<$SPIX:lower sel>]().variant() {
                    Some(ccipr3::[<$SPICLKSEL SEL>]::Pll1Q) => clocks.pll1_q_ck(),
                    Some(ccipr3::[<$SPICLKSEL SEL>]::Pll2P) => clocks.pll2_p_ck(),
                    Some(ccipr3::[<$SPICLKSEL SEL>]::Audioclk) => clocks.audio_ck(),
                    Some(ccipr3::[<$SPICLKSEL SEL>]::PerCk) => clocks.per_ck(),
                    _ => unreachable!(),
                }.expect("Source clock not enabled")
            }

            fn rec() -> Self::Rec {
                rec::$Spi { _marker: PhantomData }
            }
        }

        impl crate::Sealed for $SPIX {}
    }};

}

instance! { SPI1: Spi1, SPI123 }
instance! { SPI2: Spi2, SPI123 }
instance! { SPI3: Spi3, SPI123 }

pub trait FrameSize<SPI>: Copy + Default + 'static + crate::Sealed {
    const BITS: u8;
}

macro_rules! framesize {
    ($type:ty) => {
        impl<SPI> FrameSize<SPI> for $type {
            const BITS: u8 = <$type>::BITS as u8;
        }
        impl crate::Sealed for $type {}
    };
}

macro_rules! framesize_u32 {
    ($SPIx:ty) => {
        impl FrameSize<$SPIx> for u32 {
            const BITS: u8 = u32::BITS as u8;
        }
    };
}
impl crate::Sealed for u32 {}

framesize!(u16);
framesize!(u8);

// Only SPI[1,2,3] support 32bit data size
framesize_u32!(SPI1);
framesize_u32!(SPI2);
framesize_u32!(SPI3);

pub trait SpiExt<SPI: Instance, W: FrameSize<SPI> = u8> {
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

impl<SPI: Instance, W: FrameSize<SPI>> SpiExt<SPI, W> for SPI {
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

impl<SPI: Instance, W: FrameSize<SPI>> Spi<SPI, W> {
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
            Err(nb::Error::Other(Error::Overrun))
        } else if $sr.udr().is_underrun() {
            Err(nb::Error::Other(Error::Underrun))
        } else if $sr.modf().is_fault() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if $sr.crce().is_error() {
            Err(nb::Error::Other(Error::Crc))
        } else {
            Ok(())
        }
    };
}

impl<SPI: Instance, W: FrameSize<SPI>> Inner<SPI, W> {
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

    /// Check if SPI is enabled
    fn is_enabled(&self) -> bool {
        self.spi.cr1().read().spe().is_enabled()
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

    /// Enable interrupts for the given `event`:
    ///  - Received data ready to be read (RXP)
    ///  - Transmit data register empty (TXP)
    ///  - Error
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxp => self.spi.ier().modify(|_, w| w.rxpie().not_masked()),
            Event::Txp => self.spi.ier().modify(|_, w| w.txpie().not_masked()),
            Event::Error => self.spi.ier().modify(|_, w| {
                w.udrie() // Underrun
                    .not_masked()
                    .ovrie() // Overrun
                    .not_masked()
                    .crceie() // CRC error
                    .not_masked()
                    .modfie() // Mode fault
                    .not_masked()
            }),
            Event::Eot => self.spi.ier().modify(|_, w| w.eotie().not_masked()),
        };
    }

    /// Disable interrupts for the given `event`:
    ///  - Received data ready to be read (RXP)
    ///  - Transmit data register empty (TXP)
    ///  - Error
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxp => self.spi.ier().modify(|_, w| w.rxpie().masked()),
            Event::Txp => self.spi.ier().modify(|_, w| w.txpie().masked()),
            Event::Error => {
                self.spi.ier().modify(|_, w| {
                    w.udrie() // Underrun
                        .masked()
                        .ovrie() // Overrun
                        .masked()
                        .crceie() // CRC error
                        .masked()
                        .modfie() // Mode fault
                        .masked()
                })
            }
            Event::Eot => self.spi.ier().modify(|_, w| w.eotie().masked()),
        };
        let _ = self.spi.ier().read();
        let _ = self.spi.ier().read(); // Delay 2 peripheral clocks
    }

    /// Clears the MODF flag, which indicates that a
    /// mode fault has occurred.
    #[inline]
    fn clear_modf(&mut self) {
        self.spi.ifcr().write(|w| w.modfc().clear());
        let _ = self.spi.sr().read();
        let _ = self.spi.sr().read(); // Delay 2 peripheral clocks
    }

    /// Returns true if the transfer has been completed
    #[inline]
    pub fn is_eot(&self) -> bool {
        self.spi.sr().read().eot().is_completed()
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
    fn check_read(&mut self) -> nb::Result<W, Error> {
        let sr = self.spi.sr().read();

        check_status_error!(sr)?;

        if sr.rxp().is_not_empty() {
            Ok(self.read_data_reg())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[inline(always)]
    fn check_write(&mut self, word: W) -> nb::Result<(), Error> {
        let sr = self.spi.sr().read();

        check_status_error!(sr)?;

        if sr.txp().is_not_full() {
            self.write_data_reg(word);
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Non-blocking write. Will write out as many words as are allowed until the operation would
    /// block. The amount of words written are returned wrapped in the Ok value.
    /// Note: this does not return nb::Error::WouldBlock
    pub fn write_nb(&mut self, words: &[W]) -> Result<usize, Error> {
        for (i, word) in words.iter().enumerate() {
            match self.check_write(*word) {
                Err(nb::Error::WouldBlock) => return Ok(i),
                Err(nb::Error::Other(error)) => return Err(error),
                Ok(_) => {}
            }
        }
        Ok(words.len())
    }

    fn flush_write_nb(&mut self, len: usize) -> Result<usize, Error> {
        for i in 0..len {
            match self.check_write(W::default()) {
                Err(nb::Error::WouldBlock) => return Ok(i),
                Err(nb::Error::Other(error)) => return Err(error),
                Ok(_) => {}
            }
        }
        Ok(len)
    }

    /// Non-blocking read. Will read as many words as are allowed until the operation would
    /// block. The amount of words written are returned wrapped in the Ok value.
    /// Note: this does not return nb::Error::WouldBlock
    pub fn read_nb(&mut self, words: &mut [W]) -> Result<usize, Error> {
        for (i, word) in words.iter_mut().enumerate() {
            match self.check_read() {
                Err(nb::Error::WouldBlock) => return Ok(i),
                Err(nb::Error::Other(error)) => return Err(error),
                Ok(data) => *word = data,
            }
        }
        Ok(words.len())
    }

    fn flush_read_nb(&mut self, len: usize) -> Result<usize, Error> {
        for i in 0..len {
            match self.check_read() {
                Err(nb::Error::WouldBlock) => return Ok(i),
                Err(nb::Error::Other(error)) => return Err(error),
                Ok(_) => {}
            }
        }
        Ok(len)
    }

    fn flush_read_all_nb(&mut self) -> Result<(), Error> {
        loop {
            match self.check_read() {
                Err(nb::Error::WouldBlock) => return Ok(()),
                Err(nb::Error::Other(error)) => return Err(error),
                Ok(_) => {}
            }
        }
    }

    fn write_words(&mut self, words: &[W]) -> Result<(), Error> {
        let mut i = 0;
        i += self.write_nb(words)?;

        // Continue filling write FIFO and emptying read FIFO
        while i < words.len() {
            self.flush_read_all_nb()?;
            i += self.write_nb(&words[i..])?;
        }

        // Final flush of read buffer
        self.flush_read_all_nb()
    }

    fn read_words(&mut self, words: &mut [W]) -> Result<(), Error> {
        let mut flushed = self.flush_write_nb(words.len())?;

        let mut i = 0;
        while i < words.len() {
            i += self.read_nb(&mut words[i..])?;
            flushed += self.flush_write_nb(words.len() - flushed)?;
        }

        Ok(())
    }

    fn transfer_words_inplace(&mut self, words: &mut [W]) -> Result<(), Error> {
        let mut write_idx = 0;
        let mut read_idx = 0;
        write_idx += self.write_nb(&words[write_idx..])?;

        while write_idx < words.len() || read_idx < words.len() {
            if read_idx < words.len() {
                read_idx += self.read_nb(&mut words[read_idx..])?;
            }
            if write_idx < words.len() {
                write_idx += self.write_nb(&words[write_idx..])?;
            }
        }
        Ok(())
    }
}

impl<SPI: Instance, W: FrameSize<SPI>> Spi<SPI, W> {
    /// Sets up a frame transaction with the given amount of data words.
    ///
    /// If this is called when a transaction has already started,
    /// then an error is returned with [Error::TransactionAlreadyStarted].
    pub fn setup_transaction(
        &mut self,
        length: NonZeroUsize,
    ) -> Result<(), Error> {
        if self.is_enabled() {
            return Err(Error::TransactionAlreadyStarted);
        }

        if length.get() > u16::MAX.into() {
            return Err(Error::BufferTooBig {
                max_size: u16::MAX.into(),
            });
        }

        self.spi.cr2().write(|w| w.tsize().set(length.get() as u16));

        // Re-enable
        self.clear_modf();
        self.enable();
        self.spi.cr1().modify(|_, w| w.cstart().started());

        Ok(())
    }

    /// Ends the current transaction. Waits for all data to be transmitted
    pub fn end_transaction(&mut self) {
        // Result is only () or WouldBlock. Discard result.
        let _ = nb::block!(self.end_transaction_nb());
    }

    pub fn abort_transaction(&mut self) {
        self.disable();
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

#[derive(Debug)]
pub struct NonBlockingTransfer<'a, W> {
    write: &'a [W],
    read: &'a mut [W],
    write_idx: usize,
    read_idx: usize,
    len: usize,
}

impl<'a, W> NonBlockingTransfer<'a, W> {
    pub fn new(write: &'a [W], read: &'a mut [W]) -> Self {
        let len = core::cmp::max(read.len(), write.len());
        NonBlockingTransfer {
            write,
            read,
            write_idx: 0,
            read_idx: 0,
            len,
        }
    }

    fn has_started(&self) -> bool {
        self.read_idx > 0 || self.write_idx > 0
    }

    fn is_complete(&self) -> bool {
        self.read_idx >= self.read.len() && self.write_idx >= self.write.len()
    }

    fn write_to_spi_nb<SPI>(
        &mut self,
        spi: &mut Inner<SPI, W>,
    ) -> Result<(), Error>
    where
        SPI: Instance,
        W: FrameSize<SPI>,
    {
        if self.write_idx < self.write.len() {
            self.write_idx += spi.write_nb(&self.write[self.write_idx..])?;
        }

        if self.write_idx >= self.write.len() && self.write_idx < self.len {
            self.write_idx += spi.flush_write_nb(self.len - self.write_idx)?;
        }

        Ok(())
    }

    fn read_from_spi_nb<SPI>(
        &mut self,
        spi: &mut Inner<SPI, W>,
    ) -> Result<(), Error>
    where
        SPI: Instance,
        W: FrameSize<SPI>,
    {
        if self.read_idx < self.read.len() {
            self.read_idx += spi.read_nb(&mut self.read[self.read_idx..])?;
        }

        if self.read_idx >= self.read.len() && self.read_idx < self.len {
            self.read_idx += spi.flush_read_nb(self.len - self.read_idx)?;
        }

        Ok(())
    }

    fn exchange_nb<SPI>(
        &mut self,
        spi: &mut Inner<SPI, W>,
    ) -> nb::Result<(), Error>
    where
        SPI: Instance,
        W: FrameSize<SPI>,
    {
        self.write_to_spi_nb(spi)?;
        self.read_from_spi_nb(spi)?;
        if self.is_complete() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

// Non-blocking operations
impl<SPI: Instance, W: FrameSize<SPI>> Spi<SPI, W> {
    /// Ends the current transaction. This must always be called when all data has been sent to
    /// properly terminate the transaction and reset the SPI peripheral. Returns
    /// nb::Error::WouldBlock while a transfer is in progress (according to SR:TXC)
    pub fn end_transaction_nb(&mut self) -> nb::Result<(), Error> {
        if self.spi.sr().read().txc().is_ongoing() {
            return Err(nb::Error::WouldBlock);
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
        Ok(())
    }

    pub fn transfer_nb(
        &mut self,
        transfer: &mut NonBlockingTransfer<W>,
    ) -> nb::Result<(), Error> {
        if transfer.is_complete() {
            return Err(Error::TransferAlreadyComplete.into());
        }
        if !transfer.has_started() {
            self.setup_transaction(NonZeroUsize::new(transfer.len).unwrap())?;
        }
        match transfer.exchange_nb(self) {
            Ok(()) => Ok(()),
            Err(nb::Error::WouldBlock) => Err(nb::Error::WouldBlock),
            Err(nb::Error::Other(error)) => {
                self.abort_transaction();
                Err(error.into())
            }
        }
    }
}

// Implement blocking transaction interface for Spi
impl<SPI: Instance, W: FrameSize<SPI>> Spi<SPI, W> {
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

        self.setup_transaction(NonZeroUsize::new(words.len()).unwrap())?;

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

        self.setup_transaction(NonZeroUsize::new(words.len()).unwrap())?;

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

        let mut transfer = NonBlockingTransfer::<W>::new(write, read);
        nb::block!(self.transfer_nb(&mut transfer))?;

        self.end_transaction();

        Ok(())
    }

    /// Full duplex transfer with single buffer for transmit and receive
    fn transfer_inplace(&mut self, words: &mut [W]) -> Result<(), Error> {
        if self.communication_mode() != CommunicationMode::FullDuplex {
            return Err(Error::InvalidOperation);
        }

        if words.is_empty() {
            return Ok(());
        }

        self.setup_transaction(NonZeroUsize::new(words.len()).unwrap())?;
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
