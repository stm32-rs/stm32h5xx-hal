//! Serial communication using USART & LPUART peripherals
//!
//! The serial module provides both asynchronous and synchronous functionality using the USART
//! and LPUART* (only asynchronous) peripherals. It also exposes some, but not all, of the advanced
//! functionality provided by the USART peripheral.
//!
//! It supports both blocking and non-blocking usage. Blocking usage is abstracted as an IO
//! stream and is exposed via embedded-io traits: [`Read`][io::Read], [`ReadReady`][io::ReadReady],
//! [`Write`][io::Write], and [`WriteReady`][io::WriteReady] are implemented.
//!
//! 7, 8, and 9-bit word sizes are supported. For 7- and 8-bit operation, transactions are performed
//! with u8 values. For 9-bit operation transactions are performed with u16 values. The correct word
//! size must be indicated to the compiler during initialization.
//!
//! # Usage
//!
//! ## Asynchronous serial via USART
//!
//! ### Intialization:
//! ```
//! use stm32h5xx_hal::serial;
//!
//! let dp = ...;           // Device peripherals
//! let (tx, rx) = ...;     // Configure pin
//!
//! let serial = dp.USART1
//!         .serial((tx, rx), 115_200.Hz(), ccdr.peripheral.USART1, &ccdr.clocks)
//!         .unwrap();
//! ```
//!
//! ### Non-blocking operation
//! ```
//! use stm32h5xx_hal::serial;
//!
//! ```
//!
//! ### Blocking operation
//!
//! ```
//! use stm32h5xx_hal::serial;
//! use embedded_hal_nb::serial;
//!
//! let bytes_written = serial.write(&[0x00, 0x11, 0x22]).unwrap();
//! let buf = &mut [0u8; 3];
//! let bytes_read = serial.read(buf).unwrap();
//! ```
//!
//! * Note: LPUART funcitionality is not yet implemented
//!
//! # Examples
//!
//! - [Simple Blocking Example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/serial.rs)
//!
//! [serial::Read]: https://docs.rs/embedded-hal-nb/1.0.0-rc.1/embedded_hal_nb/serial/trait.Read.html
//! [serial::Write]: https://docs.rs/embedded-hal-nb/1.0.0-rc.1/embedded_hal_nb/serial/trait.Write.html
//! [io::Read]: https://docs.rs/embedded-io/0.6.1/embedded_io/trait.Read.html
//! [io::ReadReady]: https://docs.rs/embedded-io/0.6.1/embedded_io/trait.ReadReady.html
//! [io::Write]: https://docs.rs/embedded-io/0.6.1/embedded_io/trait.Write.html
//! [io::WriteReady]: https://docs.rs/embedded-io/0.6.1/embedded_io/trait.WriteReady.html
//! [embedded-io]: https://docs.rs/embedded-io/0.6.1/embedded_io/

use embedded_io::{Error as IoError, ErrorKind as IoErrorKind};

pub(crate) mod usart;
pub use usart::{Instance, Serial};

/// Serial error
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

impl IoError for Error {
    fn kind(&self) -> IoErrorKind {
        match self {
            Error::Overrun => IoErrorKind::Other,
            Error::Framing => IoErrorKind::Other,
            Error::Noise => IoErrorKind::Other,
            Error::Parity => IoErrorKind::Other,
        }
    }
}

/// Interrupt event
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// Rx FIFO not empty
    RxNotEmpty,
    /// Tx FIFO not full
    TxNotFull,
    /// Idle line state detected
    Idle,
    ///Tx FIFO below threshlold
    TxFifoThreshold,
    ///Rx FIFO above threshold
    RxFifoThreshold,
}

pub trait WordBits: Copy + Default {
    const BITS: usize;
}

impl WordBits for u16 {
    const BITS: usize = u16::BITS as usize;
}

impl WordBits for u8 {
    const BITS: usize = u8::BITS as usize;
}

pub mod config;

pub trait Pins<USART> {
    const SYNCHRONOUS: bool = false;
}
pub trait PinTx<USART> {}
pub trait PinRx<USART> {}
pub trait PinCk<USART> {}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
{
}

impl<USART, TX, RX, CK> Pins<USART> for (TX, RX, CK)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
    CK: PinCk<USART>,
{
    const SYNCHRONOUS: bool = true;
}

/// A filler type for when the Tx pin is unnecessary
pub struct NoTx;
/// A filler type for when the Rx pin is unnecessary
pub struct NoRx;
/// A filler type for when the Ck pin is unnecessary
pub struct NoCk;
