//! This module provides traits and structs for managing DMA transactions for peripherals.
//!  - By implementing the [`TxAddr`] and [`RxAddr`] traits, peripherals can be used with DMA for
//!    memory-to-peripheral and peripheral-to-memory transfers, respectively.
//!  - The [`Tx`] and [`Rx`] traits provide a define the interface for initiating DMA transfers for
//!    TX and RX operations, respectively.
//!  - The [`DmaTx`], [`DmaRx`] structs implement the [`Tx`] and [`Rx`] traits, respectively, and
//!    encapsulate the logic for initializing these transfers.
//!  - The [`DmaDuplex`] struct combines both TX and RX capabilities, allowing for full-duplex
//!    operations.
use core::marker::PhantomData;

use crate::Sealed;

use super::{
    DmaChannel, DmaConfig, DmaTransfer, MemoryToPeripheral, PeripheralToMemory,
    ReadBuffer, Word, WriteBuffer,
};

/// `TxAddr` is a trait that provides a method to obtain the address of the transmit data register
/// of a peripheral. This is used to facilitate memory-to-peripheral DMA transactions. The
/// peripheral must implement this trait.
pub trait TxAddr<W: Word> {
    /// Returns a pointer to the peripheral's transmit data register.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the returned pointer is only used when it is valid to access
    /// the peripheral's transmit data register, and that no data races or invalid memory accesses
    /// occur.
    unsafe fn tx_addr() -> *mut W;
}

/// `RxAddr` is a trait that provides a method to obtain the address of the receive data register
/// of a peripheral. This is used to facilitate peripheral-to-memory DMA transactions. The
/// peripheral must implement this trait.
pub trait RxAddr<W: Word> {
    /// Returns a pointer to the peripheral's receive data register.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the returned pointer is only used when it is valid to access
    /// the peripheral's receive data register, and that no data races or invalid memory accesses
    /// occur.
    unsafe fn rx_addr() -> *const W;
}

/// The `Tx` trait to defines the method needed for a peripheral DMA struct (ie. [`DmaTx`] or
/// [`DmaDuplex`]) that is used to initiate a memory-to-peripheral DMA transaction. It also
/// functions as a marker trait to indicate that the peripheral DMA struct can be used for
/// initiating transmissions.
pub trait Tx<W>: Sealed {
    type CH: DmaChannel;
    fn init_tx_transfer<'a>(
        &'a self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        words: &'a [W],
    ) -> DmaTransfer<'a, Self::CH>;
}

/// The `Rx` trait to defines the method needed for a peripheral DMA struct (ie. [`DmaRx`] or
/// [`DmaDuplex`]) that is used to initiate a peripheral-to-memory DMA transaction. It also
/// functions as a marker trait to indicate that the peripheral DMA struct can be used for
/// initiating receiving transfers.
pub trait Rx<W>: Sealed {
    type CH: DmaChannel;
    fn init_rx_transfer<'a>(
        &'a self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        words: &'a mut [W],
    ) -> DmaTransfer<'a, Self::CH>;
}

/// `DmaRx` encapsulates the initialization of a peripheral-to-memory DMA transaction for receiving
/// data. Used by peripheral DMA implementations.
pub struct DmaRx<PERIPH, W, CH> {
    _periph: PhantomData<PERIPH>,
    _word: PhantomData<W>,
    channel: CH,
}

impl<PERIPH, W, CH: DmaChannel> DmaRx<PERIPH, W, CH> {
    fn new(channel: CH) -> Self {
        Self {
            _periph: PhantomData,
            _word: PhantomData,
            channel,
        }
    }

    pub fn free(self) -> CH {
        self.channel
    }
}

impl<PERIPH, W, CH: DmaChannel> From<CH> for DmaRx<PERIPH, W, CH> {
    fn from(channel: CH) -> Self {
        Self::new(channel)
    }
}

unsafe impl<PERIPH: RxAddr<W>, W: Word, CH> ReadBuffer
    for &DmaRx<PERIPH, W, CH>
{
    type Word = W;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (PERIPH::rx_addr(), 1)
    }
}

impl<PERIPH, W, CH> Sealed for DmaRx<PERIPH, W, CH> {}

impl<PERIPH, W, CH> Rx<W> for DmaRx<PERIPH, W, CH>
where
    PERIPH: RxAddr<W>,
    CH: DmaChannel,
    W: Word,
{
    type CH = CH;
    fn init_rx_transfer<'a>(
        &'a self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        words: &'a mut [W],
    ) -> DmaTransfer<'a, CH> {
        DmaTransfer::peripheral_to_memory(config, &self.channel, self, words)
    }
}

/// `DmaTx` encapsulates the initialization of a memory-to-peripheral DMA transaction for
/// transmitting data. Used by peripheral DMA implementations.
pub struct DmaTx<PERIPH, W, CH> {
    _periph: PhantomData<PERIPH>,
    _word: PhantomData<W>,
    channel: CH,
}

impl<PERIPH, W, CH: DmaChannel> DmaTx<PERIPH, W, CH> {
    fn new(channel: CH) -> Self {
        Self {
            _periph: PhantomData,
            _word: PhantomData,
            channel,
        }
    }

    pub fn free(self) -> CH {
        self.channel
    }
}

impl<PERIPH, W, CH: DmaChannel> From<CH> for DmaTx<PERIPH, W, CH> {
    fn from(channel: CH) -> Self {
        Self::new(channel)
    }
}

unsafe impl<PERIPH: TxAddr<W>, W: Word, CH> WriteBuffer
    for &DmaTx<PERIPH, W, CH>
{
    type Word = W;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (PERIPH::tx_addr(), 1)
    }
}

impl<PERIPH, W, CH> Sealed for DmaTx<PERIPH, W, CH> {}

impl<PERIPH, W, CH> Tx<W> for DmaTx<PERIPH, W, CH>
where
    PERIPH: TxAddr<W>,
    CH: DmaChannel,
    W: Word,
{
    type CH = CH;
    fn init_tx_transfer<'a>(
        &'a self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        words: &'a [W],
    ) -> DmaTransfer<'a, CH> {
        DmaTransfer::memory_to_peripheral(config, &self.channel, words, self)
    }
}

/// `DmaDuplex` encapsulates the initialization of both memory-to-peripheral and
/// peripheral-to-memory DMA transaction for to enable setting up of full-duplex transmission and
/// reception of data. Used by peripheral DMA implementations.
pub struct DmaDuplex<PERIPH, W, TX, RX> {
    tx: DmaTx<PERIPH, W, TX>,
    rx: DmaRx<PERIPH, W, RX>,
}

impl<PERIPH, W, TX, RX> DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: Word,
    TX: DmaChannel,
    RX: DmaChannel,
{
    pub fn new(tx: TX, rx: RX) -> Self {
        Self {
            tx: DmaTx::from(tx),
            rx: DmaRx::from(rx),
        }
    }

    pub fn free(self) -> (TX, RX) {
        (self.tx.free(), self.rx.free())
    }
}

impl<PERIPH, W, TX, RX> Sealed for DmaDuplex<PERIPH, W, TX, RX> {}

impl<PERIPH, W, TX, RX> Tx<W> for DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: Word,
    TX: DmaChannel,
    RX: DmaChannel,
{
    type CH = TX;
    fn init_tx_transfer<'a>(
        &'a self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        words: &'a [W],
    ) -> DmaTransfer<'a, TX> {
        self.tx.init_tx_transfer(config, words)
    }
}

impl<PERIPH, W, TX, RX> Rx<W> for DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: Word + Word,
    TX: DmaChannel,
    RX: DmaChannel,
{
    type CH = RX;
    fn init_rx_transfer<'a>(
        &'a self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        words: &'a mut [W],
    ) -> DmaTransfer<'a, RX> {
        self.rx.init_rx_transfer(config, words)
    }
}
