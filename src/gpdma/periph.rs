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

/// `PeriphTxBuffer` is a wrapper around a peripheral's transmit data register address, used to
/// provide a WriteBuffer implementation for initiating memory-to-peripheral DMA transfers.
pub struct PeriphTxBuffer<A: TxAddr<W>, W: Word> {
    _addr: PhantomData<A>,
    _word: PhantomData<W>,
}

unsafe impl<A: TxAddr<W>, W: Word> WriteBuffer for PeriphTxBuffer<A, W> {
    type Word = W;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (A::tx_addr(), 1)
    }
}

/// `PeriphRxBuffer` is a wrapper around a peripheral's receive data register address, used to
/// provide a ReadBuffer implementation for initiating peripheral-to-memory DMA transfers.
pub struct PeriphRxBuffer<A: RxAddr<W>, W: Word> {
    _addr: PhantomData<A>,
    _word: PhantomData<W>,
}

unsafe impl<A: RxAddr<W>, W: Word> ReadBuffer for PeriphRxBuffer<A, W> {
    type Word = W;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (A::rx_addr(), 1)
    }
}

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

trait TxBuffer<W: Word> {
    /// Returns a `PeriphTxBuffer` that provides a write buffer for the peripheral's transmit data
    /// register. This is used to initiate memory-to-peripheral DMA transfers. Implemented
    /// automatically for any implementer of `TxAddr`.
    ///
    /// # Safety
    /// TxAddr already requires the caller to ensure that the returned pointer is valid and as such
    /// is marked unsafe, so marking this method as unsafe is redundant.
    fn tx_buffer() -> PeriphTxBuffer<Self, W>
    where
        Self: TxAddr<W> + Sized,
    {
        PeriphTxBuffer {
            _addr: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<W: Word, T: TxAddr<W>> TxBuffer<W> for T {}

trait RxBuffer<W: Word> {
    /// Returns a `PeriphRxBuffer` that provides a read buffer for the peripheral's receive data
    /// register. This is used to initiate peripheral-to-memory DMA transfers. Implemented
    /// automatically for any implementer of `RxAddr`.
    ///
    /// # Safety
    /// RxAddr already requires the caller to ensure that the returned pointer is valid and as such
    /// is marked unsafe, so marking this method as unsafe is redundant.
    fn rx_buffer() -> PeriphRxBuffer<Self, W>
    where
        Self: RxAddr<W> + Sized,
    {
        PeriphRxBuffer {
            _addr: PhantomData,
            _word: PhantomData,
        }
    }
}

impl<W: Word, T: RxAddr<W>> RxBuffer<W> for T {}

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

impl<PERIPH, W, CH> Sealed for DmaRx<PERIPH, W, CH> {}

impl<PERIPH, W, CH> DmaRx<PERIPH, W, CH>
where
    PERIPH: RxAddr<W>,
    CH: DmaChannel,
    W: Word,
{
    pub fn init_rx_transfer<'a, D>(
        &'a mut self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        destination: D,
    ) -> DmaTransfer<'a, CH, PeriphRxBuffer<PERIPH, W>, D>
    where
        D: WriteBuffer<Word = W>,
    {
        DmaTransfer::peripheral_to_memory(
            config,
            &mut self.channel,
            PERIPH::rx_buffer(),
            destination,
        )
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

impl<PERIPH, W, CH> Sealed for DmaTx<PERIPH, W, CH> {}

impl<PERIPH, W, CH> DmaTx<PERIPH, W, CH>
where
    PERIPH: TxAddr<W>,
    CH: DmaChannel,
    W: Word,
{
    pub fn init_tx_transfer<'a, S>(
        &'a mut self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        source: S,
    ) -> DmaTransfer<'a, CH, S, PeriphTxBuffer<PERIPH, W>>
    where
        S: ReadBuffer<Word = W>,
    {
        DmaTransfer::memory_to_peripheral(
            config,
            &mut self.channel,
            source,
            PERIPH::tx_buffer(),
        )
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

impl<PERIPH, W, TX, RX> DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: Word,
    TX: DmaChannel,
    RX: DmaChannel,
{
    pub fn init_duplex_transfer<'a, S, D>(
        &'a mut self,
        tx_config: DmaConfig<MemoryToPeripheral, W, W>,
        rx_config: DmaConfig<PeripheralToMemory, W, W>,
        source: S,
        destination: D,
    ) -> (
        DmaTransfer<'a, TX, S, PeriphTxBuffer<PERIPH, W>>,
        DmaTransfer<'a, RX, PeriphRxBuffer<PERIPH, W>, D>,
    )
    where
        S: ReadBuffer<Word = W>,
        D: WriteBuffer<Word = W>,
    {
        let tx = self.tx.init_tx_transfer(tx_config, source);
        let rx = self.rx.init_rx_transfer(rx_config, destination);
        (tx, rx)
    }
}
