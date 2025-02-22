use core::{marker::PhantomData, num::NonZeroUsize};

use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::gpdma::{
    config::{Config as DmaConfig, MemoryToPeripheral, PeripheralToMemory},
    Channel, Transfer as DmaTransfer, Word,
};

use super::{Error, FrameSize, Instance, Spi};

pub struct DmaRx<SPI, W> {
    _spi: PhantomData<SPI>,
    _word: PhantomData<W>,
}

impl<SPI, W> DmaRx<SPI, W> {
    fn new() -> Self {
        Self {
            _spi: PhantomData,
            _word: PhantomData,
        }
    }
}

unsafe impl<SPI: Instance, W: FrameSize> ReadBuffer for DmaRx<SPI, W> {
    type Word = W;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        ((*SPI::ptr()).rxdr().as_ptr() as *const W, 1)
    }
}

pub struct DmaTx<SPI, W> {
    _spi: PhantomData<SPI>,
    _word: PhantomData<W>,
}

impl<SPI, W> DmaTx<SPI, W> {
    fn new() -> Self {
        Self {
            _spi: PhantomData,
            _word: PhantomData,
        }
    }
}

unsafe impl<SPI: Instance, W: FrameSize> WriteBuffer for DmaTx<SPI, W> {
    type Word = W;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        ((*SPI::ptr()).txdr().as_ptr() as *mut W, 1)
    }
}

pub struct RxDmaTransfer<'a, SPI, W: FrameSize, CH, D> {
    spi: &'a mut Spi<SPI, W>,
    transfer: DmaTransfer<CH, DmaRx<SPI, W>, D, PeripheralToMemory>,
}

impl<'a, SPI, W, CH, D> RxDmaTransfer<'a, SPI, W, CH, D>
where
    SPI: Instance,
    W: FrameSize + Word,
    CH: Channel,
    D: WriteBuffer<Word = W>,
{
    pub fn new(spi: &'a mut Spi<SPI, W>, channel: CH, destination: D) -> Self {
        let config = DmaConfig::new().with_request(SPI::rx_dma_request());
        let source = DmaRx::new();
        let transfer = DmaTransfer::peripheral_to_memory(
            config,
            channel,
            source,
            destination,
        );
        Self { spi, transfer }
    }

    pub fn start(&mut self) -> Result<(), crate::gpdma::Error> {
        self.spi.enable_rx_dma();
        self.transfer.start_with(|_, _| {
            self.spi.enable();
            self.spi.start_transfer();
        })
    }

    pub fn is_dma_complete(&self) -> Result<bool, Error> {
        let complete = self.transfer.is_transfer_complete()?
            && self.transfer.is_transfer_complete()?;
        Ok(complete)
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.disable_dma();
    }

    pub fn free(self) -> Result<(CH, D), Error> {
        let (ch, _, d) = self.transfer.free()?;
        Ok((ch, d))
    }
}

pub struct TxDmaTransfer<'a, SPI, W: FrameSize, CH, S> {
    spi: &'a mut Spi<SPI, W>,
    transfer: DmaTransfer<CH, S, DmaTx<SPI, W>, MemoryToPeripheral>,
}

impl<'a, SPI, W, CH, S> TxDmaTransfer<'a, SPI, W, CH, S>
where
    SPI: Instance,
    W: FrameSize + Word,
    CH: Channel,
    S: ReadBuffer<Word = W>,
{
    pub fn new(spi: &'a mut Spi<SPI, W>, channel: CH, source: S) -> Self {
        let config = DmaConfig::new().with_request(SPI::tx_dma_request());
        let destination = DmaTx::new();
        let transfer = DmaTransfer::memory_to_peripheral(
            config,
            channel,
            source,
            destination,
        );
        Self { spi, transfer }
    }

    pub fn start(&mut self) -> Result<(), crate::gpdma::Error> {
        self.transfer.start_with(|_, _| {
            self.spi.enable_tx_dma();
            self.spi.enable();
            self.spi.start_transfer();
        })
    }

    pub fn is_dma_complete(&self) -> Result<bool, Error> {
        let complete = self.transfer.is_transfer_complete()?
            && self.transfer.is_transfer_complete()?;
        Ok(complete)
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.disable_dma();
    }

    pub fn free(self) -> Result<(CH, S), Error> {
        let (ch, s, _) = self.transfer.free()?;
        Ok((ch, s))
    }
}

pub struct DuplexDmaTransfer<'a, SPI, W: FrameSize, TX, RX, S, D> {
    spi: &'a mut Spi<SPI, W>,
    tx_transfer: DmaTransfer<TX, S, DmaTx<SPI, W>, MemoryToPeripheral>,
    rx_transfer: DmaTransfer<RX, DmaRx<SPI, W>, D, PeripheralToMemory>,
    len: usize,
}

impl<'a, SPI, W, RX, TX, S, D> DuplexDmaTransfer<'a, SPI, W, TX, RX, S, D>
where
    SPI: Instance,
    W: FrameSize + Word,
    TX: Channel,
    RX: Channel,
    S: ReadBuffer<Word = W>,
    D: WriteBuffer<Word = W>,
{
    pub fn new(
        spi: &'a mut Spi<SPI, W>,
        tx_channel: TX,
        rx_channel: RX,
        source: S,
        mut destination: D,
    ) -> Self {
        let (_, dest_len) = unsafe { destination.write_buffer() };

        let tx_config = DmaConfig::new().with_request(SPI::tx_dma_request());
        let tx_destination = DmaTx::new();
        let tx_transfer = DmaTransfer::memory_to_peripheral(
            tx_config,
            tx_channel,
            source,
            tx_destination,
        );
        let rx_source = DmaRx::new();
        let rx_config = DmaConfig::new().with_request(SPI::rx_dma_request());
        let rx_transfer = DmaTransfer::peripheral_to_memory(
            rx_config,
            rx_channel,
            rx_source,
            destination,
        );
        Self {
            spi,
            tx_transfer,
            rx_transfer,
            len: dest_len,
        }
    }

    pub fn enable_dma_interrupts(&self) {
        self.rx_transfer.enable_interrupts()
    }

    pub fn disable_dma_interrupts(&self) {
        self.rx_transfer.disable_interrupts()
    }

    pub fn start(&mut self) -> Result<(), Error> {
        self.spi.enable_rx_dma();
        self.rx_transfer.start_nb();
        self.tx_transfer.start_nb();
        self.spi.enable_tx_dma();
        self.spi
            .setup_transaction(NonZeroUsize::new(self.len).unwrap())?;
        Ok(())
    }

    pub fn is_dma_complete(&self) -> Result<bool, Error> {
        let complete = self.tx_transfer.is_transfer_complete()?
            && self.rx_transfer.is_transfer_complete()?;
        Ok(complete)
    }

    pub fn wait_for_complete(&mut self) -> Result<(), Error> {
        self.tx_transfer.wait_for_transfer_complete()?;
        self.rx_transfer.wait_for_transfer_complete()?;
        self.spi.end_transaction();
        self.spi.disable_dma();
        Ok(())
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.disable_dma();
    }

    pub fn abort(&mut self) -> Result<(), Error> {
        self.end_transfer();
        self.tx_transfer.abort()?;
        self.rx_transfer.abort()?;
        Ok(())
    }

    pub fn free(mut self) -> Result<(TX, RX, S, D), Error> {
        self.end_transfer();
        let (tx, s, _) = self.tx_transfer.free()?;
        let (rx, _, d) = self.rx_transfer.free()?;
        Ok((tx, rx, s, d))
    }
}
