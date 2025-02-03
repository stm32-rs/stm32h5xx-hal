use core::marker::PhantomData;

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

pub struct RxDmaTransfer<SPI, W: FrameSize, CH, D> {
    spi: Spi<SPI, W>,
    transfer: DmaTransfer<CH, DmaRx<SPI, W>, D, PeripheralToMemory>,
}

impl<SPI, W, CH, D> RxDmaTransfer<SPI, W, CH, D>
where
    SPI: Instance,
    W: FrameSize + Word,
    CH: Channel,
    D: WriteBuffer<Word = W>,
{
    pub fn new(spi: Spi<SPI, W>, channel: CH, destination: D) -> Self {
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
}

pub struct TxDmaTransfer<SPI, W: FrameSize, CH, S> {
    spi: Spi<SPI, W>,
    transfer: DmaTransfer<CH, S, DmaTx<SPI, W>, MemoryToPeripheral>,
}

impl<SPI, W, CH, S> TxDmaTransfer<SPI, W, CH, S>
where
    SPI: Instance,
    W: FrameSize + Word,
    CH: Channel,
    S: ReadBuffer<Word = W>,
{
    pub fn new(spi: Spi<SPI, W>, channel: CH, source: S) -> Self {
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
}

pub struct DuplexDmaTransfer<'a, SPI, W: FrameSize, TX, RX, S, D> {
    spi: &'a mut Spi<SPI, W>,
    tx_transfer: DmaTransfer<TX, S, DmaTx<SPI, W>, MemoryToPeripheral>,
    rx_transfer: DmaTransfer<RX, DmaRx<SPI, W>, D, PeripheralToMemory>,
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
        destination: D,
    ) -> Self {
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
        }
    }

    pub fn start(&mut self) -> Result<(), Error> {
        self.spi.enable_rx_dma();
        self.rx_transfer.start()?;
        self.tx_transfer.start_with(|_, _| {
            self.spi.enable_tx_dma();
            self.spi.enable();
            self.spi.start_transfer();
        })?;
        Ok(())
    }

    pub fn is_dma_complete(&self) -> Result<bool, Error> {
        let complete = self.tx_transfer.is_transfer_complete()?
            && self.rx_transfer.is_transfer_complete()?;
        Ok(complete)
    }

    pub fn wait_for_complete(&mut self) -> Result<(), Error> {
        while !self.is_dma_complete()? {}
        self.spi.end_transaction();
        self.spi.disable_dma();
        Ok(())
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.disable_dma();
    }

    pub fn free(self) -> Result<(TX, RX, S, D), Error> {
        let (tx, s, _) = self.tx_transfer.free()?;
        let (rx, _, d) = self.rx_transfer.free()?;
        Ok((tx, rx, s, d))
    }
}

type DuplexInplaceDmaTransfer<'a, SPI, W, TX, RX> =
    DuplexDmaTransfer<'a, SPI, W, TX, RX, &'static [W], &'static mut [W]>;

impl<SPI: Instance, W: FrameSize + Word> Spi<SPI, W> {
    pub fn dma_transfer_inplace<TX: Channel, RX: Channel>(
        &mut self,
        buffer: &'static mut [W],
        tx_channel: TX,
        rx_channel: RX,
    ) -> Result<DuplexInplaceDmaTransfer<SPI, W, TX, RX>, Error> {
        // Note (unsafe): Data will be read from the start of the buffer before data is written
        // to those locations just like for blocking non-DMA in-place transfers
        let source = unsafe { *(buffer.as_ptr() as *const &[W]) };
        let mut transfer = DuplexDmaTransfer::new(
            self, tx_channel, rx_channel, source, buffer,
        );
        transfer.start()?;
        Ok(transfer)
    }
}
