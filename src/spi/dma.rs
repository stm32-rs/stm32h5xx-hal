use core::marker::PhantomData;

use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::gpdma::{
    config::{DmaConfig, MemoryToPeripheral, PeripheralToMemory},
    Channel, ChannelRegs, DmaTransfer, DmaTransferBuilder, Word as DmaWord,
};

use super::{Error, Instance, Spi, Word};

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

unsafe impl<SPI: Instance, W: Word> ReadBuffer for DmaRx<SPI, W> {
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

unsafe impl<SPI: Instance, W: Word> WriteBuffer for DmaTx<SPI, W> {
    type Word = W;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        ((*SPI::ptr()).txdr().as_ptr() as *mut W, 1)
    }
}

pub struct RxDmaTransfer<'a, SPI, W: Word, CH, D> {
    spi: &'a mut Spi<SPI, W>,
    transfer: DmaTransfer<CH, DmaRx<SPI, W>, D, PeripheralToMemory>,
}

impl<'a, SPI, W, CH, D> RxDmaTransfer<'a, SPI, W, CH, D>
where
    SPI: Instance,
    W: Word + DmaWord,
    CH: ChannelRegs,
    D: WriteBuffer<Word = W>,
{
    pub fn new(
        spi: &'a mut Spi<SPI, W>,
        channel: Channel<CH>,
        mut destination: D,
    ) -> Self {
        let (_, len) = unsafe { destination.write_buffer() };
        let config = DmaConfig::new().with_request(SPI::rx_dma_request());
        let source = DmaRx::new();
        let transfer = DmaTransferBuilder::peripheral_to_memory(
            config,
            channel,
            source,
            destination,
        );
        spi.inner.set_transfer_word_count(len as u16);
        Self { spi, transfer }
    }

    pub fn start(&mut self) -> Result<(), Error> {
        self.spi.setup_read_mode()?;
        self.spi.inner.enable_rx_dma();
        self.transfer.start_with(|_, _| {
            self.spi.start_transaction();
        })?;
        Ok(())
    }

    pub fn is_dma_complete(&self) -> Result<bool, Error> {
        let complete = self.transfer.is_transfer_complete()?
            && self.transfer.is_transfer_complete()?;
        Ok(complete)
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.inner.disable_dma();
    }

    pub fn free(self) -> Result<(Channel<CH>, D), Error> {
        let (ch, _, d) = self.transfer.free()?;
        Ok((ch, d))
    }
}

pub struct TxDmaTransfer<'a, SPI, W: Word, CH: ChannelRegs> {
    spi: &'a mut Spi<SPI, W>,
    transfer: DmaTransfer<'a, CH>,
}

impl<'a, SPI, W, CH> TxDmaTransfer<'a, SPI, W, CH>
where
    SPI: Instance,
    W: DmaWord + Word,
    CH: ChannelRegs,
{
    pub fn new<S: ReadBuffer<Word = W>>(
        spi: &'a mut Spi<SPI, W>,
        channel: &'a Channel<CH>,
        source: S,
    ) -> Self {
        let (_, len) = unsafe { source.read_buffer() };
        let config = DmaConfig::new().with_request(SPI::tx_dma_request());
        let destination = DmaTx::new();
        let transfer = DmaTransferBuilder::memory_to_peripheral(
            config,
            channel,
            source,
            destination,
        );
        spi.inner.set_transfer_word_count(len as u16);
        Self { spi, transfer }
    }

    pub fn start(&mut self) -> Result<(), Error> {
        self.spi.setup_write_mode()?;
        self.transfer.start_with(|_, _| {
            self.spi.inner.enable_tx_dma();
            self.spi.start_transaction();
        })?;
        Ok(())
    }

    pub fn is_dma_complete(&self) -> Result<bool, Error> {
        let complete = self.transfer.is_transfer_complete()?
            && self.transfer.is_transfer_complete()?;
        Ok(complete)
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.inner.disable_dma();
    }

    pub fn free(self) -> Result<(Channel<CH>, S), Error> {
        let (ch, s, _) = self.transfer.free()?;
        Ok((ch, s))
    }
}

pub struct DuplexDmaTransfer<'a, SPI, W: Word, TX: ChannelRegs, RX: ChannelRegs>
{
    spi: &'a mut Spi<SPI, W>,
    tx_transfer: DmaTransfer<'a, TX>,
    rx_transfer: DmaTransfer<'a, RX>,
}

impl<'a, SPI, W, RX, TX> DuplexDmaTransfer<'a, SPI, W, TX, RX>
where
    SPI: Instance,
    W: Word + DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    pub fn new<S, D>(
        spi: &'a mut Spi<SPI, W>,
        tx_channel: &Channel<TX>,
        rx_channel: &Channel<RX>,
        source: S,
        mut destination: D,
    ) -> Self
    where
        S: ReadBuffer<Word = W>,
        D: WriteBuffer<Word = W>,
    {
        let (_, dest_len) = unsafe { destination.write_buffer() };

        let tx_config = DmaConfig::new().with_request(SPI::tx_dma_request());
        let tx_destination = DmaTx::new();
        let tx_transfer = DmaTransferBuilder::memory_to_peripheral(
            tx_config,
            tx_channel,
            source,
            tx_destination,
        );
        let rx_source = DmaRx::new();
        let rx_config = DmaConfig::new().with_request(SPI::rx_dma_request());
        let rx_transfer = DmaTransferBuilder::peripheral_to_memory(
            rx_config,
            rx_channel,
            rx_source,
            destination,
        );
        spi.inner.set_transfer_word_count(dest_len as u16);
        Self {
            spi,
            tx_transfer,
            rx_transfer,
        }
    }

    pub fn enable_dma_interrupts(&self) {
        self.rx_transfer.enable_interrupts()
    }

    pub fn disable_dma_interrupts(&self) {
        self.rx_transfer.disable_interrupts()
    }

    pub fn start(&mut self) -> Result<(), Error> {
        self.spi.check_transfer_mode()?;
        self.spi.inner.enable_rx_dma();
        self.rx_transfer.start_nonblocking();
        self.tx_transfer.start_nonblocking();
        self.spi.inner.enable_tx_dma();
        self.spi.start_transaction();
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
        self.spi.inner.disable_dma();
        Ok(())
    }

    pub fn end_transfer(&mut self) {
        self.spi.end_transaction();
        self.spi.inner.disable_dma();
    }

    pub fn abort(&mut self) -> Result<(), Error> {
        self.end_transfer();
        self.tx_transfer.abort()?;
        self.rx_transfer.abort()?;
        Ok(())
    }

    pub fn free(mut self) -> Result<(Channel<TX>, Channel<RX>, S, D), Error> {
        self.end_transfer();
        let (tx, s, _) = self.tx_transfer.free()?;
        let (rx, _, d) = self.rx_transfer.free()?;
        Ok((tx, rx, s, d))
    }
}
