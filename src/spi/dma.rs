use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::gpdma::{
    config::DmaConfig,
    DmaChannel, ChannelRegs, DmaTransfer, Error as DmaError, Word as DmaWord,
};

use super::{Error, Instance, Spi, Word};

struct DmaRx<SPI, W, CH> {
    _spi: PhantomData<SPI>,
    _word: PhantomData<W>,
    channel: DmaChannel<CH>,
}

impl<SPI, W, CH: ChannelRegs> DmaRx<SPI, W, CH> {
    fn new(channel: DmaChannel<CH>) -> Self {
        Self {
            _spi: PhantomData,
            _word: PhantomData,
            channel,
        }
    }
}

unsafe impl<SPI: Instance, W: Word, CH> ReadBuffer for &DmaRx<SPI, W, CH> {
    type Word = W;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        ((*SPI::ptr()).rxdr().as_ptr() as *const W, 1)
    }
}

struct DmaTx<SPI, W, CH> {
    _spi: PhantomData<SPI>,
    _word: PhantomData<W>,
    channel: DmaChannel<CH>,
}

impl<SPI, W, CH: ChannelRegs> DmaTx<SPI, W, CH> {
    fn new(channel: DmaChannel<CH>) -> Self {
        Self {
            _spi: PhantomData,
            _word: PhantomData,
            channel,
        }
    }
}

unsafe impl<SPI: Instance, W: Word, CH> WriteBuffer for &DmaTx<SPI, W, CH> {
    type Word = W;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        ((*SPI::ptr()).txdr().as_ptr() as *mut W, 1)
    }
}

struct DmaDuplex<SPI, W, TX, RX> {
    tx: DmaTx<SPI, W, TX>,
    rx: DmaRx<SPI, W, RX>,
}

pub struct SpiAsync<SPI, W: Word, MODE> {
    spi: Spi<SPI, W>,
    mode: MODE,
}

impl<SPI, W, MODE> SpiAsync<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    pub fn new(spi: Spi<SPI, W>, mode: MODE) -> Self {
        Self { spi, mode }
    }

    fn finish_transfer(
        &mut self,
        result: Result<(), DmaError>,
    ) -> Result<(), Error> {
        let result = match result {
            Ok(_) => {
                self.end_transaction();
                Ok(())
            }
            Err(error) => {
                self.abort_transaction();
                Err(Error::DmaError(error))
            }
        };
        self.inner.disable_dma();
        result
    }
}

impl<SPI, W, MODE> Deref for SpiAsync<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    type Target = Spi<SPI, W>;

    fn deref(&self) -> &Self::Target {
        &self.spi
    }
}

impl<SPI, W, MODE> DerefMut for SpiAsync<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.spi
    }
}

impl<SPI, W, CH> SpiAsync<SPI, W, DmaRx<SPI, W, CH>>
where
    SPI: Instance,
    W: Word + DmaWord,
    CH: ChannelRegs,
{
    fn start_dma_read<'a>(
        &'a mut self,
        words: &'a mut [W],
    ) -> Result<DmaTransfer<'a, CH>, Error>
    where
        CH: ChannelRegs,
    {
        let config = DmaConfig::new().with_request(SPI::rx_dma_request());

        self.spi.inner.set_transfer_word_count(words.len() as u16);
        // Make sure to handle any errors before initializing a transfer
        self.setup_read_mode()?;

        let spi = &mut self.spi;
        let transfer = DmaTransfer::peripheral_to_memory(
            config,
            &self.mode.channel,
            &self.mode,
            words,
        );

        spi.inner.enable_rx_dma();

        // Start DMA before starting the transaction to avoid receieve buffer overruns
        transfer.start_nonblocking();
        spi.start_transaction();

        Ok(transfer)
    }

    async fn read_dma(&mut self, words: &mut [W]) -> Result<(), Error> {
        let result = self.start_dma_read(words)?.await;
        self.finish_transfer(result)
    }
}

impl<SPI, W, CH> SpiAsync<SPI, W, DmaTx<SPI, W, CH>>
where
    SPI: Instance,
    W: Word + DmaWord,
    CH: ChannelRegs,
{
    fn start_dma_write<'a>(
        &'a mut self,
        words: &'a [W],
    ) -> Result<DmaTransfer<'a, CH>, Error>
    where
        CH: ChannelRegs,
    {
        let config = DmaConfig::new().with_request(SPI::tx_dma_request());

        self.inner.set_transfer_word_count(words.len() as u16);

        // Make sure to handle any errors before initializing a transfer
        self.setup_write_mode()?;

        let spi = &mut self.spi;
        let transfer = DmaTransfer::memory_to_peripheral(
            config,
            &self.mode.channel,
            words,
            &self.mode,
        );

        transfer.start_nonblocking();
        spi.inner.enable_tx_dma();
        spi.start_transaction();

        Ok(transfer)
    }

    async fn write_dma(&mut self, words: &[W]) -> Result<(), Error> {
        let result = self.start_dma_write(words)?.await;
        self.finish_transfer(result)
    }
}

impl<SPI, W, TX, RX> SpiAsync<SPI, W, DmaDuplex<SPI, W, TX, RX>>
where
    SPI: Instance,
    W: Word + DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    fn start_dma_duplex_transfer<'a>(
        &'a mut self,
        read: &'a mut [W],
        write: &'a [W],
    ) -> Result<(DmaTransfer<'a, TX>, DmaTransfer<'a, RX>), Error> {
        let tx_config =
            DmaConfig::new().with_request(SPI::tx_dma_request());

        let rx_config =
            DmaConfig::new().with_request(SPI::rx_dma_request());

        self.inner.set_transfer_word_count(read.len() as u16);

        self.check_transfer_mode()?;

        let spi = &mut self.spi;
        let tx_transfer = DmaTransfer::memory_to_peripheral(
            tx_config,
            &self.mode.tx.channel,
            write,
            &self.mode.tx,
        );

        let rx_transfer = DmaTransfer::peripheral_to_memory(
            rx_config,
            &self.mode.rx.channel,
            &self.mode.rx,
            read,
        );

        spi.inner.enable_rx_dma();
        rx_transfer.start_nonblocking();
        tx_transfer.start_nonblocking();
        spi.inner.enable_tx_dma();
        spi.start_transaction();

        Ok((tx_transfer, rx_transfer))
    }

    async fn transfer_dma(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Error> {
        let (tx, rx) = self.start_dma_duplex_transfer(read, write)?;

        let result = tx.await.and(rx.await);

        self.finish_transfer(result)
    }

    async fn transfer_inplace_dma(
        &mut self,
        words: &mut [W],
    ) -> Result<(), Error> {
        // Note (unsafe): Data will be read from the start of the buffer before data is written
        // to those locations just like for blocking non-DMA in-place transfers
        let write: &[W] = unsafe { *(words.as_ptr() as *const &[W]) };
        self.transfer_dma(words, write).await
    }
}
