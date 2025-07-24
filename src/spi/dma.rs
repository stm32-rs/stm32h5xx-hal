use core::{
    future::Future,
    ops::{Deref, DerefMut},
    pin::Pin,
    task::{Context, Poll},
};

use atomic_waker::AtomicWaker;
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;

use crate::gpdma::{
    config::DmaConfig,
    periph::{DmaDuplex, DmaRx, DmaTx, Rx, RxAddr, Tx, TxAddr},
    ChannelRegs, DmaChannel, DmaTransfer, Error as DmaError, Word as DmaWord,
};

use super::{Error, Instance, Spi, Word};

impl<SPI: Instance, W: DmaWord> TxAddr<W> for SPI {
    unsafe fn tx_addr() -> *mut W {
        (*SPI::ptr()).txdr().as_ptr() as *mut W
    }
}

impl<SPI: Instance, W: DmaWord> RxAddr<W> for SPI {
    unsafe fn rx_addr() -> *const W {
        (*SPI::ptr()).rxdr().as_ptr() as *const W
    }
}

impl<SPI, W> Spi<SPI, W>
where
    SPI: Instance,
    W: Word + DmaWord,
{
    pub fn use_dma_tx<CH>(
        self,
        channel: DmaChannel<CH>,
    ) -> SpiDma<SPI, W, DmaTx<SPI, W, CH>>
    where
        CH: ChannelRegs,
    {
        SpiDma::new_simplex_transmitter(self, channel)
    }

    pub fn use_dma_rx<CH>(
        self,
        channel: DmaChannel<CH>,
    ) -> SpiDma<SPI, W, DmaRx<SPI, W, CH>>
    where
        CH: ChannelRegs,
    {
        SpiDma::new_simplex_receiver(self, channel)
    }

    pub fn use_dma_duplex<TX, RX>(
        self,
        tx_channel: DmaChannel<TX>,
        rx_channel: DmaChannel<RX>,
    ) -> SpiDma<SPI, W, DmaDuplex<SPI, W, TX, RX>>
    where
        TX: ChannelRegs,
        RX: ChannelRegs,
    {
        SpiDma::new_duplex(self, tx_channel, rx_channel)
    }
}

pub struct SpiDma<SPI, W: Word, MODE> {
    spi: Spi<SPI, W>,
    mode: MODE,
}

impl<SPI, W, MODE> SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    pub fn new(spi: Spi<SPI, W>, mode: MODE) -> Self {
        Self { spi, mode }
    }

    pub fn finish_transfer(
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

    async fn finish_transfer_async(
        &mut self,
        result: Result<(), DmaError>,
    ) -> Result<(), Error> {
        let result = match result {
            Ok(_) => {
                SpiDmaFuture::new(self).await
            }
            Err(error) => {
                self.abort_transaction();
                Err(Error::DmaError(error))
            }
        };
        result
    }
}

impl<SPI, W, CH> SpiDma<SPI, W, DmaTx<SPI, W, CH>>
where
    SPI: Instance,
    W: Word,
    CH: ChannelRegs,
{
    pub fn new_simplex_transmitter(
        spi: Spi<SPI, W>,
        channel: DmaChannel<CH>,
    ) -> Self {
        Self {
            spi,
            mode: DmaTx::from(channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, DmaChannel<CH>) {
        let spi = self.spi;
        let channel = self.mode.into();
        (spi, channel)
    }
}

impl<SPI, W, CH> SpiDma<SPI, W, DmaRx<SPI, W, CH>>
where
    SPI: Instance,
    W: Word,
    CH: ChannelRegs,
{
    pub fn new_simplex_receiver(
        spi: Spi<SPI, W>,
        channel: DmaChannel<CH>,
    ) -> Self {
        Self {
            spi,
            mode: DmaRx::from(channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, DmaChannel<CH>) {
        (self.spi, self.mode.into())
    }
}

impl<SPI, W, TX, RX> SpiDma<SPI, W, DmaDuplex<SPI, W, TX, RX>>
where
    SPI: Instance + TxAddr<W> + RxAddr<W>,
    W: Word + DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    pub fn new_duplex(
        spi: Spi<SPI, W>,
        tx_channel: DmaChannel<TX>,
        rx_channel: DmaChannel<RX>,
    ) -> Self {
        Self {
            spi,
            mode: DmaDuplex::new(tx_channel, rx_channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, DmaChannel<TX>, DmaChannel<RX>) {
        let (tx, rx) = self.mode.free();
        (self.spi, tx, rx)
    }
}

impl<SPI, W, MODE> Deref for SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    type Target = Spi<SPI, W>;

    fn deref(&self) -> &Self::Target {
        &self.spi
    }
}

impl<SPI, W, MODE> DerefMut for SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.spi
    }
}

impl<SPI, W, MODE> SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word + DmaWord,
    MODE: Rx<W>,
{
    pub fn start_dma_read<'a>(
        &'a mut self,
        words: &'a mut [W],
    ) -> Result<DmaTransfer<'a, impl ChannelRegs + use<'a, SPI, W, MODE>>, Error>
    {
        let config = DmaConfig::new().with_request(SPI::rx_dma_request());

        self.spi.inner.set_transfer_word_count(words.len() as u16);
        // Make sure to handle any errors before initializing a transfer
        self.setup_read_mode()?;

        let spi = &mut self.spi;
        let transfer = self.mode.init_rx_transfer(config, words);

        spi.inner.enable_rx_dma();

        // Start DMA before starting the transaction to avoid receieve buffer overruns
        transfer.start_nonblocking();
        spi.start_transaction();

        Ok(transfer)
    }

    async fn read_dma(&mut self, words: &mut [W]) -> Result<(), Error> {
        let result = self.start_dma_read(words)?.to_async().await;
        self.finish_transfer_async(result).await
    }
}

impl<SPI, W, MODE> SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word + DmaWord,
    MODE: Tx<W>,
{
    pub fn start_dma_write<'a>(
        &'a mut self,
        words: &'a [W],
    ) -> Result<DmaTransfer<'a, impl ChannelRegs + use<'a, SPI, W, MODE>>, Error>
    {
        let config = DmaConfig::new().with_request(SPI::tx_dma_request());

        self.inner.set_transfer_word_count(words.len() as u16);

        // Make sure to handle any errors before initializing a transfer
        self.setup_write_mode()?;

        let spi = &mut self.spi;
        let transfer = self.mode.init_tx_transfer(config, words);

        transfer.start_nonblocking();
        spi.inner.enable_tx_dma();
        spi.start_transaction();

        Ok(transfer)
    }

    async fn write_dma(&mut self, words: &[W]) -> Result<(), Error> {
        let result = self.start_dma_write(words)?.to_async().await;
        self.finish_transfer_async(result).await
    }
}

impl<SPI, W, MODE> SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word + DmaWord,
    MODE: Tx<W> + Rx<W>,
{
    pub fn start_dma_duplex_transfer<'a>(
        &'a mut self,
        read: &'a mut [W],
        write: &'a [W],
    ) -> Result<
        (
            DmaTransfer<'a, impl ChannelRegs + use<'a, SPI, W, MODE>>,
            DmaTransfer<'a, impl ChannelRegs + use<'a, SPI, W, MODE>>,
        ),
        Error,
    > {
        let tx_config = DmaConfig::new().with_request(SPI::tx_dma_request());
        let rx_config = DmaConfig::new().with_request(SPI::rx_dma_request());

        self.inner.set_transfer_word_count(read.len() as u16);

        self.check_transfer_mode()?;

        let spi = &mut self.spi;
        let tx_transfer = self.mode.init_tx_transfer(tx_config, write);
        let rx_transfer = self.mode.init_rx_transfer(rx_config, read);

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
        let (tx, rx) = (tx.to_async(), rx.to_async());
        let result = tx.await.and(rx.await);

        self.finish_transfer_async(result).await
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

impl<SPI, W, MODE> ErrorType for SpiDma<SPI, W, MODE>
where
    SPI: Instance,
    W: Word,
{
    type Error = Error;
}

impl<SPI, W, CH> SpiBus<W> for SpiDma<SPI, W, DmaTx<SPI, W, CH>>
where
    SPI: Instance,
    W: Word + DmaWord,
    CH: ChannelRegs,
{
    async fn read(&mut self, _words: &mut [W]) -> Result<(), Self::Error> {
        unimplemented!("Not supported for simplex transmitter")
    }

    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.write_dma(words).await
    }

    async fn transfer(
        &mut self,
        _read: &mut [W],
        _write: &[W],
    ) -> Result<(), Self::Error> {
        unimplemented!("Not supported for simplex transmitter")
    }

    async fn transfer_in_place(
        &mut self,
        _words: &mut [W],
    ) -> Result<(), Self::Error> {
        unimplemented!("Not supported for simplex transmitter")
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // This is handled within each of the above functions
        Ok(())
    }
}

impl<SPI, W, CH> SpiBus<W> for SpiDma<SPI, W, DmaRx<SPI, W, CH>>
where
    SPI: Instance,
    W: Word + DmaWord,
    CH: ChannelRegs,
{
    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.read_dma(words).await
    }

    async fn write(&mut self, _words: &[W]) -> Result<(), Self::Error> {
        unimplemented!("Not supported for simplex receiver")
    }

    async fn transfer(
        &mut self,
        _read: &mut [W],
        _write: &[W],
    ) -> Result<(), Self::Error> {
        unimplemented!("Not supported for simplex receiver")
    }

    async fn transfer_in_place(
        &mut self,
        _words: &mut [W],
    ) -> Result<(), Self::Error> {
        unimplemented!("Not supported for simplex receiver")
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // This is handled within each of the above functions
        Ok(())
    }
}

impl<SPI, W, TX, RX> SpiBus<W> for SpiDma<SPI, W, DmaDuplex<SPI, W, TX, RX>>
where
    SPI: Instance,
    W: Word + DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.read_dma(words).await
    }

    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.write_dma(words).await
    }

    async fn transfer(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Self::Error> {
        self.transfer_dma(read, write).await
    }

    async fn transfer_in_place(
        &mut self,
        words: &mut [W],
    ) -> Result<(), Self::Error> {
        self.transfer_inplace_dma(words).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // This is handled within each of the above functions
        Ok(())
    }
}

struct SpiDmaFuture<'a, SPI: Instance, W: Word, MODE> {
    spi: &'a mut SpiDma<SPI, W, MODE>,
    waker: AtomicWaker,
}

impl<'a, SPI: Instance, W: Word, MODE> SpiDmaFuture<'a, SPI, W, MODE> {
    fn new(spi: &'a mut SpiDma<SPI, W, MODE>) -> Self {
        Self {
            spi,
            waker: AtomicWaker::new(),
        }
    }
}

impl<SPI: Instance, W: Word, MODE> Unpin for SpiDmaFuture<'_, SPI, W, MODE> {}

impl<SPI: Instance, W: Word, MODE> Drop for SpiDmaFuture<'_, SPI, W, MODE> {
    fn drop(&mut self) {
        if !self.spi.is_transaction_complete() {
            self.spi.abort_transaction();
        } else if self.spi.inner.is_enabled() {
            self.spi.disable();
        } else {
            // do nothing if the transaction is already complete
        }
    }
}

impl<SPI: Instance, W: Word, MODE> Future for SpiDmaFuture<'_, SPI, W, MODE> {
    type Output = Result<(), Error>;

    fn poll(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<Self::Output> {
        self.waker.register(cx.waker());

        if self.spi.is_transaction_complete() {
            self.spi.disable();
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}
