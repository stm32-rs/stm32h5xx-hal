use core::{
    future::{Future, IntoFuture},
    ops::{Deref, DerefMut},
    pin::Pin,
    task::{Context, Poll},
};

use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_hal::spi::ErrorType;
use futures_util::join;
use futures_util::task::AtomicWaker;

use crate::{
    gpdma::{
        config::DmaConfig,
        periph::{DmaDuplex, DmaRx, DmaTx, PeriphRxBuffer, PeriphTxBuffer, RxAddr, TxAddr},
        DmaChannel, DmaTransfer, Error as DmaError, Word as DmaWord,
    },
    interrupt,
    spi::CommunicationMode,
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
        channel: CH,
    ) -> SpiDma<SPI, DmaTx<SPI, W, CH>, W>
    where
        CH: DmaChannel,
    {
        assert!(self.inner.is_transmitter());
        SpiDma::new_simplex_transmitter(self, channel)
    }

    /// Use DMA for receiving data only in simplex receiver mode, or in half duplex mode as a
    /// receiver
    pub fn use_dma_rx<CH>(
        self,
        channel: CH,
    ) -> SpiDma<SPI, DmaRx<SPI, W, CH>, W>
    where
        CH: DmaChannel,
    {
        // Using DMA for receiving data requires that the SPI is configured as a simplex receiver or
        // in half duplex mode when receiving data only
        // otherwise no data will be received because no clock pulses are generated
        assert!(
            self.inner.communication_mode()
                == CommunicationMode::SimplexReceiver
                || (self.inner.communication_mode()
                    == CommunicationMode::HalfDuplex
                    && !self.inner.is_half_duplex_transmitter())
        );
        SpiDma::new_simplex_receiver(self, channel)
    }

    /// Use DMA for full duplex transfers
    pub fn use_dma_duplex<TX, RX>(
        self,
        tx_channel: TX,
        rx_channel: RX,
    ) -> SpiDma<SPI, DmaDuplex<SPI, W, TX, RX>, W>
    where
        TX: DmaChannel,
        RX: DmaChannel,
    {
        assert!(
            self.inner.communication_mode() == CommunicationMode::FullDuplex
        );
        SpiDma::new_duplex(self, tx_channel, rx_channel)
    }
}

pub struct SpiDma<SPI, MODE, W: Word = u8> {
    spi: Spi<SPI, W>,
    mode: MODE,
}

#[allow(private_bounds)]
impl<SPI, MODE, W> SpiDma<SPI, MODE, W>
where
    SPI: Instance + Waker,
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
            Ok(_) => SpiDmaFuture::new(self).await,
            Err(error) => {
                self.abort_transaction();
                Err(Error::DmaError(error))
            }
        };
        result
    }
}

impl<SPI, CH, W> SpiDma<SPI, DmaTx<SPI, W, CH>, W>
where
    SPI: Instance,
    W: Word,
    CH: DmaChannel,
{
    pub fn new_simplex_transmitter(spi: Spi<SPI, W>, channel: CH) -> Self {
        Self {
            spi,
            mode: DmaTx::from(channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, CH) {
        let spi = self.spi;
        let channel = self.mode.free();
        (spi, channel)
    }
}

impl<SPI, CH, W> SpiDma<SPI, DmaRx<SPI, W, CH>, W>
where
    SPI: Instance,
    W: Word,
    CH: DmaChannel,
{
    pub fn new_simplex_receiver(spi: Spi<SPI, W>, channel: CH) -> Self {
        Self {
            spi,
            mode: DmaRx::from(channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, CH) {
        (self.spi, self.mode.free())
    }
}

impl<SPI, TX, RX, W> SpiDma<SPI, DmaDuplex<SPI, W, TX, RX>, W>
where
    SPI: Instance + TxAddr<W> + RxAddr<W>,
    W: Word + DmaWord,
    TX: DmaChannel,
    RX: DmaChannel,
{
    pub fn new_duplex(
        spi: Spi<SPI, W>,
        tx_channel: TX,
        rx_channel: RX,
    ) -> Self {
        Self {
            spi,
            mode: DmaDuplex::new(tx_channel, rx_channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, TX, RX) {
        let (tx, rx) = self.mode.free();
        (self.spi, tx, rx)
    }
}

impl<SPI, MODE, W> Deref for SpiDma<SPI, MODE, W>
where
    SPI: Instance,
    W: Word,
{
    type Target = Spi<SPI, W>;

    fn deref(&self) -> &Self::Target {
        &self.spi
    }
}

impl<SPI, MODE, W> DerefMut for SpiDma<SPI, MODE, W>
where
    SPI: Instance,
    W: Word,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.spi
    }
}

#[allow(private_bounds)]
impl<SPI, CH, W> SpiDma<SPI, DmaRx<SPI, W, CH>, W>
where
    SPI: Instance + Waker,
    W: Word + DmaWord,
    CH: DmaChannel,
{
    pub fn start_dma_read<'a, D>(
        &'a mut self,
        mut destination: D,
    ) -> Result<DmaTransfer<'a, CH, PeriphRxBuffer<SPI, W>, D>, Error>
    where
        D: WriteBuffer<Word = W>,
    {
        let config = DmaConfig::new().with_request(SPI::rx_dma_request());
        let (_, len) = unsafe { destination.write_buffer() };

        self.spi.inner.set_transfer_word_count(len as u16);
        // Make sure to handle any errors before initializing a transfer
        self.setup_read_mode()?;

        let spi = &mut self.spi;
        let mut transfer = self.mode.init_rx_transfer(config, destination);

        spi.inner.enable_rx_dma();

        // Start DMA before starting the transaction to avoid receieve buffer overruns
        transfer.start_nonblocking();
        spi.start_transaction();

        Ok(transfer)
    }

    pub async fn read_dma<D>(&mut self, destination: D) -> Result<(), Error>
    where
        D: WriteBuffer<Word = W>,
    {
        let result = self.start_dma_read(destination)?.await;
        self.finish_transfer_async(result).await
    }
}

#[allow(private_bounds)]
impl<SPI, CH, W> SpiDma<SPI, DmaTx<SPI, W, CH>, W>
where
    SPI: Instance + Waker,
    W: Word + DmaWord,
    CH: DmaChannel,
{
    pub fn start_dma_write<'a, S>(
        &'a mut self,
        source: S,
    ) -> Result<DmaTransfer<'a, CH, S, PeriphTxBuffer<SPI, W>>, Error>
    where
        S: ReadBuffer<Word = W>,
    {
        let config = DmaConfig::new().with_request(SPI::tx_dma_request());

        let (_, len) = unsafe { source.read_buffer() };

        self.inner.set_transfer_word_count(len as u16);

        // Make sure to handle any errors before initializing a transfer
        self.setup_write_mode()?;

        let spi = &mut self.spi;
        let mut transfer = self.mode.init_tx_transfer(config, source);

        transfer.start_nonblocking();
        spi.inner.enable_tx_dma();
        spi.start_transaction();

        Ok(transfer)
    }

    pub async fn write_dma<S>(&mut self, source: S) -> Result<(), Error>
    where
        S: ReadBuffer<Word = W>,
    {
        let result = self.start_dma_write(source)?.await;
        self.finish_transfer_async(result).await
    }
}

#[allow(private_bounds)]
impl<SPI, TX, RX, W> SpiDma<SPI, DmaDuplex<SPI, W, TX, RX>, W>
where
    SPI: Instance + Waker,
    W: Word + DmaWord,
    TX: DmaChannel,
    RX: DmaChannel,
{
    #[allow(clippy::type_complexity)]
    pub fn start_dma_duplex_transfer<'a, S, D>(
        &'a mut self,
        source: S,
        mut destination: D,
    ) -> Result<(DmaTransfer<'a, TX, S, PeriphTxBuffer<SPI, W>>,
        DmaTransfer<'a, RX, PeriphRxBuffer<SPI, W>, D>), Error>
    where
        S: ReadBuffer<Word = W>,
        D: WriteBuffer<Word = W>,
    {
        let (_, read_len) = unsafe { source.read_buffer() };
        let (_, write_len) = unsafe { destination.write_buffer() };

        assert_eq!(
            read_len, write_len,
            "Read and write buffers must have the same length"
        );

        let tx_config = DmaConfig::new().with_request(SPI::tx_dma_request());
        let rx_config = DmaConfig::new().with_request(SPI::rx_dma_request());

        self.inner.set_transfer_word_count(read_len as u16);

        self.check_transfer_mode()?;

        let spi = &mut self.spi;
        let (mut tx_transfer, mut rx_transfer) = self
            .mode
            .init_duplex_transfer(tx_config, rx_config, source, destination);

        spi.inner.enable_rx_dma();
        rx_transfer.start_nonblocking();
        tx_transfer.start_nonblocking();
        spi.inner.enable_tx_dma();
        spi.start_transaction();

        Ok((tx_transfer, rx_transfer))
    }

    pub async fn transfer_dma<S, D>(
        &mut self,
        source: S,
        destination: D,
    ) -> Result<(), Error>
    where
        S: ReadBuffer<Word = W>,
        D: WriteBuffer<Word = W>,
    {
        let (tx, rx) = self.start_dma_duplex_transfer(source, destination)?;
        let (tx, rx) = (tx.into_future(), rx.into_future());
        let results = join!(tx, rx);

        let result = results.0.and(results.1);

        self.finish_transfer_async(result).await
    }

    pub async fn transfer_inplace_dma<B>(
        &mut self,
        mut buffer: B,
    ) -> Result<(), Error>
    where
        B: WriteBuffer<Word = W>,
    {
        let (ptr, len) = unsafe { buffer.write_buffer() };

        // Note (unsafe): Data will be read from the start of the buffer before data is written
        // to those locations just like for blocking non-DMA in-place transfers, and the location
        // is already guaranteed to be 'static
        let source = unsafe { core::slice::from_raw_parts(ptr, len) };

        self.transfer_dma(source, buffer).await
    }
}

impl<SPI, MODE, W> ErrorType for SpiDma<SPI, MODE, W>
where
    SPI: Instance,
    W: Word,
{
    type Error = Error;
}

struct SpiDmaFuture<'a, SPI: Instance, MODE, W: Word> {
    spi: &'a mut SpiDma<SPI, MODE, W>,
}

impl<'a, SPI: Instance, MODE, W: Word> SpiDmaFuture<'a, SPI, MODE, W> {
    fn new(spi: &'a mut SpiDma<SPI, MODE, W>) -> Self {
        spi.inner.enable_dma_transfer_interrupts();
        Self { spi }
    }
}

impl<SPI: Instance, MODE, W: Word> Unpin for SpiDmaFuture<'_, SPI, MODE, W> {}

impl<SPI: Instance, MODE, W: Word> Drop for SpiDmaFuture<'_, SPI, MODE, W> {
    fn drop(&mut self) {
        self.spi.disable();
    }
}

impl<SPI: Instance + Waker, MODE, W: Word> Future
    for SpiDmaFuture<'_, SPI, MODE, W>
{
    type Output = Result<(), Error>;

    fn poll(
        mut self: Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<Self::Output> {
        SPI::waker().register(cx.waker());

        if self.spi.is_transaction_complete() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

trait Waker {
    fn waker() -> &'static AtomicWaker;
}

macro_rules! spi_dma_irq {
    ($SPI:ident) => {
        paste::item! {
            static [<$SPI _WAKER>]: AtomicWaker = AtomicWaker::new();

            impl Waker for $SPI {
                #[inline(always)]
                fn waker() -> &'static AtomicWaker {
                    &[<$SPI _WAKER>]
                }
            }

            #[interrupt]
            fn $SPI() {
                let spi = unsafe { &*$SPI::ptr() };
                unsafe { spi.ier().write_with_zero(|w| w); };
                $SPI::waker().wake();
            }
        }
    };
}
use crate::pac::{SPI1, SPI2, SPI3};

spi_dma_irq!(SPI1);
spi_dma_irq!(SPI2);
spi_dma_irq!(SPI3);

#[cfg(feature = "rm0481")]
mod rm0481 {
    use super::*;
    use crate::pac::SPI4;
    spi_dma_irq!(SPI4);
}

#[cfg(feature = "h56x_h573")]
mod h56x_h573 {
    use super::*;
    use crate::pac::{SPI5, SPI6};
    spi_dma_irq!(SPI5);
    spi_dma_irq!(SPI6);
}
