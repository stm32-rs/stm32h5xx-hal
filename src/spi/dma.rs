use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use embedded_dma::{ReadBuffer, WriteBuffer};
use embedded_hal::spi::ErrorType;
use embedded_hal_async::spi::SpiBus;

use crate::gpdma::{
    config::{DmaConfig, MemoryToPeripheral, PeripheralToMemory, TransferType},
    ChannelRegs, DmaChannel, DmaTransfer, Error as DmaError, Word as DmaWord,
};

use super::{Error, Instance, Spi, Word};

pub trait TxAddr<W: DmaWord> {
    /// Returns a pointer to the peripheral's transmit data register.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the returned pointer is only used when it is valid to access
    /// the peripheral's transmit data register, and that no data races or invalid memory accesses
    /// occur.
    unsafe fn tx_addr() -> *mut W;
}

pub trait RxAddr<W: DmaWord> {
    /// Returns a pointer to the peripheral's receive data register.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the returned pointer is only used when it is valid to access
    /// the peripheral's receive data register, and that no data races or invalid memory accesses
    /// occur.
    unsafe fn rx_addr() -> *const W;
}

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

trait Tx<W> {
    fn init_tx_transfer<'a>(
        &'a self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        words: &'a [W],
    ) -> DmaTransfer<'a, impl ChannelRegs>;
}

trait Rx<W> {
    fn init_rx_transfer<'a>(
        &'a self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        words: &'a mut [W],
    ) -> DmaTransfer<'a, impl ChannelRegs>;
}

pub struct DmaRx<PERIPH, W, CH> {
    _periph: PhantomData<PERIPH>,
    _word: PhantomData<W>,
    channel: DmaChannel<CH>,
}

impl<PERIPH, W, CH: ChannelRegs> DmaRx<PERIPH, W, CH> {
    fn new(channel: DmaChannel<CH>) -> Self {
        Self {
            _periph: PhantomData,
            _word: PhantomData,
            channel,
        }
    }
}

unsafe impl<PERIPH: RxAddr<W>, W: DmaWord, CH> ReadBuffer for &DmaRx<PERIPH, W, CH> {
    type Word = W;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (PERIPH::rx_addr(), 1)
    }
}

impl<PERIPH, W, CH> Rx<W> for DmaRx<PERIPH, W, CH>
where
    PERIPH: RxAddr<W>,
    CH: ChannelRegs,
    W: DmaWord,
{
    fn init_rx_transfer<'a>(
        &'a self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        words: &'a mut [W],
    ) -> DmaTransfer<'a, impl ChannelRegs> {
        DmaTransfer::peripheral_to_memory(config, &self.channel, self, words)
    }
}

pub struct DmaTx<PERIPH, W, CH> {
    _periph: PhantomData<PERIPH>,
    _word: PhantomData<W>,
    channel: DmaChannel<CH>,
}

impl<PERIPH, W, CH: ChannelRegs> DmaTx<PERIPH, W, CH> {
    fn new(channel: DmaChannel<CH>) -> Self {
        Self {
            _periph: PhantomData,
            _word: PhantomData,
            channel,
        }
    }
}

unsafe impl<PERIPH: TxAddr<W>, W: DmaWord, CH> WriteBuffer for &DmaTx<PERIPH, W, CH> {
    type Word = W;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (PERIPH::tx_addr(), 1)
    }
}

impl<PERIPH, W, CH> Tx<W> for DmaTx<PERIPH, W, CH>
where
    PERIPH: TxAddr<W>,
    CH: ChannelRegs,
    W: DmaWord,
{
    fn init_tx_transfer<'a>(
        &'a self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        words: &'a [W],
    ) -> DmaTransfer<'a, impl ChannelRegs> {
        DmaTransfer::memory_to_peripheral(config, &self.channel, words, self)
    }
}

pub struct DmaDuplex<PERIPH, W, TX, RX> {
    tx: DmaTx<PERIPH, W, TX>,
    rx: DmaRx<PERIPH, W, RX>,
}

impl<PERIPH, W, TX, RX> DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    fn new(tx: DmaChannel<TX>, rx: DmaChannel<RX>) -> Self {
        Self {
            tx: DmaTx::new(tx),
            rx: DmaRx::new(rx),
        }
    }
}

impl<PERIPH, W, TX, RX> Tx<W> for DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    fn init_tx_transfer<'a>(
        &'a self,
        config: DmaConfig<MemoryToPeripheral, W, W>,
        words: &'a [W],
    ) -> DmaTransfer<'a, impl ChannelRegs> {
        self.tx.init_tx_transfer(config, words)
    }
}

impl<PERIPH, W, TX, RX> Rx<W> for DmaDuplex<PERIPH, W, TX, RX>
where
    PERIPH: TxAddr<W> + RxAddr<W>,
    W: Word + DmaWord,
    TX: ChannelRegs,
    RX: ChannelRegs,
{
    fn init_rx_transfer<'a>(
        &'a self,
        config: DmaConfig<PeripheralToMemory, W, W>,
        words: &'a mut [W],
    ) -> DmaTransfer<'a, impl ChannelRegs> {
        self.rx.init_rx_transfer(config, words)
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
            mode: DmaTx::new(channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, DmaChannel<CH>) {
        (self.spi, self.mode.channel)
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
            mode: DmaRx::new(channel),
        }
    }

    pub fn free(self) -> (Spi<SPI, W>, DmaChannel<CH>) {
        (self.spi, self.mode.channel)
    }
}

impl<SPI, W, TX, RX> SpiDma<SPI, W, DmaDuplex<SPI, W, TX, RX>>
where
    SPI: Instance,
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
        (self.spi, self.mode.tx.channel, self.mode.rx.channel)
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
        self.finish_transfer(result)
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
        self.finish_transfer(result)
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
