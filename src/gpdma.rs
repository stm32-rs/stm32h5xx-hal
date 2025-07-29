//! The GPDMA is the general purpose DMA engine in use on the STM32H5 family of processors. It is
//! used to perform programmable data transfers that are offloaded from the CPU to the DMA engine.
//!
//! The GPDMA can perform the following transfers from a *source* address to a *destination*
//! address:
//! - Memory to memory
//! - Memory to peripheral
//! - Peripheral to memory
//! - Peripheral to peripheral
//!
//! Each GPDMA has 8 channels. Each channel can service any hardware request (or memory to memory
//! transfer) that is supported by the processor (ie. they're not tied to specific channels). All
//! channels support direct and linked-buffer transfers. However, the channels do have different
//! capabilities (see RM0492 Rev 3 section 15.3.2 for full details), notably that channels 0-5 can
//! only service transfers in a linear address space, while channels 6 & 7 can also service transfers
//! using a 2D addressing scheme. Both GPDMA peripherals support the same requests/channel
//! capabilities.
//!
//! # Usage
//! At the most basic level transfers take a *source* address and a *destination* address and
//! transfers the data from the source to the destination. The [embedded-dma] traits `ReadBuffer`
//! and `WriteBuffer` represent a source and destination, respectively.
//!
//! ## Memory to memory transfers
//! As long as the buffers satisfy the constraints of embedded-dma's `ReadBuffer` and `WriteBuffer`
//! traits, they can be used directly with the Transfer API:
//! ```
//! use stm32h5xx_hal::{pac, gpdma::{DmaConfig, DmaTransfer};
//!
//! let source_buf = ... // source buffer
//! let dest_buf = ... // destination buffer
//!
//! let dp = pac::Peripherals::take().unwrap();
//! let channels = dp.GPDMA1.channels(ccdr.peripheral.GPDMA1);
//! let channel = channels.0
//! let config = DmaConfig::default();
//! let mut transfer = DmaTransfer::memory_to_memory(config, channel, source_buf, dest_buf);
//! transfer.start().unwrap();
//! transfer.wait_for_transfer_complete().unwrap();
//! ```
//!
//! ## Memory to peripheral transfers
//!
//! The peripheral must provide a `WriteBuffer` implementation for its data register to which the
//! DMA will write. Then it can be used similarly to the memory to memory transfer. The `Transfer`
//! API does provide for performing an operation immediately after enabling the DMA channel, via the
//! Transfer::start_with method, which allows for a closure to be provided. Additionally, a
//! hardware request line must be specified to the Config in order to connect the peripheral to the
//! DMA channel. Another additional option for these transfers is to perform block requests or burst
//! requests.
//!
//! ## Peripheral to memory transfers
//!
//! The peripheral must provide a `ReadBuffer` implementation for its data register from which the
//! DMA will read. Otherwise it is used similarly to Peripheral to memory transfers, including the
//! additional configuration requirements/options specified above. In addition, peripheral flow
//! control mode can be used to enable the peripheral to early terminate a transaction. Per RM0492
//! Rev 3 section 15.3.6, this is only used by the I3C peripheral, and only on channels 0 and 7.
//!
//! ## Peripheral to peripheral transfers
//!
//! These work similarly to the peripheral to memory transfers, but the peripheral driving the
//! request must be identified via the typing of the TransferType implementation.
//!
//! ## Data transforms
//!
//! The GPDMA provides a data transformation pipeline which facilitates transforms for transfers
//! between peripherals or memory that have different source and destination data widths or byte
//! representations (e.g. little endian vs big endian) with zero CPU overhead. See
//! `config::DataTransformBuilder` for more information on it.
//!
//! # Channel/transfer arbitration
//!
//! Every transfer is assigned a priority and a AHB port assignments for each of it its source and
//! destination. The transfer priority is used by the GPDMA controller to arbitrate between requests
//! that are both ready to transfer data via one of the AHB ports.

use crate::{
    pac::{gpdma1, GPDMA1, GPDMA2},
    rcc::{rec, ResetEnable},
    Sealed,
};
use core::{
    marker::PhantomData,
    mem,
    ops::Deref,
    sync::atomic::{fence, Ordering},
};
use embedded_dma::{ReadBuffer, Word as DmaWord, WriteBuffer};

mod ch;
pub mod config;
mod future;
pub mod periph;

pub use ch::{
    DmaChannel, DmaChannel0, DmaChannel1, DmaChannel2, DmaChannel3,
    DmaChannel4, DmaChannel5, DmaChannel6, DmaChannel7,
};
pub use config::DmaConfig;
use config::{
    HardwareRequest, MemoryToMemory, MemoryToPeripheral, PeripheralRequest,
    PeripheralSource, PeripheralToMemory, PeripheralToPeripheral,
    PeripheralToPeripheralDirection, TransferDirection, TransferType,
};

/// Supported word types for the STM32H5 GPDMA implementation.
///
/// Currently only u8, u16, and u32 word types are supported. Signed types are currently not
/// supported because they would add a fair bit of complexity/redundancy to the DataTransform
/// implementation. This is easy to work around by having buffers of signed types implement Deref
/// to an unsigned type of the same width.
pub trait Word: DmaWord + Default + Copy {}

impl Word for u32 {}
impl Word for u16 {}
impl Word for u8 {}

/// Errors that can occur during operation
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The DMA determined that a user setting was invalid while starting a transfer.
    ///
    /// See RM0492 Rev 3 Section 15.4.16 for details on how to debug
    UserSettingError,
    /// An error occurred transferring data during a transfer
    ///
    /// See RM0492 Rev 3 Section 15.4.16 for details on how to debug
    DataTransferError,
    /// An error occurred loading a linked transfer configuration
    ///
    /// See RM0492 Rev 3 Section 15.4.16 for details on how to debug
    LinkTransferError,

    /// Resume was called on a channel that was not previously suspended
    NotSuspendedError,
}

pub trait GpdmaExt<DMA: Instance> {
    fn channels(self, rec: DMA::Rec) -> DmaChannels<DMA>;
}

impl<DMA: Instance> GpdmaExt<DMA> for DMA {
    fn channels(self, rec: DMA::Rec) -> DmaChannels<DMA> {
        DmaChannels::new(self, rec)
    }
}

pub trait Instance: Sealed + Deref<Target = gpdma1::RegisterBlock> {
    type Rec: ResetEnable;

    fn ptr() -> *const gpdma1::RegisterBlock;

    /// Access channel registers. Valid for channels 0-5 only.
    /// # Safety
    /// This function is unsafe because it allows access to the DMA channel registers
    /// without enforcing exclusive access or checking that the channel index is valid.
    /// The caller must ensure that the channel index is within bounds and that no data races occur.
    unsafe fn ch(channel: usize) -> &'static gpdma1::CH {
        (*Self::ptr()).ch(channel)
    }

    /// Access 2D channel registers. Valid for channels 6 and 7 only.
    /// # Safety
    /// This function is unsafe because it allows access to the DMA channel registers
    /// without enforcing exclusive access or checking that the channel index is valid.
    /// The caller must ensure that the channel index is within bounds and that no data races occur.
    unsafe fn ch2d(channel: usize) -> &'static gpdma1::CH2D {
        // Note (unsafe): only accessing registers belonging to specific channel
        (*Self::ptr()).ch2d(channel - 6)
    }

    fn rec() -> Self::Rec;
}

impl Sealed for GPDMA1 {}
impl Sealed for GPDMA2 {}

impl Instance for GPDMA1 {
    type Rec = rec::Gpdma1;

    fn ptr() -> *const gpdma1::RegisterBlock {
        GPDMA1::ptr()
    }

    fn rec() -> Self::Rec {
        Self::Rec {
            _marker: PhantomData,
        }
    }
}

impl Instance for GPDMA2 {
    type Rec = rec::Gpdma2;

    fn ptr() -> *const gpdma1::RegisterBlock {
        GPDMA2::ptr()
    }

    fn rec() -> Self::Rec {
        Self::Rec {
            _marker: PhantomData,
        }
    }
}

/// DmaChannels represents the set of channels on each GPDMA peripheral. To use, simply move the
/// desired channel out of the tuple:
///
/// ```
/// let dp = pac::Peripherals::take().unwrap();
/// let channels = dp.GPDMA1.channels(ccdr.peripheral.GPDMA1);
/// let channel = channels.0;
/// ```
#[allow(private_interfaces)]
pub struct DmaChannels<D>(
    pub DmaChannel0<D>,
    pub DmaChannel1<D>,
    pub DmaChannel2<D>,
    pub DmaChannel3<D>,
    pub DmaChannel4<D>,
    pub DmaChannel5<D>,
    pub DmaChannel6<D>,
    pub DmaChannel7<D>,
);

impl<DMA: Instance> DmaChannels<DMA> {
    /// Splits the DMA peripheral into channels.
    pub(super) fn new(_regs: DMA, rec: DMA::Rec) -> Self {
        let _ = rec.reset().enable();
        Self(
            DmaChannel0::new(),
            DmaChannel1::new(),
            DmaChannel2::new(),
            DmaChannel3::new(),
            DmaChannel4::new(),
            DmaChannel5::new(),
            DmaChannel6::new(),
            DmaChannel7::new(),
        )
    }
}

/// DmaTransfer represents a single transfer operation on a GPDMA channel. It is created using the
/// [`DmaTransfer::memory_to_memory`], [`DmaTransfer::memory_to_peripheral`],
/// [`DmaTransfer::peripheral_to_memory`], or [`DmaTransfer::peripheral_to_peripheral`]
/// methods, which take a channel and the source and destination buffers. The transfer can then be
/// started using the [`DmaTransfer::start`] or [`DmaTransfer::start_nonblocking`] methods.
pub struct DmaTransfer<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    channel: &'a mut CH,
    source: S,
    destination: D,
}

impl<'a, CH, S, D> DmaTransfer<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    fn new<T>(
        channel: &'a mut CH,
        config: DmaConfig<T, S::Word, D::Word>,
        source: S,
        mut destination: D,
        size: usize,
    ) -> Self
    where
        T: TransferType,
    {
        assert!(size <= u16::MAX as usize, "Max block size is {}", u16::MAX);

        let (src_ptr, _) = unsafe { source.read_buffer() };
        let (dest_ptr, _) = unsafe { destination.write_buffer() };

        channel.reset_channel();
        channel.set_source(src_ptr);
        channel.set_destination(dest_ptr);
        channel.set_transfer_size_bytes(size);
        channel.apply_config(config);

        Self {
            channel,
            source,
            destination,
        }
    }

    /// Create a new memory-to-memory transfer with the channel, source and destination buffers
    /// provided.
    pub fn memory_to_memory(
        config: DmaConfig<MemoryToMemory, S::Word, D::Word>,
        channel: &'a mut CH,
        source: S,
        mut destination: D,
    ) -> Self {
        let src_width = core::mem::size_of::<S::Word>();
        let dest_width = core::mem::size_of::<D::Word>();

        let (_, src_words) = unsafe { source.read_buffer() };
        let src_size = src_width * src_words;
        let (_, dest_words) = unsafe { destination.write_buffer() };
        let dest_size = dest_width * dest_words;

        // Size must be aligned with destination width if source width is greater than destination
        // width and packing mode is used, therefore the maximum size must be dictated by
        // destination size (width * count). When not in packing mode, this still holds true as
        // the destination size must not be exceeded (so only read the same number of words from
        // the source as there is room in the destination)
        let size = if src_width > dest_width {
            dest_size
        } else {
            // When the source data width is less than or equal to the destination data width, we
            // just need to ensure that the destination buffer is large enough to hold all of the
            // source data.
            assert!(src_size <= dest_size, "Transfer size ({src_size} bytes) will overflow the destination buffer ({dest_size} bytes)!");
            src_size
        };

        Self::new::<MemoryToMemory>(channel, config, source, destination, size)
    }

    /// Create a new memory-to-peripheral transfer with the channel, source buffer and destination
    /// peripheral provided.
    pub fn memory_to_peripheral(
        config: DmaConfig<MemoryToPeripheral, S::Word, D::Word>,
        channel: &'a mut CH,
        source: S,
        destination: D,
    ) -> Self {
        let (_, src_words) = unsafe { source.read_buffer() };
        let src_size = core::mem::size_of::<S::Word>() * src_words;

        Self::new::<MemoryToPeripheral>(
            channel,
            config,
            source,
            destination,
            src_size,
        )
        .apply_hardware_request_config(config)
    }

    /// Create a new peripheral-to-memory transfer with the channel, source peripheral and
    /// destination buffer provided.
    pub fn peripheral_to_memory(
        config: DmaConfig<PeripheralToMemory, S::Word, D::Word>,
        channel: &'a mut CH,
        source: S,
        mut destination: D,
    ) -> Self {
        let (_, dest_words) = unsafe { destination.write_buffer() };
        let dest_size = core::mem::size_of::<D::Word>() * dest_words;

        Self::new::<PeripheralToMemory>(
            channel,
            config,
            source,
            destination,
            dest_size,
        )
        .apply_hardware_request_config(config)
        .apply_peripheral_source_config(config)
    }

    /// Create a new peripheral-to-peripheral transfer with source and destination peripherals
    /// provided.
    pub fn peripheral_to_peripheral<T>(
        config: DmaConfig<PeripheralToPeripheral<T>, S::Word, D::Word>,
        channel: &'a mut CH,
        source: S,
        mut destination: D,
    ) -> Self
    where
        T: PeripheralToPeripheralDirection,
    {
        let (_, src_words) = unsafe { source.read_buffer() };
        let (_, dest_words) = unsafe { destination.write_buffer() };

        let size = match T::DIRECTION {
            TransferDirection::PeripheralToPeripheral(
                PeripheralRequest::SourceRequest,
            ) => src_words * core::mem::size_of::<S::Word>(),
            TransferDirection::PeripheralToPeripheral(
                PeripheralRequest::DestinationRequest,
            ) => dest_words * core::mem::size_of::<D::Word>(),
            _ => unreachable!(),
        };

        Self::new::<PeripheralToPeripheral<T>>(
            channel,
            config,
            source,
            destination,
            size,
        )
        .apply_hardware_request_config(config)
        .apply_peripheral_source_config(config)
    }

    fn apply_hardware_request_config<T: HardwareRequest>(
        self,
        config: DmaConfig<T, S::Word, D::Word>,
    ) -> Self {
        self.channel.configure_hardware_request(config);
        self
    }

    fn apply_peripheral_source_config<T: PeripheralSource>(
        self,
        config: DmaConfig<T, S::Word, D::Word>,
    ) -> Self {
        self.channel.configure_peripheral_flow_control(config);
        self
    }

    fn start_transfer_internal(&mut self) {
        // Preserve the instruction and bus ordering of preceding buffer access
        // to the subsequent access by the DMA peripheral due to enabling it.
        fence(Ordering::SeqCst);

        self.channel.enable();
    }

    /// Start a transfer. Does not block waiting for the transfer to start and does not check for
    /// errors starting the transfer
    pub fn start_nonblocking(&mut self) {
        self.start_transfer_internal();
    }

    /// Start a transfer and block waiting for it to start. Returns an error if one occurred
    /// starting the transfer.
    pub fn start(&mut self) -> Result<(), Error> {
        self.start_nonblocking();
        self.channel.wait_for_transfer_started()
    }

    /// Suspend a transfer. Does not wait for channel transfer to be suspended and does not report
    /// any errors that occur doing so.
    pub fn suspend_nonblocking(&mut self) {
        if self.channel.is_suspended() {
            return;
        }
        self.channel.initiate_suspend();

        // Preserve the instruction and bus sequence of the preceding disable and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);
    }

    /// Suspend a transfer and block waiting for it to be suspended. Returns an error if one
    /// occurred during the transfer or while suspending the transfer.
    pub fn suspend(&mut self) {
        if !self.channel.is_suspended() {
            self.channel.suspend_transfer();
        }

        // Preserve the instruction and bus sequence of the preceding disable and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);
    }

    /// Resume a transfer. Does not wait for channel transfer to be suspended and does not report
    /// any errors that occur doing so.
    pub fn resume_nonblocking(&mut self) -> Result<(), Error> {
        if !self.channel.is_suspended() {
            return Err(Error::NotSuspendedError);
        }
        // Preserve the instruction and bus ordering of preceding buffer access
        // to the subsequent access by the DMA peripheral due to enabling it.
        fence(Ordering::SeqCst);

        self.channel.initiate_resume();
        Ok(())
    }

    /// Resume a transfer and block waiting for it to be resumed. Returns an error if one occurred
    /// resuming the transfer.
    pub fn resume(&mut self) -> Result<(), Error> {
        if !self.channel.is_suspended() {
            return Err(Error::NotSuspendedError);
        }
        // Preserve the instruction and bus ordering of preceding buffer access
        // to the subsequent access by the DMA peripheral due to enabling it.
        fence(Ordering::SeqCst);

        self.channel.resume_transfer()
    }

    pub fn is_running(&self) -> bool {
        self.channel.is_running()
    }

    /// Blocks waiting for a transfer to complete. Returns an error if one occurred during the
    /// transfer.
    pub fn wait_for_transfer_complete(&mut self) -> Result<(), Error> {
        let result = self.channel.wait_for_transfer_complete();
        // Preserve the instruction and bus sequence of the preceding operation and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);

        result
    }

    /// Blocks waiting for the half transfer complete event. Returns an error if one occurred during
    /// the transfer.
    pub fn wait_for_half_transfer_complete(&mut self) -> Result<(), Error> {
        self.channel.wait_for_half_transfer_complete()
    }

    /// Enable interrupts for this transfer. This will enable the transfer complete and half
    /// transfer complete interrupts, as well as error interrupts.
    pub fn enable_interrupts(&mut self) {
        self.channel.enable_transfer_interrupts();
    }

    /// Disable interrupts for this transfer.
    pub fn disable_interrupts(&mut self) {
        self.channel.disable_transfer_interrupts();
    }

    /// Abort a transaction and wait for it to suspend the transfer before resetting the channel
    pub fn abort(&mut self) {
        if self.is_running() {
            self.channel.abort();
        }

        self.disable_interrupts();

        // Preserve the instruction and bus sequence of the preceding operation and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);
    }

    pub fn free(mut self) -> (S, D) {
        self.abort();
        let (src, dest) = unsafe {
            (
                core::ptr::read(&self.source),
                core::ptr::read(&self.destination),
            )
        };
        mem::forget(self);
        (src, dest)
    }
}

impl<'a, CH, S, D> Drop for DmaTransfer<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    fn drop(&mut self) {
        self.abort();
    }
}
