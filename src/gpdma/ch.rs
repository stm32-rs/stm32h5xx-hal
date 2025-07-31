use core::{marker::PhantomData, ops::Deref};

use crate::stm32::gpdma1::{
    self,
    ch::{CR, DAR, FCR, LBAR, SAR, SR, TR1, TR2},
};
use crate::Sealed;

use super::{
    config::{
        transform::{DataTransform, PaddingAlignmentMode},
        AddressingMode, AhbPort, HardwareRequest, PeripheralRequest,
        PeripheralSource, Priority, TransferDirection, TransferType,
    },
    DmaConfig, Error, Instance, Word,
};

pub(super) trait ChannelRegs: Sealed {
    #[allow(unused)] // TODO: this will be used for linked-list transfers
    fn lbar(&self) -> &LBAR;
    fn fcr(&self) -> &FCR;
    fn sr(&self) -> &SR;
    fn cr(&self) -> &CR;
    fn tr1(&self) -> &TR1;
    fn tr2(&self) -> &TR2;
    fn sar(&self) -> &SAR;
    fn dar(&self) -> &DAR;
    fn set_block_size(&self, size: u16);
}

impl Sealed for gpdma1::CH {}
impl Sealed for gpdma1::CH2D {}

impl ChannelRegs for gpdma1::CH {
    fn lbar(&self) -> &LBAR {
        self.lbar()
    }
    fn fcr(&self) -> &FCR {
        self.fcr()
    }
    fn sr(&self) -> &SR {
        self.sr()
    }
    fn cr(&self) -> &CR {
        self.cr()
    }
    fn tr1(&self) -> &TR1 {
        self.tr1()
    }
    fn tr2(&self) -> &TR2 {
        self.tr2()
    }
    fn sar(&self) -> &SAR {
        self.sar()
    }
    fn dar(&self) -> &DAR {
        self.dar()
    }
    fn set_block_size(&self, size: u16) {
        self.br1().modify(|_, w| w.bndt().set(size));
    }
}

impl ChannelRegs for gpdma1::CH2D {
    fn lbar(&self) -> &LBAR {
        self.lbar()
    }
    fn fcr(&self) -> &FCR {
        self.fcr()
    }
    fn sr(&self) -> &SR {
        self.sr()
    }
    fn cr(&self) -> &CR {
        self.cr()
    }
    fn tr1(&self) -> &TR1 {
        self.tr1()
    }
    fn tr2(&self) -> &TR2 {
        self.tr2()
    }
    fn sar(&self) -> &SAR {
        self.sar()
    }
    fn dar(&self) -> &DAR {
        self.dar()
    }
    fn set_block_size(&self, size: u16) {
        self.br1().modify(|_, w| w.bndt().set(size));
    }
}

/// DmaChannelRef provides access to individual channels of the GPDMA instance via Deref.
/// It implements the Channel and DmaChannel traits, and is exposed to user code via the DmaChannels
/// struct. It does not expose a public API to allow user code to use it directly, but should rather
/// be assigned to a DmaTransfer that manages a single transfer on a channel.
#[doc(hidden)]
pub struct DmaChannelRef<DMA, CH, const N: usize> {
    _dma: PhantomData<DMA>,
    _ch: PhantomData<CH>,
}

impl<DMA: Instance, const N: usize> Deref
    for DmaChannelRef<DMA, gpdma1::CH, N>
{
    type Target = gpdma1::CH;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        // Note (unsafe): only accessing registers belonging to Channel N
        unsafe { DMA::ch(N) }
    }
}

impl<DMA: Instance, const N: usize> Deref
    for DmaChannelRef<DMA, gpdma1::CH2D, N>
{
    type Target = gpdma1::CH2D;

    #[inline(always)]
    fn deref(&self) -> &Self::Target {
        // Note (unsafe): only accessing registers belonging to Channel N
        unsafe { DMA::ch2d(N) }
    }
}

#[allow(private_bounds)]
impl<DMA, CH, const N: usize> DmaChannelRef<DMA, CH, N>
where
    DMA: Instance,
    CH: ChannelRegs,
{
    pub(super) fn new() -> Self {
        DmaChannelRef {
            _dma: PhantomData,
            _ch: PhantomData,
        }
    }
}

impl<DMA, CH, const N: usize> Sealed for DmaChannelRef<DMA, CH, N> {}

/// Non-error transfer event, including transfer complete and half-transfer events. Half-transfer
/// events can be used for double-buffering/linked buffer transfers.
pub enum TransferEvent {
    /// Transfer complete event has occurred
    TransferComplete,
    /// Half transfer event has occurred
    HalfTransferComplete,
}

// Checks for errors in the captured status register provided, and returns a Result<(), Error>
macro_rules! check_error {
    ($sr:expr) => {
        if $sr.usef().is_trigger() {
            Err(Error::UserSettingError)
        } else if $sr.dtef().is_trigger() {
            Err(Error::DataTransferError)
        } else if $sr.ulef().is_trigger() {
            Err(Error::LinkTransferError)
        } else {
            Ok(())
        }
    };
}

#[allow(private_bounds)]
impl<DMA, CH, const N: usize> DmaChannelRef<DMA, CH, N>
where
    DMA: Instance,
    CH: ChannelRegs,
    Self: Deref<Target = CH>,
{
    #[inline(always)]
    fn reset(&self) {
        self.cr().modify(|_, w| w.reset().reset());
    }

    // TODO: remove clippy allow when used. This will likely be useful in the future
    #[allow(unused)]
    #[inline(always)]
    pub(super) fn is_enabled(&self) -> bool {
        self.cr().read().en().is_enabled()
    }

    /// Initiates the suspension of a transfer
    #[inline(always)]
    pub(super) fn suspend(&self) {
        self.cr().modify(|_, w| w.susp().suspended());
    }

    /// Resume transfer
    #[inline(always)]
    fn resume(&self) {
        self.cr().modify(|_, w| w.susp().not_suspended());
    }

    /// Clear all event flags in the FCR register.
    fn clear_all_event_flags(&self) {
        self.fcr().write(|w| {
            w.tcf()
                .clear()
                .htf()
                .clear()
                .dtef()
                .clear()
                .usef()
                .clear()
                .ulef()
                .clear()
                .suspf()
                .clear()
                .tof()
                .clear()
        });
    }

    #[inline(always)]
    /// Checks if the specified transfer event has triggered or if an error has occurred. If an
    /// error has occurred, it is returned. If the event has triggered, `Ok(true)` is returned.
    /// Otherwise, if the event has not triggered, `Ok(false)` is returned.
    fn check_transfer_event(
        &self,
        event: TransferEvent,
    ) -> Result<bool, Error> {
        let sr = self.sr().read();
        check_error!(sr)?;
        let triggered = match event {
            TransferEvent::TransferComplete => sr.tcf().is_trigger(),
            TransferEvent::HalfTransferComplete => sr.htf().is_trigger(),
        };

        Ok(triggered)
    }

    fn clear_transfer_event_flag(&self, event: TransferEvent) {
        self.fcr().write(|w| match event {
            TransferEvent::TransferComplete => w.tcf().clear(),
            TransferEvent::HalfTransferComplete => w.htf().clear(),
        });
    }

    // TODO: Remove clippy allow when FIFO use is implemented
    #[allow(unused)]
    #[inline(always)]
    fn fifo_level(&self) -> u8 {
        self.sr().read().fifol().bits()
    }

    /// Checks if the channel is idle. Ignores error conditions.
    #[inline(always)]
    fn is_idle(&self) -> bool {
        self.sr().read().idlef().is_trigger()
    }

    #[inline(always)]
    fn check_idle(&self) -> Result<bool, Error> {
        let sr = self.sr().read();
        check_error!(sr)?;
        Ok(sr.idlef().is_trigger())
    }

    #[inline(always)]
    fn set_source_address(&self, addr: u32) {
        self.sar().write(|w| w.sa().set(addr));
    }

    #[inline(always)]
    fn set_destination_address(&self, addr: u32) {
        self.dar().write(|w| w.da().set(addr));
    }

    #[inline(always)]
    fn set_source_addressing_mode(&self, mode: AddressingMode) {
        self.tr1().modify(|_, w| match mode {
            AddressingMode::ContiguouslyIncremented => w.sinc().contiguous(),
            AddressingMode::Fixed => w.sinc().fixed_burst(),
        });
    }

    #[inline(always)]
    fn set_destination_addressing_mode(&self, mode: AddressingMode) {
        self.tr1().modify(|_, w| match mode {
            AddressingMode::ContiguouslyIncremented => w.dinc().contiguous(),
            AddressingMode::Fixed => w.dinc().fixed_burst(),
        });
    }

    #[inline(always)]
    fn set_source_burst_length(&self, burst_length: u8) {
        self.tr1().modify(|_, w| w.dbl_1().set(burst_length));
    }

    #[inline(always)]
    fn set_destination_burst_length(&self, burst_length: u8) {
        self.tr1().modify(|_, w| w.sbl_1().set(burst_length));
    }

    #[inline(always)]
    fn set_source_ahb_port(&self, port: AhbPort) {
        self.tr1().modify(|_, w| match port {
            AhbPort::Port0 => w.sap().port0(),
            AhbPort::Port1 => w.sap().port1(),
        });
    }

    #[inline(always)]
    fn set_destination_ahb_port(&self, port: AhbPort) {
        self.tr1().modify(|_, w| match port {
            AhbPort::Port0 => w.dap().port0(),
            AhbPort::Port1 => w.dap().port1(),
        });
    }

    #[inline(always)]
    fn set_source_data_width(&self, width: usize) {
        self.tr1().modify(|_, w| match width {
            1 => w.sdw_log2().byte(),
            2 => w.sdw_log2().half_word(),
            4 => w.sdw_log2().word(),
            _ => unreachable!(),
        });
    }

    #[inline(always)]
    fn set_destination_data_width(&self, width: usize) {
        self.tr1().modify(|_, w| match width {
            1 => w.ddw_log2().byte(),
            2 => w.ddw_log2().half_word(),
            4 => w.ddw_log2().word(),
            _ => unreachable!(),
        });
    }

    #[inline(always)]
    fn set_source_byte_exchange(&self, source_byte_exchange: bool) {
        self.tr1().modify(|_, w| {
            if source_byte_exchange {
                w.sbx().exchanged()
            } else {
                w.sbx().not_exchanged()
            }
        });
    }

    #[inline(always)]
    fn set_padding_alignment_mode(&self, pam: PaddingAlignmentMode) {
        self.tr1().modify(|_, w| match pam {
            PaddingAlignmentMode::None => w,
            _ => w.pam().set(pam.bits()),
        });
    }

    #[inline(always)]
    fn set_destination_half_word_exchange(&self, half_word_exchange: bool) {
        self.tr1().modify(|_, w| {
            if half_word_exchange {
                w.dhx().exchanged()
            } else {
                w.dhx().not_exchanged()
            }
        });
    }

    #[inline(always)]
    fn set_destination_byte_exchange(&self, destination_byte_exchange: bool) {
        self.tr1().modify(|_, w| {
            if destination_byte_exchange {
                w.dbx().exchanged()
            } else {
                w.dbx().not_exchanged()
            }
        });
    }

    #[inline(always)]
    fn set_priority(&self, priority: Priority) {
        self.cr().modify(|_, w| match priority {
            Priority::LowPriorityLowWeight => w.prio().low_prio_low_weight(),
            Priority::LowPriorityMedWeight => w.prio().low_prio_mid_weight(),
            Priority::LowPriorityHighWeight => w.prio().low_prio_high_weight(),
            Priority::HighPriority => w.prio().high_prio(),
        });
    }

    #[inline(always)]
    fn set_transfer_type(&self, transfer_dir: TransferDirection) {
        self.tr2().modify(|_, w| match transfer_dir {
            TransferDirection::MemoryToMemory => w.swreq().software(),
            TransferDirection::MemoryToPeripheral => {
                w.swreq().hardware().dreq().destination()
            }
            TransferDirection::PeripheralToMemory => {
                w.swreq().hardware().dreq().source()
            }
            TransferDirection::PeripheralToPeripheral(
                PeripheralRequest::SourceRequest,
            ) => w.swreq().hardware().dreq().source(),
            TransferDirection::PeripheralToPeripheral(
                PeripheralRequest::DestinationRequest,
            ) => w.swreq().hardware().dreq().destination(),
        });
    }

    // TODO: Use enum?
    #[inline(always)]
    fn set_request_line(&self, request: u8) {
        self.tr2()
            .modify(|_, w| unsafe { w.reqsel().bits(request) });
    }

    #[inline(always)]
    fn set_block_request_mode(&self, block_requests_enabled: bool) {
        self.tr2().modify(|_, w| {
            if block_requests_enabled {
                w.breq().block()
            } else {
                w.breq().burst()
            }
        });
    }

    #[inline(always)]
    fn set_peripheral_flow_control_mode(
        &self,
        peripheral_control_enabled: bool,
    ) {
        self.tr2().modify(|_, w| {
            if peripheral_control_enabled {
                w.pfreq().peripheral_control_mode()
            } else {
                w.pfreq().gpdma_control_mode()
            }
        });
    }
}

/// The Channel trait is a private trait that abstracts over control of the linear and 2D channels.
/// It exposes to the DmaTransfer struct all the methods needed to control transfers on a particular
/// channel. It is private in order to not expose the low level functionality beyond the gpdma
/// module.
#[doc(hidden)]
pub(super) trait Channel {
    fn enable(&mut self);

    fn is_suspended(&self) -> bool;

    /// Initiates the suspension of a transfer
    fn initiate_suspend(&mut self);

    /// Resume transfer
    fn initiate_resume(&self);

    /// Checks whether the channel transfer is complete. If the channel indicates an error occurred,
    /// during the transaction an `Error`` is returned.
    fn check_transfer_complete(&self) -> Result<bool, Error>;

    /// Checks whether the channel half transfer complete event has triggered. If the channel
    /// indicates an error occurred, during the transaction an `Error`` is returned.
    fn check_half_transfer_complete(&self) -> Result<bool, Error>;

    /// Checks whether the channel transfer has started (has transitioned out of the idle state, or
    /// the transfer complete event has already triggered if it is idle)
    fn check_transfer_started(&self) -> Result<bool, Error>;

    fn is_running(&self) -> bool;

    /// Reset the channel registers so it can be reused.
    fn reset_channel(&mut self);

    /// Suspend the transfer and blocks until it has been suspended. Reports any that occur while
    /// waiting for the transfer to suspend.
    fn suspend_transfer(&mut self);

    /// Resumes a suspended transfer and blocks until the channel transitions out of the idle state
    /// Reports any errors that occur resuming the transfer.
    fn resume_transfer(&mut self) -> Result<(), Error>;

    /// Aborts an operation by suspending the transfer and resetting the channel.
    fn abort(&mut self);

    /// Blocks waiting for a transfer to be started (or for it to be idle and complete). Reports any
    /// errors that occur while waiting for the transfer to start.
    fn wait_for_transfer_started(&mut self) -> Result<(), Error>;

    /// Blocks waiting for a transfer to complete. Reports any errors that occur during a transfer.
    fn wait_for_transfer_complete(&mut self) -> Result<(), Error>;

    /// Blocks waiting for a half transfer event to trigger. Reports any errors that occur during a
    /// transfer.
    fn wait_for_half_transfer_complete(&mut self) -> Result<(), Error>;

    /// Apply a transfer configuration to the channel
    fn apply_config<T: TransferType, S: Word, D: Word>(
        &mut self,
        config: DmaConfig<T, S, D>,
    );

    /// Apply hardware request configuration to the channel. Not relevant to memory-to-memory
    /// transfers.
    fn configure_hardware_request<T: HardwareRequest, S: Word, D: Word>(
        &mut self,
        config: DmaConfig<T, S, D>,
    );

    /// Apply peripheral flow control configuration for transactions where a peripheral is the
    /// source
    fn configure_peripheral_flow_control<
        T: PeripheralSource,
        S: Word,
        D: Word,
    >(
        &mut self,
        config: DmaConfig<T, S, D>,
    );

    /// Apply a data transform to the channel transfer
    fn apply_data_transform(&mut self, data_transform: DataTransform);
    /// Set the source address. This sets the source address and data width.
    fn set_source<W: Word>(&mut self, ptr: *const W);

    /// Set the destination address. This sets the destination address and data width
    fn set_destination<W: Word>(&mut self, ptr: *mut W);

    /// Set the transfer size in bytes (not words!). Size must be aligned with destination width if
    /// source width is greater than destination width and packing mode is used. Otherwise the size
    /// must be aligned with the source data width.
    fn set_transfer_size_bytes(&mut self, size: usize);

    /// Enable transfer interrupts for the channel. This enables the transfer complete,
    /// half-transfer complete, data transfer error and user setting error interrupts. This is
    /// useful for starting a transfer that will be monitored by an interrupt handler.
    fn enable_transfer_interrupts(&mut self);

    /// Disable transfer interrupts for the channel. It is expected that this will be called from
    /// an interrupt handler after a transfer is completed.
    fn disable_transfer_interrupts(&mut self);
}

impl<DMA, CH, const N: usize> Channel for DmaChannelRef<DMA, CH, N>
where
    DMA: Instance,
    CH: ChannelRegs,
    Self: Deref<Target = CH>,
{
    #[inline(always)]
    fn enable(&mut self) {
        self.cr().modify(|_, w| w.en().enabled());
    }

    #[inline(always)]
    fn is_suspended(&self) -> bool {
        self.sr().read().suspf().bit_is_set()
    }

    fn initiate_suspend(&mut self) {
        if self.is_suspended() {
            return;
        }
        self.suspend();
    }

    #[inline(always)]
    fn initiate_resume(&self) {
        self.resume();
    }

    fn check_transfer_complete(&self) -> Result<bool, Error> {
        self.check_transfer_event(TransferEvent::TransferComplete)
    }

    fn check_half_transfer_complete(&self) -> Result<bool, Error> {
        self.check_transfer_event(TransferEvent::HalfTransferComplete)
    }

    fn check_transfer_started(&self) -> Result<bool, Error> {
        // TODO: Resolve multiple status register reads
        match self.check_idle() {
            // If we're idle we might have finished the transaction already, so also check if the
            // transfer complete flag is set
            Ok(true) => self.check_transfer_complete(),
            Ok(false) => Ok(false),
            Err(error) => Err(error),
        }
    }

    #[inline(always)]
    fn is_running(&self) -> bool {
        !self.is_idle()
    }

    fn reset_channel(&mut self) {
        self.reset();
        self.clear_all_event_flags();
    }

    fn suspend_transfer(&mut self) {
        self.initiate_suspend();
        while !self.is_suspended() {}
    }

    fn resume_transfer(&mut self) -> Result<(), Error> {
        self.initiate_resume();
        while !self.check_transfer_started()? {}
        Ok(())
    }

    fn abort(&mut self) {
        if !self.is_idle() {
            self.suspend_transfer();
        }

        self.reset_channel();
    }

    fn wait_for_transfer_started(&mut self) -> Result<(), Error> {
        while !self.check_transfer_started().inspect_err(|_| {
            self.clear_all_event_flags();
        })? {}
        Ok(())
    }

    fn wait_for_transfer_complete(&mut self) -> Result<(), Error> {
        loop {
            match self.check_transfer_complete() {
                Ok(true) => {
                    self.clear_transfer_event_flag(
                        TransferEvent::TransferComplete,
                    );
                    return Ok(());
                }
                Ok(false) => continue,
                Err(error) => {
                    self.clear_all_event_flags();
                    return Err(error);
                }
            }
        }
    }

    fn wait_for_half_transfer_complete(&mut self) -> Result<(), Error> {
        loop {
            match self.check_half_transfer_complete() {
                Ok(true) => {
                    self.clear_transfer_event_flag(
                        TransferEvent::HalfTransferComplete,
                    );
                    return Ok(());
                }
                Ok(false) => continue,
                Err(error) => {
                    self.clear_all_event_flags();
                    return Err(error);
                }
            }
        }
    }

    fn apply_config<T: TransferType, S: Word, D: Word>(
        &mut self,
        config: DmaConfig<T, S, D>,
    ) {
        self.set_source_addressing_mode(
            config.transfer_type.source_addressing_mode(),
        );
        self.set_destination_addressing_mode(
            config.transfer_type.destination_addressing_mode(),
        );
        self.set_source_burst_length(config.source_burst_length);
        self.set_destination_burst_length(config.destination_burst_length);
        self.set_source_ahb_port(config.source_ahb_port);
        self.set_destination_ahb_port(config.destination_ahb_port);

        self.set_transfer_type(T::DIRECTION);
        self.set_priority(config.priority);
        if config.enable_interrupts {
            self.enable_transfer_interrupts();
        }
        if let Some(data_transform) = config.data_transform {
            self.apply_data_transform(data_transform);
        }
    }

    fn configure_hardware_request<T: HardwareRequest, S: Word, D: Word>(
        &mut self,
        config: DmaConfig<T, S, D>,
    ) {
        self.set_block_request_mode(config.transfer_type.block_request());
        self.set_request_line(config.transfer_type.request());
    }

    fn configure_peripheral_flow_control<
        T: PeripheralSource,
        S: Word,
        D: Word,
    >(
        &mut self,
        config: DmaConfig<T, S, D>,
    ) {
        self.set_peripheral_flow_control_mode(
            config.transfer_type.peripheral_flow_control(),
        );
    }

    fn apply_data_transform(&mut self, data_transform: DataTransform) {
        self.set_source_byte_exchange(data_transform.source_byte_exchange);
        self.set_padding_alignment_mode(data_transform.padding_alignment);
        self.set_destination_half_word_exchange(
            data_transform.dest_half_word_exchange,
        );
        self.set_destination_byte_exchange(data_transform.dest_byte_exchange);
    }

    fn set_source<W: Word>(&mut self, ptr: *const W) {
        self.set_source_address(ptr as u32);
        self.set_source_data_width(core::mem::size_of::<W>());
    }

    fn set_destination<W: Word>(&mut self, ptr: *mut W) {
        self.set_destination_address(ptr as u32);
        self.set_destination_data_width(core::mem::size_of::<W>());
    }

    fn set_transfer_size_bytes(&mut self, size: usize) {
        self.set_block_size(size as u16);
    }

    #[inline(always)]
    fn enable_transfer_interrupts(&mut self) {
        self.cr().modify(|_, w| {
            w.tcie().enabled().dteie().enabled().useie().enabled()
        });
    }

    #[inline(always)]
    fn disable_transfer_interrupts(&mut self) {
        self.cr().modify(|_, w| {
            w.tcie().disabled().dteie().disabled().useie().disabled()
        });
    }
}

#[cfg(feature = "gpdma-futures")]
pub use super::future::DmaChannel;

/// DmaChannel trait provides the API contract that all GPDMA channels exposed to the user
/// implement.
#[cfg(not(feature = "gpdma-futures"))]
#[allow(private_bounds)]
pub trait DmaChannel: Channel {}

#[cfg(not(feature = "gpdma-futures"))]
#[allow(private_bounds)]
impl<DMA, CH, const N: usize> DmaChannel for DmaChannelRef<DMA, CH, N>
where
    DMA: Instance,
    CH: ChannelRegs,
    Self: Deref<Target = CH>,
{
}

/// Channel 0 on GPDMA controller
pub type DmaChannel0<D> = DmaChannelRef<D, gpdma1::CH, 0>;
/// Channel 1 on GPDMA controller
pub type DmaChannel1<D> = DmaChannelRef<D, gpdma1::CH, 1>;
/// Channel 2 on GPDMA controller
pub type DmaChannel2<D> = DmaChannelRef<D, gpdma1::CH, 2>;
/// Channel 3 on GPDMA controller
pub type DmaChannel3<D> = DmaChannelRef<D, gpdma1::CH, 3>;
/// Channel 4 on GPDMA controller
pub type DmaChannel4<D> = DmaChannelRef<D, gpdma1::CH, 4>;
/// Channel 5 on GPDMA controller
pub type DmaChannel5<D> = DmaChannelRef<D, gpdma1::CH, 5>;
/// Channel 6 on GPDMA controller
pub type DmaChannel6<D> = DmaChannelRef<D, gpdma1::CH2D, 6>;
/// Channel 7 on GPDMA controller
pub type DmaChannel7<D> = DmaChannelRef<D, gpdma1::CH2D, 7>;
