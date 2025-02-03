use core::marker::PhantomData;

use super::Word;

pub mod transform;
use transform::*;

/// PeripheralRequests is used for peripheral-to-peripheral transfers to indicate which side of the
/// transfer is driving the request (ie. which has the hardware request assigned)
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PeripheralRequest {
    SourceRequest,
    DestinationRequest,
}

/// The TransferDirection represents the available options for transfer types
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TransferDirection {
    MemoryToMemory,
    MemoryToPeripheral,
    PeripheralToMemory,
    PeripheralToPeripheral(PeripheralRequest),
}

/// Addressing mode represents whether the source or destination address is contiguously incremented
/// or fixed during a transfer
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum AddressingMode {
    #[default]
    ContiguouslyIncremented,
    Fixed,
}

/// Transfer type encapsulates the transfer direction and the addressing mode for both the source
/// and destination of a transfer.
pub trait TransferType: crate::Sealed + Default {
    const DIRECTION: TransferDirection;

    fn source_addressing_mode(&self) -> AddressingMode {
        AddressingMode::ContiguouslyIncremented
    }

    fn destination_addressing_mode(&self) -> AddressingMode {
        AddressingMode::ContiguouslyIncremented
    }
}

/// Transfers to or from a peripheral have these additional options
pub trait HardwareRequest {
    fn block_request(&self) -> bool;
    fn enable_block_request(&mut self);
    fn request(&self) -> u8;
    fn set_request(&mut self, request: u8);
}

/// When a peripheral is the source of the transfer it can optionally be configured in peripheral
/// flow control mode, when the peripheral supports it (currently just the I3C peripheral)
pub trait PeripheralSource {
    fn peripheral_flow_control(&self) -> bool;
    fn enable_peripheral_flow_control(&mut self);
}

/// Represents the options specifically available for peripheral-to-memory transfers
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PeripheralToMemory {
    request: u8,
    block_request: bool,
    peripheral_flow_control: bool,
}

impl crate::Sealed for PeripheralToMemory {}

impl TransferType for PeripheralToMemory {
    const DIRECTION: TransferDirection = TransferDirection::PeripheralToMemory;

    fn source_addressing_mode(&self) -> AddressingMode {
        AddressingMode::Fixed
    }
}

impl HardwareRequest for PeripheralToMemory {
    fn block_request(&self) -> bool {
        self.block_request
    }

    fn enable_block_request(&mut self) {
        self.block_request = true;
    }

    fn request(&self) -> u8 {
        self.request
    }

    fn set_request(&mut self, request: u8) {
        self.request = request;
    }
}

impl PeripheralSource for PeripheralToMemory {
    fn peripheral_flow_control(&self) -> bool {
        self.peripheral_flow_control
    }

    fn enable_peripheral_flow_control(&mut self) {
        self.peripheral_flow_control = true;
    }
}

/// Represents the options specifically available for memory-to-peripheral transfers
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MemoryToPeripheral {
    request: u8,
    block_request: bool,
}

impl crate::Sealed for MemoryToPeripheral {}

impl TransferType for MemoryToPeripheral {
    const DIRECTION: TransferDirection = TransferDirection::MemoryToPeripheral;

    fn destination_addressing_mode(&self) -> AddressingMode {
        AddressingMode::Fixed
    }
}

impl HardwareRequest for MemoryToPeripheral {
    fn block_request(&self) -> bool {
        self.block_request
    }

    fn enable_block_request(&mut self) {
        self.block_request = true;
    }

    fn request(&self) -> u8 {
        self.request
    }

    fn set_request(&mut self, request: u8) {
        self.request = request;
    }
}

/// Marker struct to indicate that the source peripheral drives the request via its request line.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SourceRequest;

/// Marker struct to indicate that the destination peripheral drives the request via its request
/// line.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DestinationRequest;

/// Indicates which peripheral in a peripheral-to-peripheral transfer is driving the request line
pub trait PeripheralToPeripheralDirection: Default + Clone + Copy {
    const DIRECTION: TransferDirection;
}

impl PeripheralToPeripheralDirection for SourceRequest {
    const DIRECTION: TransferDirection =
        TransferDirection::PeripheralToPeripheral(
            PeripheralRequest::SourceRequest,
        );
}

impl PeripheralToPeripheralDirection for DestinationRequest {
    const DIRECTION: TransferDirection =
        TransferDirection::PeripheralToPeripheral(
            PeripheralRequest::DestinationRequest,
        );
}

/// Represents the options specifically available for peripheral-to-peripheral transfers
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PeripheralToPeripheral<T> {
    _peripheral_request: PhantomData<T>,
    request: u8,
    block_request: bool,
    peripheral_flow_control: bool,
}

impl<T> crate::Sealed for PeripheralToPeripheral<T> {}

impl<T: PeripheralToPeripheralDirection> TransferType
    for PeripheralToPeripheral<T>
{
    const DIRECTION: TransferDirection = T::DIRECTION;

    fn source_addressing_mode(&self) -> AddressingMode {
        AddressingMode::Fixed
    }

    fn destination_addressing_mode(&self) -> AddressingMode {
        AddressingMode::Fixed
    }
}

impl<T> HardwareRequest for PeripheralToPeripheral<T> {
    fn block_request(&self) -> bool {
        self.block_request
    }

    fn enable_block_request(&mut self) {
        self.block_request = true;
    }

    fn request(&self) -> u8 {
        self.request
    }

    fn set_request(&mut self, request: u8) {
        self.request = request;
    }
}

impl<T> PeripheralSource for PeripheralToPeripheral<T> {
    fn peripheral_flow_control(&self) -> bool {
        self.peripheral_flow_control
    }

    fn enable_peripheral_flow_control(&mut self) {
        self.peripheral_flow_control = true;
    }
}

/// Marker struct for memory-to-memory transfers (no special options)
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MemoryToMemory;

impl crate::Sealed for MemoryToMemory {}

impl TransferType for MemoryToMemory {
    const DIRECTION: TransferDirection = TransferDirection::MemoryToMemory;
}

/// Priority of the transfer. Used by the GPDMA channel arbitration to determine which transfer
/// to service.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Priority {
    LowPriorityLowWeight = 0,
    #[default]
    LowPriorityMedWeight = 1,
    LowPriorityHighWeight = 2,
    HighPriority = 3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Continuation {
    #[default]
    Direct,
    LinkedList,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum AhbPort {
    #[default]
    Port0 = 0,
    Port1 = 1,
}

const MAX_BURST_LEN: u8 = 64;

/// Configuration options for a DMA transfer
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DmaConfig<T, S, D> {
    _src_word: PhantomData<S>,
    _dest_word: PhantomData<D>,
    pub(super) transfer_type: T,
    pub(super) priority: Priority,
    pub(super) source_ahb_port: AhbPort,
    pub(super) destination_ahb_port: AhbPort,
    pub(super) source_burst_length: u8,
    pub(super) destination_burst_length: u8,
    pub(super) enable_interrupts: bool,
    pub(super) data_transform: Option<DataTransform>,
}

impl<T: TransferType, S: Word, D: Word> DmaConfig<T, S, D> {
    /// Create a config with default settings
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the priority of the transfer. Default: Low Priority, Medium Weight
    pub fn priority(mut self, priority: Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Set the source AHB port (0 or 1). Default: 0.
    pub fn source_ahb_port(mut self, port: AhbPort) -> Self {
        self.source_ahb_port = port;
        self
    }

    /// Set the destination AHB port (0 or 1). Default 0.
    pub fn destination_ahb_port(mut self, port: AhbPort) -> Self {
        self.destination_ahb_port = port;
        self
    }

    /// Set the source burst length in words (1 - 64 incl.). Default 1.
    pub fn source_burst_length(mut self, len: u8) -> Self {
        assert!(
            (1..=MAX_BURST_LEN).contains(&len),
            "Must specify a burst length between 1 and 64"
        );
        self.source_burst_length = len - 1;
        self
    }

    /// Set the destination burst length in words (1 - 64 incl.). Default 1.
    pub fn destination_burst_length(mut self, len: u8) -> Self {
        assert!(
            (1..=MAX_BURST_LEN).contains(&len),
            "Must specify a burst length between 1 and 64"
        );
        self.destination_burst_length = len - 1;
        self
    }

    pub fn enable_interrupts(mut self) -> Self {
        self.enable_interrupts = true;
        self
    }

    /// Apply a data transform via a closure that takes a DataTransformBuilder that provides APIs
    /// relevant to the source and destination data widths.
    pub fn with_data_transform(
        mut self,
        builder: DataTransformBuilder<S, D>,
    ) -> Self {
        self.data_transform = Some(builder.transform);
        self
    }
}

impl<T: PeripheralSource, S: Word, D: Word> DmaConfig<T, S, D> {
    /// Enable peripheral flow control (only supported by I3C)
    pub fn enable_peripheral_flow_control(mut self) -> Self {
        self.transfer_type.enable_peripheral_flow_control();
        self
    }
}

impl<T: HardwareRequest, S: Word, D: Word> DmaConfig<T, S, D> {
    /// Enable block requests for peripherals that support it
    pub fn enable_hardware_block_requests(mut self) -> Self {
        self.transfer_type.enable_block_request();
        self
    }

    /// Select the hardware request line
    pub fn with_request(mut self, request: u8) -> Self {
        self.transfer_type.set_request(request);
        self
    }
}

#[cfg(test)]
mod test {
    use crate::gpdma::{
        config::{self, MemoryToMemory},
        DmaConfig,
    };

    use super::*;

    impl DataTransform {
        fn new(
            source_byte_exchange: bool,
            padding_alignment: PaddingAlignmentMode,
            dest_half_word_exchange: bool,
            dest_byte_exchange: bool,
        ) -> Self {
            Self {
                source_byte_exchange,
                padding_alignment,
                dest_half_word_exchange,
                dest_byte_exchange,
            }
        }
    }

    #[test]
    fn test_data_transform() {
        let builder: DataTransformBuilder<u32, u16> =
            DataTransform::builder().swap_source_middle_bytes();
        assert_eq!(
            builder.transform,
            DataTransform::new(true, Default::default(), false, false)
        );
    }

    #[test]
    fn test_with_data_transform() {
        let config: DmaConfig<MemoryToMemory, u32, u16> = DmaConfig::new();
        let transform = DataTransform::builder()
            .swap_source_middle_bytes()
            .left_align_right_truncate()
            .swap_destination_half_word_byte_order();
        let config = config.with_data_transform(transform);
        assert_eq!(
            config.data_transform,
            Some(DataTransform::new(
                true,
                PaddingAlignmentMode::LeftAlignedRightTruncated,
                false,
                true
            ))
        );
    }
}
