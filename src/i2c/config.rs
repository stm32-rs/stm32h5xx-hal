use super::AddressMode;

/// A structure for specifying the I2C Target configuration
///
/// This structure uses the builder pattern to generate the configuration:
///
/// ```
/// let config = Config::new().secondary_address(0x10);
/// ```
#[derive(Copy, Clone)]
pub struct TargetConfig {
    /// Address mode of the MCU acting as a target
    pub(crate) own_address_mode: AddressMode,
    /// Target address for MCU
    pub(crate) own_address: u16,
    /// Secondary target address for MCU (7-bit only)
    pub(crate) secondary_address: Option<u8>,
    // Address mask for secondary mask
    pub(crate) secondary_address_mask_bits: Option<u8>,
    /// Frequency at which bus is expected to run.
    pub(crate) bus_frequency_hz: u32,
}

impl TargetConfig {
    /// Create a default configuration with the address of the MCU and the expected bus frequency.
    ///
    /// The address should be specified unshifted. 7-bit addressing mode is used by default.
    ///
    /// If this is not known, use a lower bound in order to ensure that the correct timing
    /// parameters are used.
    pub const fn new(own_address: u16, bus_frequency_hz: u32) -> Self {
        TargetConfig {
            own_address_mode: AddressMode::AddressMode7bit,
            own_address,
            secondary_address: None,
            secondary_address_mask_bits: None,
            bus_frequency_hz,
        }
    }

    /// Set the primary address that the target will listen to. The address should be specified
    /// unshifted. For 7-bit addresses the driver will shift it left by 1 to match the peripheral's
    /// expectation. 10 bit addresses are not shifted.
    pub const fn own_address(mut self, own_address: u16) -> Self {
        self.own_address = own_address;
        self
    }

    /// Set the addressing mode for the primary (own) address. Secondary addresses can only be
    /// specified as 7 bit addresses.
    pub const fn own_address_mode(mut self, address_mode: AddressMode) -> Self {
        self.own_address_mode = address_mode;
        self
    }

    /// Optional secondary 7-bit address for which the MCU can listen. This can also be masked
    /// (using secondary_address_mask()) to enable the MCU to listen to a range of addresses. The
    /// address should be specified unshifted. For 7-bit addresses the driver will shift it left by
    /// 1 to match the peripheral's expectation.
    pub const fn secondary_address(mut self, secondary_address: u8) -> Self {
        self.secondary_address = Some(secondary_address);
        self
    }

    /// Mask bits for secondary address. This allows the MCU to listen to a range of addresses. The
    /// lower `mask_bits` bits will be masked.
    pub const fn secondary_address_mask_bits(mut self, mask_bits: u8) -> Self {
        self.secondary_address_mask_bits = Some(mask_bits);
        self
    }
}
