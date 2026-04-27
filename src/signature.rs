//! Device electronic signature
//!
//! (stored in system flash memory)

/// Uniqure Device ID register
pub struct Uid;

const UID_PTR: *const u8 = 0x08FF_F800 as _;

impl Uid {
    /// Read Unique Device ID
    pub fn read() -> &'static [u8; 12] {
        unsafe { &*UID_PTR.cast::<[u8; 12]>() }
    }
}

/// Size of integrated flash
pub struct FlashSize;

const FLASH_SIZE_PTR: *const u16 = 0x08FF_F80C as _;

impl FlashSize {
    /// Read flash size in kilobytes
    pub fn kilo_bytes() -> usize {
        let size = unsafe { *FLASH_SIZE_PTR };
        size as usize
    }

    /// Read flash size in bytes
    pub fn bytes() -> usize {
        Self::kilo_bytes() * 1024
    }
}
