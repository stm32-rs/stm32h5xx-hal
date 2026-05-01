//! Device electronic signature
//!
//! (stored in system flash memory)

/// Unique Device ID register
/// 96 bits of unique device ID, factory programmed and read-only
pub struct Uid;

const UID_PTR: *const u32 = 0x08FF_F800 as _;

impl Uid {
    /// Read Unique Device ID as three 32-bit words.
    pub fn read() -> [u32; 3] {
        unsafe {
            [
                core::ptr::read_volatile(UID_PTR),
                core::ptr::read_volatile(UID_PTR.add(1)),
                core::ptr::read_volatile(UID_PTR.add(2)),
            ]
        }
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
