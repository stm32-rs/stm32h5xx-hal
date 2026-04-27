//! Embedded Flash memory
//!
//! Support for Read, Write and Erase options on the Embedded Flash memory. This
//! module implements traits from the
//! [embedded-storage](https://github.com/rust-embedded-community/embedded-storage)
//! crate.
//!
//! # Examples
//!
//! - [Flash example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/flash.rs)
//!
//! # Supported devices
//!
//! | Reference Manual | Flash Sizes | Banks | Sector Size
//! | --- | --- | --- | ---
//! | RM0433 | 128kB, 1MB, 2MB | One or Two | 128kB
//! | RM0399 | 1MB, 2MB | Two | 128kB
//! | RM0455 | 128kB, 1MB, 2MB | One or Two | 8kB
//! | RM0468 | 128kB, 512kB, 1MB | One | 128kB

use core::iter;
use core::ops::Deref;

use crate::signature::FlashSize;
use crate::stm32::FLASH;
use embedded_storage::{nor_flash, Region};

mod operations;

// All sectors in the user main memory sectors have the same size.
pub const SECTOR_SIZE: usize = 0x2000; // 8kB

// The maximum write size is 128 bits
const USER_FLASH_MAX_WRITE_SIZE: usize = 16; // 128-bit

/// Flash erase/program error. From RM0433 Rev 7. Section 4.7
#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum Error {
    // Flash is busy
    Busy,
    // Previously incomplete operation
    Incomplete,
    /// The arguments are not properly aligned
    NotAligned,
    /// The arguments are out of bounds
    OutOfBounds,
    /// An illegal erase/program operation was attempted
    WriteProtection,
    /// The programming sequence was incorrect
    ProgrammingSequence,
    /// Application software wrote several times to the same byte
    Strobe,
    /// Write operation was attempted before the completion of the previous
    /// write operation OR a wrap burst request overlaps two or more 256-bit
    /// flash-word addresses
    Inconsistency,
    /// Error occurred during an option byte change operation
    OptionByteChange,
    /// Other errors
    Other,
}

/// Flash memory sector
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlashSector {
    /// Sector number
    pub number: u8,
    /// Offset from base memory address
    pub offset: u32,
}

impl Region for FlashSector {
    /// Returns true if given offset belongs to this sector
    fn contains(&self, offset: u32) -> bool {
        (self.offset <= offset) && (offset < (self.offset + SECTOR_SIZE as u32))
    }
}

/// Iterator of flash memory sectors within a single bank
pub struct FlashSectorIterator {
    index: u8,
    start_sector: u8,
    start_offset: u32,
    end_offset: u32,
}
impl FlashSectorIterator {
    fn new(start_sector: u8, start_offset: u32, end_offset: u32) -> Self {
        Self {
            index: 0,
            start_sector,
            start_offset,
            end_offset,
        }
    }
}
impl Iterator for FlashSectorIterator {
    type Item = FlashSector;

    fn next(&mut self) -> Option<Self::Item> {
        if self.start_offset >= self.end_offset {
            None
        } else {
            let sector = FlashSector {
                number: self.start_sector + self.index,
                offset: self.start_offset,
            };

            self.index += 1;
            self.start_offset += SECTOR_SIZE as u32;

            Some(sector)
        }
    }
}

/// Flash methods implemented for `pac::FLASH`
pub trait FlashExt {
    fn flash(self) -> Flash;
}

const USER_FLASH_BASE_ADDRESS: u32 = 0x0800_0000;

impl FlashExt for FLASH {
    fn flash(self) -> Flash {
        Flash { flash: self }
    }
}

pub struct Flash {
    flash: FLASH,
}

impl Deref for Flash {
    type Target = FLASH;

    fn deref(&self) -> &Self::Target {
        &self.flash
    }
}

impl Flash {
    fn is_configuration_locked(&self) -> bool {
        self.nscr().read().lock().bit_is_set()
    }

    fn is_busy(&self) -> bool {
        self.nssr().read().bsy().bit_is_set()
    }

    fn check_write_buffer_empty(&self) -> Result<(), Error> {
        if self.nssr().read().wbne().bit_is_set() {
            Err(Error::Incomplete)
        } else {
            Ok(())
        }
    }

    fn check_busy(&self) -> Result<(), Error> {
        if self.is_busy() {
            Err(Error::Busy)
        } else {
            Ok(())
        }
    }

    fn wait_ready(&self) {
        while self.is_busy() {}
    }

    fn wait_data_buffer_empty(&self) {
        while self.nssr().read().dbne().bit_is_set() {}
    }

    fn check_error(&self) -> Result<(), Error> {
        let sr = self.nssr().read();

        if sr.wrperr().bit_is_set() {
            Err(Error::WriteProtection)
        } else if sr.pgserr().bit_is_set() {
            Err(Error::ProgrammingSequence)
        } else if sr.strberr().bit_is_set() {
            Err(Error::Strobe)
        } else if sr.incerr().bit_is_set() {
            Err(Error::Inconsistency)
        } else if sr.optchangeerr().bit_is_set() {
            Err(Error::OptionByteChange)
        } else {
            Ok(())
        }
    }

    fn clear_error_flags(&self) {
        self.nsccr().write(|w| {
            w.clr_wrperr()
                .set_bit()
                .clr_pgserr()
                .set_bit()
                .clr_strberr()
                .set_bit()
                .clr_incerr()
                .set_bit()
                .clr_optchangeerr()
                .set_bit()
        });
    }

    fn unlock_configuration(&self) {
        if self.is_configuration_locked() {
            const UNLOCK_KEY1: u32 = 0x4567_0123;
            const UNLOCK_KEY2: u32 = 0xCDEF_89AB;

            self.nskeyr()
                .write(|w| unsafe { w.nskey().bits(UNLOCK_KEY1) });
            self.nskeyr()
                .write(|w| unsafe { w.nskey().bits(UNLOCK_KEY2) });
            assert!(!self.is_configuration_locked())
        }
    }

    fn lock_configuration(&self) {
        self.nscr().modify(|_, w| w.lock().set_bit());
    }

    fn unlock_option_byte_control(&self) -> Result<(), Error> {
        const UNLOCK_KEY1: u32 = 0x0819_2A3B;
        const UNLOCK_KEY2: u32 = 0x4C5D_6E7F;

        self.optkeyr()
            .write(|w| unsafe { w.optkey().bits(UNLOCK_KEY1) });
        self.optkeyr()
            .write(|w| unsafe { w.optkey().bits(UNLOCK_KEY2) });

        Ok(())
    }

    fn lock_option_bytes(&self) {
        self.optcr().modify(|_, w| w.optlock().set_bit());
    }

    fn enable_programming(&self) {
        self.nscr().modify(|_, w| w.pg().set_bit());
    }

    fn disable_programming(&self) {
        self.nscr().modify(|_, w| w.pg().clear_bit());
    }

    fn prepare_operation(&self) -> Result<(), Error> {
        // Ensure no effective write, erase or option byte change operation is ongoing
        self.check_busy()?;
        // Check that previous write operation completed successfully
        self.check_write_buffer_empty()?;
        // Wait for data buffer to be empty
        self.wait_data_buffer_empty();
        Ok(())
    }

    fn programming_request(&self) -> Result<ProgrammingRequest<'_>, Error> {
        self.prepare_operation()?;
        self.enable_programming();

        Ok(ProgrammingRequest { flash: self })
    }

    pub fn user_flash(&mut self) -> LockedUserFlash<'_> {
        self.unlock_configuration();
        LockedUserFlash::new(self)
    }

    pub fn otp_data(&mut self) -> Result<LockedOtpData<'_>, Error> {
        Ok(LockedOtpData::new(self))
    }

    pub fn unlock_option_bytes(
        &mut self,
    ) -> Result<UnlockedOptionBytes<'_>, Error> {
        self.unlock_option_byte_control()?;
        Ok(UnlockedOptionBytes { flash: self })
    }

    pub fn free(self) -> FLASH {
        self.flash
    }
}

struct ProgrammingRequest<'a> {
    flash: &'a Flash,
}

impl<'a> ProgrammingRequest<'a> {}

impl<'a> Drop for ProgrammingRequest<'a> {
    fn drop(&mut self) {
        self.flash.disable_programming();
    }
}

pub struct LockedUserFlash<'a> {
    flash: &'a mut Flash,
}

impl<'a> LockedUserFlash<'a> {
    pub fn new(flash: &'a mut Flash) -> Self {
        Self { flash }
    }

    pub fn region(&self) -> UserFlashRegion {
        UserFlashRegion
    }

    pub fn unlocked(&mut self) -> UnlockedUserFlash<'_> {
        self.flash.unlock_configuration();
        UnlockedUserFlash::new(self.flash)
    }
}

pub struct UnlockedUserFlash<'a> {
    flash: &'a mut Flash,
}

impl<'a> UnlockedUserFlash<'a> {
    pub fn new(flash: &'a mut Flash) -> Self {
        Self { flash }
    }

    pub fn region(&self) -> UserFlashRegion {
        UserFlashRegion
    }
}

impl<'a> Drop for UnlockedUserFlash<'a> {
    fn drop(&mut self) {
        self.flash.lock_configuration();
    }
}

#[derive(Clone, Copy)]
pub struct UserFlashRegion;

impl UserFlashRegion {
    /// Memory-mapped address
    pub fn base_address(&self) -> u32 {
        USER_FLASH_BASE_ADDRESS
    }
    /// Size in bytes
    pub fn size(&self) -> usize {
        FlashSize::bytes()
    }

    /// Returns flash memory sector of a given offset. Returns none if offset is
    /// out of range
    pub fn sector(&self, offset: u32) -> FlashSector {
        assert!(offset < self.size() as u32);
        let bank = self.bank(offset);
        let offset_within_bank = offset - self.bank_offset(bank);
        FlashSector {
            number: (offset_within_bank / SECTOR_SIZE as u32) as u8,
            offset,
        }
    }

    /// Returns iterator of flash memory sectors
    ///
    /// Sectors are returned in memory order
    pub fn bank_sector_iter(
        &self,
    ) -> impl Iterator<Item = (usize, FlashSector)> {
        // Second user main memory bank always starts at an offset of 0x10_0000
        iter::repeat(0)
            .zip(FlashSectorIterator::new(0, 0, self.bank_size() as u32))
            .chain(iter::repeat(1).zip(FlashSectorIterator::new(
                0,
                self.bank_size() as u32,
                self.size() as u32,
            )))
    }

    pub fn bank_size(&self) -> usize {
        self.size() / 2
    }

    pub fn bank_offset(&self, bank: u32) -> u32 {
        match bank {
            0 => 0,
            1 => self.bank_size() as u32,
            _ => unreachable!(),
        }
    }

    pub fn bank(&self, offset: u32) -> u32 {
        offset / self.bank_size() as u32
    }
}

impl nor_flash::ErrorType for LockedUserFlash<'_> {
    type Error = Error;
}

impl nor_flash::ReadNorFlash for LockedUserFlash<'_> {
    const READ_SIZE: usize = 1;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        let offset = offset as usize;

        let ptr = self.region().base_address() as *const _;
        let data =
            unsafe { core::slice::from_raw_parts(ptr, self.region().size()) };

        bytes.copy_from_slice(&data[offset..offset + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.region().size()
    }
}

impl nor_flash::ErrorType for UnlockedUserFlash<'_> {
    type Error = Error;
}

impl nor_flash::ReadNorFlash for UnlockedUserFlash<'_> {
    const READ_SIZE: usize = 1;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        let offset = offset as usize;

        let ptr = self.region().base_address() as *const _;
        let data =
            unsafe { core::slice::from_raw_parts(ptr, self.region().size()) };

        bytes.copy_from_slice(&data[offset..offset + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.region().size()
    }
}

pub const OTP_BASE_ADDRESS: u32 = 0x08FF_F000;
pub const OTP_SIZE: usize = 2048;
pub const OTP_SLOTS_PER_BLOCK: usize = 32;
pub const OTP_NUM_SLOTS: usize = OTP_SIZE / core::mem::size_of::<u16>();

pub struct LockedOtpData<'a> {
    flash: &'a mut Flash,
}

pub struct OtpDataRegion;

impl OtpDataRegion {
    /// Memory-mapped address
    pub fn base_address(&self) -> u32 {
        OTP_BASE_ADDRESS
    }
    /// Size in bytes
    pub fn size(&self) -> usize {
        OTP_SIZE
    }

    fn region_end_address(&self) -> u32 {
        OTP_BASE_ADDRESS + OTP_SIZE as u32
    }

    fn block(&self, slot: usize) -> usize {
        slot / OTP_SLOTS_PER_BLOCK
    }
}

impl<'a> LockedOtpData<'a> {
    fn new(flash: &'a mut Flash) -> Self {
        Self { flash }
    }

    pub fn region(&self) -> OtpDataRegion {
        OtpDataRegion
    }

    pub fn read_value(&mut self, slot: usize) -> Result<u16, Error> {
        let ptr = self.region().base_address() as *const u16;
        let ptr = unsafe { ptr.add(slot) };
        assert!(ptr < self.region().region_end_address() as *const u16);
        Ok(unsafe { core::ptr::read_volatile(ptr) })
    }
    pub fn unlocked(&mut self) -> UnlockedOtpData<'_> {
        self.flash.unlock_configuration();
        UnlockedOtpData::new(self.flash)
    }
}

pub struct UnlockedOtpData<'a> {
    flash: &'a mut Flash,
}

impl<'a> UnlockedOtpData<'a> {
    fn new(flash: &'a mut Flash) -> Self {
        Self { flash }
    }

    pub fn region(&self) -> OtpDataRegion {
        OtpDataRegion
    }

    pub fn read_value(&mut self, slot: usize) -> Result<u16, Error> {
        let ptr = self.region().base_address() as *const u16;
        let ptr = unsafe { ptr.add(slot) };
        assert!(ptr < self.region().region_end_address() as *mut u16);
        Ok(unsafe { core::ptr::read_volatile(ptr) })
    }
}

impl<'a> Drop for UnlockedOtpData<'a> {
    fn drop(&mut self) {
        self.flash.lock_configuration();
    }
}

pub struct UnlockedOptionBytes<'a> {
    flash: &'a mut Flash,
}

impl<'a> Drop for UnlockedOptionBytes<'a> {
    fn drop(&mut self) {
        self.flash.lock_option_bytes();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flash_dual_bank_1m() {
        let mut sectors = flash_sectors(1 * 1024 * 1024);

        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 0, offset: 0x00000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 1, offset: 0x20000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 2, offset: 0x40000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 3, offset: 0x60000 }));
        // Offsets 0x80000 - 0x100000 not available
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 0, offset: 0x100000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 1, offset: 0x120000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 2, offset: 0x140000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 3, offset: 0x160000 }));
        // Offsets 0x180000 - 0x200000 not available
        assert_eq!(sectors.next(), None);
    }

    /// Test the memory layout of the STM32H750xB and STM32H730
    #[test]
    fn flash_single_bank_128k() {
        let mut sectors = flash_sectors(128 * 1024);

        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 0, offset: 0x00000 }));
        assert_eq!(sectors.next(), None);
    }
}
