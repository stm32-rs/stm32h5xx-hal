//! Erase, write, read operations for Flash

use core::ptr;
use core::sync::atomic::{fence, Ordering};

use embedded_storage::nor_flash::{
    check_erase, NorFlash, NorFlashError, NorFlashErrorKind,
};
use embedded_storage::Region;

use crate::stm32::FLASH;

use super::{
    Error, UnlockedOptionBytes, UnlockedOtpData, UnlockedUserFlash,
    OTP_NUM_SLOTS, USER_FLASH_MAX_WRITE_SIZE,
};

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match &self {
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            Error::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            _ => NorFlashErrorKind::Other,
        }
    }
}

/// Erase and program operations for unlocked user flash
///
/// # Examples
///
/// ```no_run
/// use stm32h5xx_hal::pac::Peripherals;
/// use stm32h5xx_hal::flash::FlashExt;
/// use embedded_storage::nor_flash::NorFlash;
///
/// let dp = Peripherals::take().unwrap();
/// let mut flash = dp.FLASH.flash();
/// let mut user_flash = flash.user_flash();
/// let mut unlocked = user_flash.unlocked();
///
/// // Erase the second 8 kB sector (bank 0, sector 1).
/// NorFlash::erase(&mut unlocked, 1 * 8 * 1024, 2 * 8 * 1024).unwrap();
///
/// // Write 16 bytes (one 128-bit flash word) at the start of sector 1.
/// let buf = [0u8; 16];
/// NorFlash::write(&mut unlocked, 1 * 8 * 1024, &buf).unwrap();
///
/// // Flash is re-locked when `unlocked` is dropped.
/// ```
impl UnlockedUserFlash<'_> {
    /// Erase a flash sector
    ///
    /// Refer to the reference manual to see which sector corresponds to which
    /// memory address. Out of bounds sectors will cause an error.
    pub fn erase_sector(
        &mut self,
        bank: usize,
        sector: usize,
    ) -> Result<(), Error> {
        if sector * super::SECTOR_SIZE >= self.region().bank_size() {
            return Err(Error::OutOfBounds);
        }

        self.flash.wait_ready();
        self.flash.prepare_operation()?;

        // Clear all the error flags due to previous programming/erase
        self.flash.clear_error_flags();

        self.flash.nscr().modify(|_, w| unsafe {
            match bank {
                0 => w.bksel().clear_bit(),
                1 => w.bksel().set_bit(),
                _ => unreachable!(),
            }
            .ser()
            .set_bit()
            .snb()
            .bits(sector as u8)
            .strt()
            .set_bit()
        });
        self.flash.wait_ready();

        self.flash.nscr().modify(|_, w| w.ser().clear_bit());

        self.flash.check_error()
    }

    /// Program bytes with offset into flash memory
    ///
    /// This method always issues writes with an alignment and size of 128 bits
    /// by padding the first and last writes with 0xFF if needed.
    pub fn program<'a, I>(&mut self, offset: u32, bytes: I) -> Result<(), Error>
    where
        I: Iterator<Item = &'a u8>,
    {
        self.flash.wait_ready();
        // Unlock programming and ensure it is locked again on drop
        let _request = self.flash.programming_request()?;

        // prepend padding to align the offset to the write_size
        let padding = offset as usize % USER_FLASH_MAX_WRITE_SIZE;
        let mut bytes = (0..padding).map(|_| &0xFFu8).chain(bytes).peekable();
        let offset = offset - padding as u32;

        // pointer and offset in 32-bit words
        let ptr = self.region().base_address() as *mut u32;
        let mut offset = offset as usize / core::mem::size_of::<u32>();
        let write_size =
            USER_FLASH_MAX_WRITE_SIZE / core::mem::size_of::<u32>();

        // Clear all the error flags due to previous programming/erase
        self.flash.clear_error_flags();

        // Iterate over buffers of size `write_size`
        while bytes.peek().is_some() {
            // Ensure that the write to the CR register (device memory) is
            // committed *before* we write to flash (normal memory). This
            // prevents ProgrammingSequence errors
            fence(Ordering::SeqCst);

            for _ in 0..write_size {
                let b0 = bytes.next().unwrap_or(&0xFF);
                let b1 = bytes.next().unwrap_or(&0xFF);
                let b2 = bytes.next().unwrap_or(&0xFF);
                let b3 = bytes.next().unwrap_or(&0xFF);

                let word = u32::from_le_bytes([*b0, *b1, *b2, *b3]);
                unsafe {
                    ptr::write_volatile(ptr.add(offset), word);
                }
                offset += 1;
            }

            // Ensure that the writes to flash (normal memory) are committed
            // before we start polling the status register. The write will have
            // already started once the last word was written, so this fence
            // does not cause any additional slowdown
            fence(Ordering::SeqCst);

            self.flash.wait_ready();
            self.flash.check_error()?;
        }

        Ok(())
    }
}

impl NorFlash for UnlockedUserFlash<'_> {
    const WRITE_SIZE: usize = super::USER_FLASH_MAX_WRITE_SIZE;
    const ERASE_SIZE: usize = super::SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(|e| match e {
            NorFlashErrorKind::NotAligned => Error::NotAligned,
            NorFlashErrorKind::OutOfBounds => Error::OutOfBounds,
            _ => Error::Other,
        })?;

        let mut current = from;
        if to > from {
            return Err(Error::OutOfBounds);
        }

        for (bank, sector) in self.region().bank_sector_iter() {
            if sector.contains(current) {
                self.erase_sector(bank, sector.number as usize)?;
                current += super::SECTOR_SIZE as u32;
            }
            if current >= to {
                break;
            }
        }

        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !(offset as usize).is_multiple_of(Self::WRITE_SIZE)
            || !bytes.len().is_multiple_of(Self::WRITE_SIZE)
        {
            return Err(Error::NotAligned);
        }
        if offset as usize + bytes.len() > self.region().size() {
            return Err(Error::OutOfBounds);
        }
        self.program(offset, bytes.iter())
    }
}

impl<'a> UnlockedOtpData<'a> {
    /// Programs a single 16-bit word into an OTP `slot`.
    ///
    /// Returns an error if the hardware signals a programming fault.
    /// Panics if `slot >= OTP_NUM_SLOTS`.
    pub fn program_value(
        &mut self,
        slot: usize,
        val: u16,
    ) -> Result<u16, Error> {
        assert!(slot < OTP_NUM_SLOTS);

        self.flash.wait_ready();

        if self.is_otp_block_locked(self.region().block(slot)) {
            return Err(Error::WriteProtection);
        }

        {
            // Unlock programming and ensure it is locked again on drop (at the end of this block)
            let _request = self.flash.programming_request()?;

            // Clear all the error flags due to previous programming/erase
            self.flash.clear_error_flags();

            // Ensure that the write to the CR register (device memory) is
            // committed *before* we write to flash (normal memory). This
            // prevents ProgrammingSequence errors
            fence(Ordering::SeqCst);

            let ptr = self.region().base_address() as *mut u16;
            unsafe { core::ptr::write_volatile(ptr.add(slot), val) }

            // Ensure that the writes to flash (normal memory) are committed
            // before we start polling the status register. The write will have
            // already started once the last word was written, so this fence
            // does not cause any additional slowdown
            fence(Ordering::SeqCst);

            self.flash.wait_ready();
            self.flash.check_error()?;
        }
        self.read_value(slot)
    }

    /// Programs consecutive 16-bit OTP words starting at `start_slot`.
    ///
    /// Each word in `words` is written to `start_slot + n` and verified before
    /// the next word is written.  Returns an error on the first hardware fault.
    /// Panics if `start_slot >= OTP_NUM_SLOTS` or any derived slot is out of range.
    pub fn program_sequence<I>(
        &mut self,
        start_slot: usize,
        words: I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a u16>,
    {
        assert!(start_slot < OTP_NUM_SLOTS);

        let base_ptr = self.region().base_address() as *mut u16;
        self.flash.wait_ready();
        {
            // Unlock programming and ensure it is locked again on drop (at the end of this block)
            let _request = self.flash.programming_request()?;

            // Clear all the error flags due to previous programming/erase
            self.flash.clear_error_flags();

            for (slot, word) in words.enumerate() {
                if self
                    .is_otp_block_locked(self.region().block(start_slot + slot))
                {
                    return Err(Error::WriteProtection);
                }

                // Ensure that the write to the CR register (device memory) is
                // committed *before* we write to flash (normal memory). This
                // prevents ProgrammingSequence errors
                fence(Ordering::SeqCst);

                let ptr = unsafe { base_ptr.add(start_slot + slot) };

                assert!(ptr < self.region().region_end_address() as *mut u16);
                unsafe { core::ptr::write_volatile(ptr, *word) }

                // Ensure that the writes to flash (normal memory) are committed
                // before we start polling the status register. The write will have
                // already started once the last word was written, so this fence
                // does not cause any additional slowdown
                fence(Ordering::SeqCst);

                self.flash.wait_ready();
                self.flash.check_error()?;
            }
        }

        Ok(())
    }

    /// Programs raw bytes into the OTP region starting at byte `offset`.
    ///
    /// Returns an error on the first hardware fault.
    /// Panics if `offset` is unaligned or any derived address exceeds the OTP region.
    pub fn program_bytes<I>(
        &mut self,
        offset: usize,
        bytes: I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a u8>,
    {
        let base_ptr = self.region().base_address() as *mut u8;
        let start_ptr = unsafe { base_ptr.add(offset) } as *mut u16;
        assert!(start_ptr.is_aligned());

        let mut bytes = bytes.peekable();
        self.flash.wait_ready();
        {
            // Unlock programming and ensure it is locked again on drop (at the end of this block)
            let _request = self.flash.programming_request()?;

            // Clear all the error flags due to previous programming/erase
            self.flash.clear_error_flags();

            let mut count = 0usize;

            while bytes.peek().is_some() {
                let slot = offset / 2 + count;
                if self.is_otp_block_locked(self.region().block(slot)) {
                    return Err(Error::WriteProtection);
                }

                // Ensure that the write to the CR register (device memory) is
                // committed *before* we write to flash (normal memory). This
                // prevents ProgrammingSequence errors
                fence(Ordering::SeqCst);

                let b0 = bytes.next().unwrap_or(&0xFF);
                let b1 = bytes.next().unwrap_or(&0xFF);
                let word = u16::from_le_bytes([*b0, *b1]);

                let ptr = unsafe { start_ptr.add(count) };
                assert!(ptr < self.region().region_end_address() as *mut u16);

                unsafe { core::ptr::write_volatile(ptr, word) }

                // Ensure that the writes to flash (normal memory) are committed
                // before we start polling the status register. The write will have
                // already started once the last word was written, so this fence
                // does not cause any additional slowdown
                fence(Ordering::SeqCst);

                self.flash.wait_ready();
                self.flash.check_error()?;
                count += 1;
            }
        }

        Ok(())
    }

    fn is_otp_block_locked(&self, block: usize) -> bool {
        let bits = self.flash.otpblr_cur().read().lockbl().bits();
        (bits >> block) & 1 != 0
    }
}

impl<'a> UnlockedOptionBytes<'a> {
    /// Applies option-byte changes via a closure and starts the option-byte
    /// programming sequence (`OPTSTRT`).
    pub fn modify<F>(&mut self, f: F) -> Result<(), Error>
    where
        F: FnOnce(&mut FLASH),
    {
        self.flash.prepare_operation()?;

        // Ensure that the write to the CR register (device memory) is
        // committed *before* we write to flash (normal memory). This
        // prevents ProgrammingSequence errors
        fence(Ordering::SeqCst);

        f(&mut self.flash.flash);

        self.flash.optcr().modify(|_, w| w.optstrt().set_bit());

        // Ensure that the writes to flash (normal memory) are committed
        // before we start polling the status register. The write will have
        // already started once the last word was written, so this fence
        // does not cause any additional slowdown
        fence(Ordering::SeqCst);

        self.flash.wait_ready();
        self.flash.check_error()?;

        Ok(())
    }
}
