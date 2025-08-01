// #![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use core::mem::MaybeUninit;

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use stm32h5xx_hal::{
    gpdma::{config::transform::*, DmaConfig, DmaTransfer},
    pac,
    prelude::*,
};

static mut SOURCE_BYTES: MaybeUninit<[u8; 40]> = MaybeUninit::uninit();
static mut DEST_BYTES: MaybeUninit<[u8; 40]> = MaybeUninit::zeroed();
static mut DEST_HALF_WORDS: MaybeUninit<[u16; 20]> = MaybeUninit::uninit();
static mut SOURCE_WORDS: MaybeUninit<[u32; 10]> = MaybeUninit::uninit();
static mut DEST_WORDS: MaybeUninit<[u32; 10]> = MaybeUninit::uninit();

fn u8_to_u8_sequential() -> (&'static [u8; 40], &'static mut [u8; 40]) {
    let buf: &mut [MaybeUninit<u8>; 40] = unsafe {
        &mut *(core::ptr::addr_of_mut!(SOURCE_BYTES)
            as *mut [MaybeUninit<u8>; 40])
    };

    for (i, value) in buf.iter_mut().enumerate() {
        unsafe {
            value.as_mut_ptr().write(i as u8);
        }
    }
    #[allow(static_mut_refs)] // TODO: Fix this
    let src = unsafe { SOURCE_BYTES.assume_init_ref() };

    let dest =
        unsafe { (*core::ptr::addr_of_mut!(DEST_BYTES)).assume_init_mut() };

    dest.fill(0);

    (src, dest)
}

fn u32_to_u32_transform() -> (&'static [u32; 10], &'static mut [u32; 10]) {
    let buf: &mut [MaybeUninit<u32>; 10] = unsafe {
        &mut *(core::ptr::addr_of_mut!(SOURCE_WORDS)
            as *mut [MaybeUninit<u32>; 10])
    };

    buf.fill(MaybeUninit::new(0x12345678));

    #[allow(static_mut_refs)] // TODO: Fix this
    let src = unsafe { SOURCE_WORDS.assume_init_ref() };

    let dest =
        unsafe { (*core::ptr::addr_of_mut!(DEST_WORDS)).assume_init_mut() };

    dest.fill(0);
    (src, dest)
}

fn u32_to_u16_truncate() -> (&'static [u32; 10], &'static mut [u16; 20]) {
    let buf: &mut [MaybeUninit<u32>; 10] = unsafe {
        &mut *(core::ptr::addr_of_mut!(SOURCE_WORDS)
            as *mut [MaybeUninit<u32>; 10])
    };

    buf.fill(MaybeUninit::new(0x12345678));

    #[allow(static_mut_refs)] // TODO: Fix this
    let src = unsafe { SOURCE_WORDS.assume_init_ref() };

    let dest = unsafe {
        (*core::ptr::addr_of_mut!(DEST_HALF_WORDS)).assume_init_mut()
    };

    dest.fill(0);
    (src, dest)
}

fn u32_to_u8_unpack() -> (&'static [u32; 10], &'static mut [u8; 40]) {
    let buf: &mut [MaybeUninit<u32>; 10] = unsafe {
        &mut *(core::ptr::addr_of_mut!(SOURCE_WORDS)
            as *mut [MaybeUninit<u32>; 10])
    };

    buf.fill(MaybeUninit::new(0x12345678));

    #[allow(static_mut_refs)] // TODO: Fix this
    let src = unsafe { SOURCE_WORDS.assume_init_ref() };

    let dest =
        unsafe { (*core::ptr::addr_of_mut!(DEST_BYTES)).assume_init_mut() };

    dest.fill(0);
    (src, dest)
}

fn u8_to_u32_pack() -> (&'static [u8; 40], &'static mut [u32; 10]) {
    let buf: &mut [MaybeUninit<u8>; 40] = unsafe {
        &mut *(core::ptr::addr_of_mut!(SOURCE_BYTES)
            as *mut [MaybeUninit<u8>; 40])
    };

    for chunk in buf.chunks_mut(4) {
        unsafe {
            chunk[0].as_mut_ptr().write(0x78);
            chunk[1].as_mut_ptr().write(0x56);
            chunk[2].as_mut_ptr().write(0x34);
            chunk[3].as_mut_ptr().write(0x12);
        }
    }

    #[allow(static_mut_refs)] // TODO: Fix this
    let src = unsafe { SOURCE_BYTES.assume_init_ref() };

    let dest =
        unsafe { (*core::ptr::addr_of_mut!(DEST_WORDS)).assume_init_mut() };

    dest.fill(0);
    (src, dest)
}

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    let channels = dp.GPDMA1.channels(ccdr.peripheral.GPDMA1);

    let (source_buf, dest_buf) = u8_to_u8_sequential();
    let source_copy = unsafe { &*(source_buf.as_ptr() as *const [u8; 40]) };
    let dest_copy = unsafe { &*(dest_buf.as_ptr() as *const [u8; 40]) };

    let channel = channels.0;
    let config = DmaConfig::new();
    let transfer =
        DmaTransfer::memory_to_memory(config, &channel, source_buf, dest_buf);
    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    assert_eq!(source_copy, dest_copy);

    let (source_buf, dest_buf) = u32_to_u32_transform();
    let dest_copy = unsafe { &*(dest_buf.as_ptr() as *const [u32; 10]) };
    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder()
            .swap_destination_half_words()
            .swap_destination_half_word_byte_order(),
    );

    let transfer =
        DmaTransfer::memory_to_memory(config, &channel, source_buf, dest_buf);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x78563412; 10];
    assert_eq!(expected, *dest_copy);

    let (source_buf, dest_buf) = u32_to_u16_truncate();
    let dest_copy = unsafe { &*(dest_buf.as_ptr() as *const [u16; 20]) };
    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder().left_align_right_truncate(),
    );
    let transfer =
        DmaTransfer::memory_to_memory(config, &channel, source_buf, dest_buf);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x1234; 10];
    assert_eq!(expected, (*dest_copy)[0..10]);

    let (source_buf, dest_buf) = u32_to_u8_unpack();
    let dest_copy = unsafe { &*(dest_buf.as_ptr() as *const [u8; 40]) };
    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().unpack());
    let transfer =
        DmaTransfer::memory_to_memory(config, &channel, source_buf, dest_buf);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x78, 0x56, 0x34, 0x12];
    assert_eq!(expected, (*dest_copy)[0..4]);
    assert_eq!(expected, (*dest_copy)[36..40]);

    let (source_buf, dest_buf) = u8_to_u32_pack();
    let dest_copy = unsafe { &*(dest_buf.as_ptr() as *const [u32; 10]) };
    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().pack());
    let transfer =
        DmaTransfer::memory_to_memory(config, &channel, source_buf, dest_buf);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x12345678; 10];
    assert_eq!(expected, (*dest_copy));
    assert_eq!(expected, (*dest_copy));

    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
