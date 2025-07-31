// #![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use stm32h5xx_hal::{
    gpdma::{config::transform::*, DmaConfig, DmaTransfer},
    pac,
    prelude::*,
};

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

    // u8 to u8
    log::info!("u8 to u8 transfer");
    let src: [u8; 40] = core::array::from_fn(|i| i as u8);
    let dest = &mut [0u8; 40];

    let channel = channels.0;
    let config = DmaConfig::new();
    let transfer = DmaTransfer::memory_to_memory(config, &channel, &src, dest);
    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    assert_eq!(src, *dest, "u8 to u8 transfer failed");

    // u32 to u32 with data transform
    log::info!("u32 to u32 with data transform");
    let src = [0x12345678u32; 10];
    let dest = &mut [0u32; 10];
    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder()
            .swap_destination_half_words()
            .swap_destination_half_word_byte_order(),
    );

    let transfer = DmaTransfer::memory_to_memory(config, &channel, &src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    assert_eq!(
        [0x78563412; 10], *dest,
        "u32 to u32 with data transform failed"
    );

    // u32 to u16 with truncate
    log::info!("u32 to u16 with truncate");
    let src = [0x12345678u32; 10];
    let dest = &mut [0u16; 20];
    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder().left_align_right_truncate(),
    );
    let transfer = DmaTransfer::memory_to_memory(config, &channel, &src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    assert_eq!(
        [0x1234; 10],
        (*dest)[0..10],
        "u32 to u16 with truncate failed"
    );
    assert_eq!([0; 10], (*dest)[10..20], "u32 to u16 with truncate failed");

    // u32 to u8 with unpack
    log::info!("u32 to u8 with unpack");
    let src = [0x12345678u32; 10];
    let dest = &mut [0u8; 40];
    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().unpack());
    let transfer = DmaTransfer::memory_to_memory(config, &channel, &src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x78, 0x56, 0x34, 0x12];
    assert_eq!(expected, (*dest)[0..4], "u32 to u8 unpack failed");
    assert_eq!(expected, (*dest)[36..40], "u32 to u8 unpack failed");

    // u8 to u32 with pack
    log::info!("u8 to u32 with pack");
    let mut src = [0u8; 40];
    let dest = &mut [0u32; 10];
    for chunk in src.chunks_mut(4) {
        chunk.copy_from_slice(&[0x78, 0x56, 0x34, 0x12]);
    }
    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().pack());
    let transfer = DmaTransfer::memory_to_memory(config, &channel, &src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    assert_eq!([0x12345678; 10], (*dest), "u8 to u32 with pack failed");

    log::info!("All tests passed!");

    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
