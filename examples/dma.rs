// #![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use cortex_m::singleton;
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

    log::info!("u8 to u8");
    let src =
        singleton!(: [u8; 40] = core::array::from_fn(|i| i as u8)).unwrap();

    let dest = singleton!(: [u8; 40] = [0u8; 40]).unwrap();

    let mut channel = channels.0;
    let config = DmaConfig::new();
    let mut transfer =
        DmaTransfer::memory_to_memory(config, &mut channel, src, dest);
    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let (src, dest) = transfer.free();
    assert_eq!(src, dest);

    log::info!("u32 to u32 with data transform");
    let src = singleton!(: [u32; 10] = [0x12345678u32; 10]).unwrap();
    let dest = singleton!(: [u32; 10] = [0u32; 10]).unwrap();

    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder()
            .swap_destination_half_words()
            .swap_destination_half_word_byte_order(),
    );

    let mut transfer =
        DmaTransfer::memory_to_memory(config, &mut channel, src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let (_, dest) = transfer.free();

    let expected = [0x78563412; 10];
    assert_eq!(expected, *dest);

    log::info!("u32 to u16 with truncate");
    let src = singleton!(: [u32; 10] = [0x12345678u32; 10]).unwrap();
    let dest = singleton!(: [u16; 20] = [0u16; 20]).unwrap();

    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder().left_align_right_truncate(),
    );
    let mut transfer =
        DmaTransfer::memory_to_memory(config, &mut channel, src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let (_, dest) = transfer.free();

    let expected = [0x1234; 10];
    assert_eq!(expected, (*dest)[0..10]);

    log::info!("u32 to u8 with unpack");
    let src = singleton!(: [u32; 10] = [0x12345678u32; 10]).unwrap();
    let dest = singleton!(: [u8; 40] = [0u8; 40]).unwrap();

    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().unpack());
    let mut transfer =
        DmaTransfer::memory_to_memory(config, &mut channel, src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let (_, dest) = transfer.free();
    let expected = [0x78, 0x56, 0x34, 0x12];
    assert_eq!(expected, (*dest)[0..4]);
    assert_eq!(expected, (*dest)[36..40]);

    log::info!("u8 to u32 with pack");
    let src = singleton!(: [u8; 40] = [0u8; 40]).unwrap();
    let dest = singleton!(: [u32; 10] = [0u32; 10]).unwrap();

    for chunk in src.chunks_mut(4) {
        chunk.copy_from_slice(&[0x78, 0x56, 0x34, 0x12]);
    }

    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().pack());
    let mut transfer =
        DmaTransfer::memory_to_memory(config, &mut channel, src, dest);

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let (_, dest) = transfer.free();

    let expected = [0x12345678; 10];
    assert_eq!(expected, *dest);
    assert_eq!(expected, *dest);

    log::info!("All tests passed!");
    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
