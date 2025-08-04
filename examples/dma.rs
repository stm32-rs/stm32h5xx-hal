// #![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use cortex_m::singleton;
use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use embedded_dma::{ReadBuffer, WriteBuffer};
use stm32h5xx_hal::{
    gpdma::{config::transform::*, DmaConfig, DmaTransfer, Word},
    pac,
    prelude::*,
};

// Buffer is used to manage a reference to a static buffer returned by the cortex_m::singleton!
// macro and which can be with the DmaTransfer API (which requires passing ReadBuffer and
// WriteBuffer implementations by value) and then used to access the buffer after the transfer has
// completed.
struct Buffer<T: Word + 'static, const N: usize> {
    data: &'static mut [T; N],
}

impl<T, const N: usize> Buffer<T, N>
where
    T: Word + 'static,
{
    fn new(data: &'static mut [T; N]) -> Self {
        Self { data }
    }
}

unsafe impl<T, const N: usize> ReadBuffer for &Buffer<T, N>
where
    T: Word + 'static,
{
    type Word = T;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.data.as_ptr(), N)
    }
}

unsafe impl<T, const N: usize> WriteBuffer for &mut Buffer<T, N>
where
    T: Word + 'static,
{
    type Word = T;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (self.data.as_mut_ptr(), N)
    }
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

    log::info!("u8 to u8");
    let src =
        singleton!(: [u8; 40] = core::array::from_fn(|i| i as u8)).unwrap();

    let src_buffer = Buffer::new(src);
    let dest = singleton!(: [u8; 40] = [0u8; 40]).unwrap();
    let mut dest_buffer = Buffer::new(dest);

    let mut channel = channels.0;
    let config = DmaConfig::new();
    let mut transfer = DmaTransfer::memory_to_memory(
        config,
        &mut channel,
        &src_buffer,
        &mut dest_buffer,
    );
    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    assert_eq!(src_buffer.data, dest_buffer.data);

    log::info!("u32 to u32 with data transform");
    let src = singleton!(: [u32; 10] = [0x12345678u32; 10]).unwrap();
    let dest = singleton!(: [u32; 10] = [0u32; 10]).unwrap();
    let mut dest_buffer = Buffer::new(dest);

    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder()
            .swap_destination_half_words()
            .swap_destination_half_word_byte_order(),
    );

    let mut transfer = DmaTransfer::memory_to_memory(
        config,
        &mut channel,
        src,
        &mut dest_buffer,
    );

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x78563412; 10];
    assert_eq!(expected, *dest_buffer.data);

    log::info!("u32 to u16 with truncate");
    let src = singleton!(: [u32; 10] = [0x12345678u32; 10]).unwrap();
    let dest = singleton!(: [u16; 20] = [0u16; 20]).unwrap();
    let mut dest_buffer = Buffer::new(dest);

    let config = DmaConfig::new().with_data_transform(
        DataTransform::builder().left_align_right_truncate(),
    );
    let mut transfer = DmaTransfer::memory_to_memory(
        config,
        &mut channel,
        src,
        &mut dest_buffer,
    );

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x1234; 10];
    assert_eq!(expected, (*dest_buffer.data)[0..10]);

    log::info!("u32 to u8 with unpack");
    let src = singleton!(: [u32; 10] = [0x12345678u32; 10]).unwrap();
    let dest = singleton!(: [u8; 40] = [0u8; 40]).unwrap();
    let mut dest_buffer = Buffer::new(dest);

    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().unpack());
    let mut transfer = DmaTransfer::memory_to_memory(
        config,
        &mut channel,
        src,
        &mut dest_buffer,
    );

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x78, 0x56, 0x34, 0x12];
    assert_eq!(expected, (*dest_buffer.data)[0..4]);
    assert_eq!(expected, (*dest_buffer.data)[36..40]);

    log::info!("u8 to u32 with pack");
    let src = singleton!(: [u8; 40] = [0u8; 40]).unwrap();
    let dest = singleton!(: [u32; 10] = [0u32; 10]).unwrap();
    let mut dest_buffer = Buffer::new(dest);

    for chunk in src.chunks_mut(4) {
        chunk.copy_from_slice(&[0x78, 0x56, 0x34, 0x12]);
    }

    let config =
        DmaConfig::new().with_data_transform(DataTransform::builder().pack());
    let mut transfer = DmaTransfer::memory_to_memory(
        config,
        &mut channel,
        src,
        &mut dest_buffer,
    );

    transfer.start().unwrap();
    transfer.wait_for_transfer_complete().unwrap();
    let expected = [0x12345678; 10];
    assert_eq!(expected, (*dest_buffer.data));
    assert_eq!(expected, (*dest_buffer.data));

    log::info!("All tests passed!");
    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
