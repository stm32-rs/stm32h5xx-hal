#![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use core::mem::MaybeUninit;

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use stm32h5xx_hal::{
    pac,
    prelude::*,
    spi::{self, Config as SpiConfig, Spi, dma::DuplexDmaTransfer},
};

static mut SOURCE_BYTES: MaybeUninit<[u8; 40]> = MaybeUninit::uninit();
static mut DEST_BYTES: MaybeUninit<[u8; 40]> = MaybeUninit::zeroed();

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

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    log::info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    log::info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(192.MHz())
        .pll1_q_ck(64.MHz())
        .freeze(pwrcfg, &dp.SBS);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();

    log::info!("stm32h5xx-hal example - SPI DMA");

    // Initialise the SPI peripheral.
    let mut spi: Spi<_, u8> = dp.SPI2.spi(
        (sck, miso, mosi),
        SpiConfig::new(spi::MODE_0),
        1.MHz(),
        ccdr.peripheral.SPI2,
        &ccdr.clocks,
    );

    let (source_buf, dest_buf) = u8_to_u8_sequential();

    let channels = dp.GPDMA1.channels(ccdr.peripheral.GPDMA1);
    let tx_ch = channels.0;
    let rx_ch = channels.1;

    let mut transfer = DuplexDmaTransfer::new(&mut spi, tx_ch, rx_ch, source_buf, dest_buf);
    transfer.start().unwrap();
    transfer.wait_for_complete().unwrap();
    let (_, _, source_buf, dest_buf) = transfer.free().unwrap();

    assert_eq!(source_buf, dest_buf);

    log::info!("Success!");
    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
