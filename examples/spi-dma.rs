//! This example shows off the using the SPI with the DMA engine
//!
//! For more docs, see https://docs.rs/stm32h7xx-hal/latest/stm32h5xx_hal/spi/index.html
//!
// #![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use core::mem::MaybeUninit;

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use embedded_hal::delay::DelayNs;
use stm32h5xx_hal::{
    delay::Delay,
    pac,
    prelude::*,
    spi::{self, Config as SpiConfig},
    time::MilliSeconds,
};

static mut SOURCE_BYTES: MaybeUninit<[u8; 40]> = MaybeUninit::uninit();
static mut DEST_BYTES: MaybeUninit<[u8; 40]> = MaybeUninit::zeroed();

fn u8_buf_pair() -> (&'static [u8; 40], &'static mut [u8; 40]) {
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

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Select highest power mode for max possible clock frequency
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Configure system PLLs and clocks - choose a PLL1 output so 1MHz SPI clock can be exactly
    // derived from it
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(192.MHz())
        .pll1_q_ck(64.MHz())
        .freeze(pwrcfg, &dp.SBS);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // This example requires that MISO is connected to MOSI via a jumper (pins 28 and 26 on CN10
    // header on NUCLEO-H503RB)
    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();

    log::info!("stm32h5xx-hal example - SPI DMA");

    // Initialise the SPI peripheral.
    let spi = dp.SPI2.spi(
        (sck, miso, mosi),
        SpiConfig::new(spi::MODE_0),
        1.MHz(),
        ccdr.peripheral.SPI2,
        &ccdr.clocks,
    );

    let (source_buf, dest_buf) = u8_buf_pair();

    let channels = dp.GPDMA1.channels(ccdr.peripheral.GPDMA1);
    let tx_ch = channels.0;
    let rx_ch = channels.1;

    let mut spi = spi.use_dma_duplex(tx_ch, rx_ch);

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = MilliSeconds::secs(1).to_millis();

    loop {
        let (tx, rx) =
            spi.start_dma_duplex_transfer(dest_buf, source_buf).unwrap();

        tx.wait_for_transfer_complete().unwrap();
        rx.wait_for_transfer_complete().unwrap();

        spi.finish_transfer(Ok(())).unwrap();
        assert_eq!(source_buf, dest_buf);

        log::info!("Success!");
        delay.delay_ms(duration);
    }
}
