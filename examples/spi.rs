#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
mod utilities;
use embedded_hal::{delay::DelayNs, spi::SpiBus};
use stm32h5xx_hal::{delay::Delay, pac, prelude::*, spi, time::MilliSeconds};

use log::info;

const TEST_STR: &[u8] = b"TEST SPI TESTING, TESTING, TESTING";

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
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

    info!("");
    info!("stm32h5xx-hal example - SPI");
    info!("");

    // Initialise the SPI peripheral.
    let mut spi = dp.SPI2.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        1.MHz(),
        ccdr.peripheral.SPI2,
        &ccdr.clocks,
    );

    // Write short fixed data
    spi.write(&[0x11u8]).unwrap();
    spi.write(&[0x11u8, 0x22, 0x33]).unwrap();

    info!("Transfer starting");
    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = MilliSeconds::secs(1).to_millis();
    // Echo what is received on the SPI
    let write = TEST_STR;
    let read = &mut [0u8; TEST_STR.len()];
    loop {
        spi.transfer(read, write).unwrap();
        delay.delay_ms(duration);
    }
}
