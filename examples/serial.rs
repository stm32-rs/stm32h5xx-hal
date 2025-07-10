#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
mod utilities;
use embedded_hal::delay::DelayNs;
use embedded_io::Write;
use fugit::SecsDurationU32;
use stm32h5xx_hal::{delay::Delay, pac, prelude::*};

use log::info;

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
        .sys_ck(100.MHz())
        .pll1_q_ck(50.MHz())
        .freeze(pwrcfg, &dp.SBS);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let tx = gpiob.pb14.into_alternate();
    let rx = gpiob.pb15.into_alternate();

    info!("");
    info!("stm32h5xx-hal example - Serial (USART)");
    info!("");

    // Initialise the SPI peripheral.
    let mut serial = dp
        .USART1
        .serial((tx, rx), 115_200.Hz(), ccdr.peripheral.USART1, &ccdr.clocks)
        .unwrap();

    // Write short fixed data using blocking embedded-io API
    serial.write(&[0x11u8]).unwrap();
    serial.write(&[0x11u8, 0x22, 0x33]).unwrap();

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = SecsDurationU32::secs(1).to_millis();

    loop {
        // Use embedded-io trait to write out entire longer string
        let write = b"TESTING TESTING";
        serial.write_all(write).unwrap();

        delay.delay_ms(duration);
    }
}
