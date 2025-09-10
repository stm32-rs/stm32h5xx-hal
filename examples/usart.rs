// #![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
mod utilities;
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};
use stm32h5xx_hal::{delay::Delay, pac, prelude::*, time::MilliSeconds};

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
        .sys_ck(192.MHz())
        .pll1_q_ck(64.MHz())
        .freeze(pwrcfg, &dp.SBS);

    // Acquire the GPIOA peripheral. This also enables the clock for
    // GPIOA in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let rx = gpioa.pa11.into_alternate();
    let tx = gpioa.pa12.into_alternate();

    info!("");
    info!("stm32h5xx-hal example - USART");
    info!("");

    // Initialise the USART peripheral.
    let mut serial = dp
        .USART2
        .serial((tx, rx), 115200.Hz(), ccdr.peripheral.USART2, &ccdr.clocks)
        .unwrap();

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = MilliSeconds::secs(1).to_millis();
    // Echo what is received on the USART
    let mut write = 'A';
    let read = &mut [0u8; 1];
    loop {
        serial.write(&[write as u8]).unwrap();
        serial.read(read).unwrap();
        info!("Received: {}", read[0] as char);
        write = (write as u8).wrapping_add(1) as char;
        delay.delay_ms(duration);
    }
}
