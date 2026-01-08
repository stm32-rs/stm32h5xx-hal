#![deny(warnings)]
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

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // Connect TX and RX pins together for this to work. On the Nucleo-H503 this is pins 35 & 37 on
    // CN10
    let rx = gpiob.pb15.into_alternate();
    let tx = gpiob.pb14.into_alternate();

    info!("");
    info!("stm32h5xx-hal example - USART");
    info!("");

    // Initialise the USART peripheral.
    let mut serial = dp
        .USART1
        .serial((tx, rx), 115200.Hz(), ccdr.peripheral.USART1, &ccdr.clocks)
        .unwrap();

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = MilliSeconds::secs(1).to_millis();
    // Echo what is received on the USART
    let read = &mut [0u8; 6];
    let mut count = 1;
    loop {
        // Note: the USART driver uses a FIFO, so as long as we keep the messages below the FIFO
        // size we won't miss any data.
        write!(serial, "Test {count}").unwrap();
        count = (count + 1) % 10;
        serial.read_exact(read).unwrap();
        let received = core::str::from_utf8(read).unwrap();
        info!("Received: {}", received);
        delay.delay_ms(duration);
    }
}
