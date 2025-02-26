#![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use fugit::SecsDurationU32;
use stm32h5xx_hal::{delay::Delay, pac, prelude::*};
use utilities::logger::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let mut led = gpioa.pa5.into_push_pull_output();

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = SecsDurationU32::secs(1).to_millis();

    loop {
        led.set_low();
        info!("Off");
        delay.delay_ms(duration);
        led.set_high();
        info!("On");
        delay.delay_ms(duration);
    }
}
