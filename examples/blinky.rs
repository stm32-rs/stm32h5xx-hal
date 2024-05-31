#![no_main]
#![no_std]

mod utilities;

use cortex_m_rt::entry;
use stm32h5xx_hal::pac;

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let dp = pac::Peripherals::take().unwrap();

    // TODO: Power/clock config is required before blinky can... blink.

    dp.GPIOA.moder().write(|w| w.mode5().output()); // output
    dp.GPIOA.pupdr().write(|w| w.pupd5().pull_up()); // pull-up

    // dp.GPIOA.odr.write(|w| w.od5().set_bit());

    loop {
        dp.GPIOA.odr().write(|w| w.od5().low());
        for _ in 0..10_000 {
            cortex_m::asm::nop();
        }
        dp.GPIOA.odr().write(|w| w.od5().high());
        for _ in 0..10_000 {
            cortex_m::asm::nop();
        }
    }
}
