#![no_main]
#![no_std]

mod utilities;

use cortex_m_rt::entry;
use stm32h5xx_hal::pac;

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // TODO: Power/clock config is required before blinky can... blink.

    dp.GPIOA.moder.write(|w| w.mode5().variant(1)); // output
    dp.GPIOA.pupdr.write(|w| w.pupd5().variant(1)); // pull-up

    // dp.GPIOA.odr.write(|w| w.od5().set_bit());

    loop {
        dp.GPIOA.odr.write(|w| w.od5().clear_bit());
        for _ in 0..1_000_0 {
            cortex_m::asm::nop();
        }
        dp.GPIOA.odr.write(|w| w.od5().set_bit());
        for _ in 0..1_000_0 {
            cortex_m::asm::nop();
        }
    }
}
