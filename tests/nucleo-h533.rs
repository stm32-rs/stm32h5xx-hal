#![no_std]
#![no_main]

#[path = "../examples/utilities/mod.rs"]
mod utils;

mod common;

use fugit::HertzU32;
use hal::stm32;
use stm32h5xx_hal::{self as hal, gpio};

pub const F_SYS: HertzU32 = HertzU32::MHz(16);
pub const CYCLES_PER_US: u32 = F_SYS.raw() / 1_000_000;

use crate::common::is_pax_low;
use embedded_hal::delay::DelayNs;
use fugit::RateExtU32;
use stm32h5xx_hal::{delay::Delay, gpio::GpioExt, pwr::PwrExt, rcc::RccExt};

#[embedded_test::tests]
mod tests {
    use stm32h5xx_hal::gpio::{PinExt, Pull};

    #[test]
    fn gpio_resistors() {
        use super::*;

        let (gpioa, mut delay) = init();

        let mut pin = gpioa.pa8.into_pull_down_input();
        let pin_num = pin.pin_id();

        delay.delay_ms(1); // Give the pin plenty of time to go low
        assert!(is_pax_low(pin_num));

        pin.set_internal_resistor(Pull::Up);
        delay.delay_ms(1); // Give the pin plenty of time to go high
        assert!(!is_pax_low(pin_num));
    }
    #[test]
    fn gpio_push_pull() {
        use super::*;

        let (gpioa, mut delay) = init();

        let mut pin = gpioa.pa8.into_push_pull_output();
        let pin_num = pin.pin_id();

        pin.set_high();
        delay.delay_ms(1); // Give the pin plenty of time to go high
        assert!(!is_pax_low(pin_num));

        pin.set_low();
        delay.delay_ms(1); // Give the pin plenty of time to go low
        assert!(is_pax_low(pin_num));
    }

    #[test]
    fn gpio_open_drain() {
        use super::*;

        let (gpioa, mut delay) = init();

        let mut pin = gpioa.pa8.into_open_drain_output().internal_pull_up(true);
        let pin_num = pin.pin_id();

        pin.set_high();
        delay.delay_ms(1); // Give the pin plenty of time to go high
        assert!(pin.is_high());
        assert!(!is_pax_low(pin_num));

        pin.set_low();
        delay.delay_ms(1); // Give the pin plenty of time to go low
        assert!(pin.is_low());
        assert!(is_pax_low(pin_num));
    }
}

fn init() -> (gpio::gpioa::Parts, Delay) {
    utils::logger::init();
    let cp = stm32::CorePeripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(250u32.MHz()).freeze(pwrcfg, &dp.SBS);

    let delay = Delay::new(cp.SYST, &ccdr.clocks);
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    (gpioa, delay)
}
