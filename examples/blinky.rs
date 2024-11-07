#![no_main]
#![no_std]

mod utilities;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use fugit::SecsDurationU32;
use stm32h5xx_hal::{delay::Delay, pac, prelude::*, rcc::ResetEnable};

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

    ccdr.peripheral.GPIOA.enable();

    dp.GPIOA.moder().write(|w| w.mode5().output()); // output
    dp.GPIOA.pupdr().write(|w| w.pupd5().pull_up()); // pull-up

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = SecsDurationU32::secs(1).to_millis();

    loop {
        dp.GPIOA.odr().write(|w| w.od5().low());
        delay.delay_ms(duration);
        log::info!("Off");
        dp.GPIOA.odr().write(|w| w.od5().high());
        delay.delay_ms(duration);
        log::info!("On");
    }
}
