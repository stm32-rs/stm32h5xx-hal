#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use log::info;
use stm32h5xx_hal::{
    dwt::{ClockDuration, DwtExt},
    pac,
    prelude::*,
};

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let mut led = gpioa.pa5.into_push_pull_output();

    // Create a delay abstraction based on DWT cycle counter
    let dwt = cp.DWT.constrain(cp.DCB, &ccdr.clocks);
    let mut delay = dwt.delay();

    // Create a stopwatch for maximum 9 laps
    // Note: it starts immediately
    let mut lap_times = [0u32; 10];
    let mut sw = dwt.stopwatch(&mut lap_times);
    loop {
        // On for 1s, off for 1s.
        led.set_high();
        delay.delay_ms(1000);
        sw.lap();
        led.set_low();
        delay.delay_ms(900);

        // Also you can measure with almost clock precision
        let cd: ClockDuration = dwt.measure(|| delay.delay_ms(100));
        info!("Ticks: {}", cd.as_ticks()); // Should return 250MHz * 0.1s as u32
        info!("Secs (f32): {}", cd.as_secs_f32()); // Should return ~0.1s as a f32
        info!("Secs (f64): {}", cd.as_secs_f64()); // Should return ~0.1s as a f64
        info!("Nanos: {}", cd.as_nanos()); // Should return 100000000ns as a u64

        sw.lap();

        // Get all the lap times
        {
            let mut lap = 1;
            while let Some(lap_time) = sw.lap_time(lap) {
                let _t = lap_time.as_secs_f64();
                lap += 1;
            }
        }

        // Reset stopwatch
        sw.reset();
    }
}
