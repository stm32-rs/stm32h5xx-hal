#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use cortex_m_rt::entry;
use log::info;
use stm32h5xx_hal::rcc;
use stm32h5xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(250.MHz())
        .pll2_strategy(rcc::PllConfigStrategy::Fractional)
        .pll2_p_ck(12_288_000.Hz())
        .pll2_q_ck(6_144_000.Hz())
        .pll2_r_ck(3_024_000.Hz())
        // pll2_p / 2 --> mco2
        .mco2_from_pll2_p_ck(7.MHz())
        .freeze(pwrcfg, &dp.SBS);

    // // Enable MCO2 output pin
    // let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    // let _mco2_pin = gpioc.pc9.into_alternate::<0>().speed(Speed::High);

    info!("");
    info!("stm32h5xx-hal example - Fractional PLL");
    info!("");

    // SYS_CK
    info!("sys_ck = {} Hz", ccdr.clocks.sys_ck().raw());
    assert_eq!(ccdr.clocks.sys_ck().raw(), 250_000_000);

    info!("pll2_p_ck = {}", ccdr.clocks.pll2().p_ck().unwrap());
    info!("pll2_q_ck = {}", ccdr.clocks.pll2().q_ck().unwrap());
    info!("pll2_r_ck = {}", ccdr.clocks.pll2().r_ck().unwrap());

    let _mco2_ck = ccdr.clocks.mco2_ck().unwrap().raw();

    loop {
        cortex_m::asm::nop()
    }
}
