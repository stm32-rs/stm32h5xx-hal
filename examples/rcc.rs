#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use log::info;

use cortex_m_rt::entry;
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
    let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    info!("");
    info!("stm32h5xx-hal example - RCC");
    info!("");

    // HCLK
    info!("hclk = {} Hz", ccdr.clocks.hclk().raw());
    assert_eq!(ccdr.clocks.hclk().raw(), 250_000_000);

    // SYS_CK
    info!("sys_ck = {} Hz", ccdr.clocks.sys_ck().raw());
    assert_eq!(ccdr.clocks.sys_ck().raw(), 250_000_000);

    loop {
        cortex_m::asm::nop()
    }
}
