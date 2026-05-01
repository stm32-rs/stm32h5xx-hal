//! Example of Flash erasing, writing and reading
//!
//! Tested on the following parts:
//! - STM32H503 (NUCLEO-H503RB)

// #![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::debug;
use stm32h5xx_hal::{pac, prelude::*};

#[macro_use]
mod utilities;
use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let _ = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    info!("");
    info!("stm32h5xx-hal example - Flash Option Bytes");
    info!("");

    let mut flash = dp.FLASH.flash();

    info!(
        "WWDG_SW: {}",
        flash.optsr_cur().read().wwdg_sw().bit_is_set()
    );

    // Attempting to modify an option byte without unlocking should have no effect
    flash.optsr_prg().modify(|_, w| w.wwdg_sw().clear_bit());

    info!(
        "WWDG_SW: {}",
        flash.optsr_cur().read().wwdg_sw().bit_is_set()
    );

    // Unlock option bytes, modify WWDG_SW and relock
    {
        let mut option_bytes = flash.unlock_option_bytes().unwrap();

        option_bytes
            .modify(|f| {
                f.optsr_prg().modify(|_, w| w.wwdg_sw().clear_bit());
            })
            .unwrap();
    }

    info!(
        "WWDG_SW: {}",
        flash.optsr_cur().read().wwdg_sw().bit_is_set()
    );

    // Unlock option bytes, return WWDG_SW to its original state and relock
    {
        let mut option_bytes = flash.unlock_option_bytes().unwrap();

        option_bytes
            .modify(|f| {
                f.optsr_prg().modify(|_, w| w.wwdg_sw().set_bit());
            })
            .unwrap();
    }

    info!(
        "WWDG_SW: {}",
        flash.optsr_cur().read().wwdg_sw().bit_is_set()
    );

    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
