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

// traits for read, write and erase methods
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

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
    info!("stm32h5xx-hal example - Flash erase, write and read");
    info!("");

    let mut flash = dp.FLASH.flash();

    let mut locked_flash = flash.user_flash();
    let (bank, sector) =
        locked_flash.region().bank_sector_iter().last().unwrap();

    // Erase data in last sector in flash
    {
        let mut f = locked_flash.unlocked();
        f.erase_sector(bank, sector.number as usize).unwrap();
    }

    let mut buf = [0xFFu8; 1024 * 2];
    let mut read = [0u8; 1024 * 2];
    locked_flash.read(sector.offset, &mut read).unwrap();
    assert_eq!(read, buf);

    // Fill up write buffer
    let mut count = 0;
    for b in buf.iter_mut() {
        *b = count;
        count += 1;
        if count >= 100 {
            count = 0;
        }
    }

    {
        let mut f = locked_flash.unlocked();
        f.write(sector.offset, &buf).unwrap();
    }

    locked_flash.read(sector.offset, &mut read).unwrap();
    assert_eq!(read, buf);

    info!("Successfully erased, written and read back flash data");

    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}
