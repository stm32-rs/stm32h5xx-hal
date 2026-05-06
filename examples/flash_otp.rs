//! Example of OTP reading and programming.
//!
//! // WARNING: Programming OTP is irreversible. OTP programming is disabled by default in this
//! example to prevent accidental programming during testing. Modify the code to enable
//! (see note below)
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
    info!("stm32h5xx-hal example - Flash erase, write and read");
    info!("");

    let mut flash = dp.FLASH.flash();

    let mut otp = flash.otp_data();
    info!("Reading data slot 0");

    // Reading unprogrammed OTP generates an ECC error, which triggers an NMI. The NMI handler
    // is defined below and clears the ECC error flag, allowing the read to complete successfully
    // and return 0xFFFF for unprogrammed slots.
    let data = otp.read_value(0).unwrap();
    info!("OTP data slot 0: {:4X}", data);

    // WARNING: Programming OTP is irreversible. This block will permanently modify OTP slot 0 on
    // this device. It is disabled by default to prevent accidental programming during testing.
    // Remove `#[cfg(false)]` to enable OTP programming.
    #[cfg(false)]
    {
        let mut unlocked_otp = otp.unlocked();
        info!("Writing 0xBEEF to data slot 0");
        unlocked_otp.program_value(0, 0xBEEF).unwrap();

        // OTP locks when unlocked_otp goes out of scope
    }

    let data = otp.read_value(0).unwrap();
    info!("OTP data slot 0: {:4X}", data);

    assert!(
        data == 0xBEEF,
        "OTP read value does not match programmed value"
    );

    loop {
        debug::exit(debug::EXIT_SUCCESS)
    }
}

// The NMI handler is used to catch ECC errors from OTP reads. When an ECC error occurs,
// the handler reads the ECC error details from the FLASH peripheral, logs the error and address,
// and clears the ECC error flag.
#[exception]
unsafe fn NonMaskableInt() {
    info!("NMI exception");
    let flash = pac::FLASH::steal();
    let eccdetr = flash.eccdetr().read();
    if eccdetr.eccd().bit_is_set() {
        let addr = eccdetr.addr_ecc().bits();
        info!("ECC error address: 0x{:08X}", addr);
        if eccdetr.otp_ecc().bit_is_set() {
            info!("ECC error detected in OTP");
        } else {
            info!("ECC error detected in flash");
        }

        // Clear ECC error flag
        flash.eccdetr().write(|w| w.eccd().set_bit());
    }
}
