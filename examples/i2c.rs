#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;
use embedded_hal::{delay::DelayNs, i2c::I2c};
use fugit::SecsDurationU32;
use stm32h5xx_hal::{delay::Delay, pac, prelude::*};

use cortex_m_rt::entry;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    log::set_max_level(log::LevelFilter::Debug);
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SBS);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);
    let duration = SecsDurationU32::secs(1).to_millis();

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb5.into_alternate_open_drain();
    let sda = gpiob.pb3.into_alternate_open_drain();

    info!("");
    info!("stm32h5xx-hal example - I2C");
    info!("");

    let mut i2c =
        dp.I2C2
            .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C2, &ccdr.clocks);

    // The STM32H503 NUCLEO board does not have any I2C peripherals, so put in the address of
    // whatever peripheral you connect
    let device_addr: u8 = 0x18;
    // This implements a typical 8-bit register read operation, writing the register address and
    // then issuing a repeat start to read the register value. Tweak these settings to read the
    // desired amount of bytes from the register address.
    let register_addr = 0x02;
    let write = &[register_addr];
    let mut read = [0u8; 1];

    loop {
        i2c.write_read(device_addr, write, &mut read).unwrap();
        info!("Read reg {register_addr}: {read:X?}");
        delay.delay_ms(duration);
    }
}
