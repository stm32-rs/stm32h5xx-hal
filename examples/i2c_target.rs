#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;
use stm32h5xx_hal::{
    i2c::{TargetConfig, TargetListenEvent, Targetable},
    pac,
    prelude::*,
};

use cortex_m_rt::entry;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    log::set_max_level(log::LevelFilter::Debug);
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

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb5.into_alternate_open_drain();
    let sda = gpiob.pb3.into_alternate_open_drain();

    info!("");
    info!("stm32h5xx-hal example - I2C Target");
    info!("");

    let own_addr: u16 = 0x18;
    let bus_freq_hz: u32 = 100_000;
    let mut i2c = dp.I2C2.i2c_target_only(
        (scl, sda),
        TargetConfig::new(own_addr, bus_freq_hz),
        ccdr.peripheral.I2C2,
        &ccdr.clocks,
    );

    i2c.enable_listen_event(TargetListenEvent::PrimaryAddress);
    let mut buf = [0u8; 2];

    loop {
        let count = i2c.wait_for_target_read(&mut buf).unwrap();
        info!("Read {count} bytes: {:X?}", &buf[..count]);

        let count = i2c.wait_for_target_write(b"Hello").unwrap();
        info!("Wrote {count} bytes");

        i2c.wait_for_stop().unwrap();
        info!("==========");
    }
}
