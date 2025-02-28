#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;
use stm32h5xx_hal::{
    i2c::{TargetConfig, TargetEvent, TargetListenEvent, Targetable},
    pac,
    prelude::*,
};

use cortex_m_rt::entry;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    log::set_max_level(log::LevelFilter::Info);
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
    let mut i2c = dp
        .I2C2
        .i2c_target_only(
            (scl, sda),
            TargetConfig::new(own_addr, bus_freq_hz),
            ccdr.peripheral.I2C2,
            &ccdr.clocks,
        )
        .with_manual_ack_control();

    i2c.enable_listen_event(TargetListenEvent::PrimaryAddress);
    let mut buf = [0u8; 2];

    loop {
        let event = i2c.wait_for_event().unwrap();
        let result = match event {
            TargetEvent::TargetWrite { address: _ } => i2c.write(b"Hello"),
            TargetEvent::TargetRead { address: _ } => {
                let result = i2c.read(&mut buf);
                // An operation can be performed here while the clock is stretched low (ie.
                // calculating or fetching data)
                i2c.ack_transfer();
                result
            }
            TargetEvent::Stop => Ok(0),
        };

        match result {
            Err(error) => info!("Error: {event:?} - {error:?}"),
            Ok(count) => match event {
                TargetEvent::Stop => info!("=========="),
                _ => info!("{event:?} ({count})"),
            },
        };
    }
}
