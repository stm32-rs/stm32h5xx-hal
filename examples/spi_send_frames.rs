//! This example shows off the FrameTransaction mode of hardware chip select functionality.
//!
//! For more docs, see https://docs.rs/stm32h7xx-hal/latest/stm32h7xx_hal/spi/index.html
//!

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use cortex_m_semihosting::debug;
use embedded_hal::spi::{Operation, SpiDevice};
mod utilities;
use spi::Spi;
use stm32h5xx_hal::{
    pac,
    prelude::*,
    spi::{self, CommunicationMode},
};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    // let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(192.MHz())
        .pll1_q_ck(64.MHz())
        .freeze(&pwrcfg, &dp.SBS);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();
    // Because we want to use the hardware chip select, we need to provide that too
    let hcs = gpiob.pb4.into_alternate();

    info!("");
    info!("stm32h5xx-hal example - SPI Frame Transactions");
    info!("");

    // Initialise the SPI peripheral.
    let mut spi: Spi<_, u8> = dp.SPI2.spi(
        // Give ownership of the pins
        (sck, miso, mosi, hcs),
        // Create a config with the hardware chip select given
        spi::Config::new(spi::MODE_0)
            // Put 1 us idle time between every word sent
            .inter_word_delay(0.000001)
            // Specify that we use the hardware cs
            .hardware_cs(spi::HardwareCS {
                // See the docs of the HardwareCSMode to see what the different modes do
                mode: spi::HardwareCSMode::FrameTransaction,
                // Put 1 us between the CS being asserted and the first clock
                assertion_delay: 0.000001,
                // Our CS should be high when not active and low when asserted
                polarity: spi::Polarity::IdleHigh,
            })
            .communication_mode(CommunicationMode::SimplexTransmitter),
        1.MHz(),
        ccdr.peripheral.SPI2,
        &ccdr.clocks,
    );

    spi.write(&[0, 1, 2]).unwrap();
    spi.write(&[0, 1, 2, 3, 4, 5, 6]).unwrap();

    // Compose multiple operations into a single compound transfer using Operations
    let mut ops = [
        Operation::Write(&[0x11u8, 0x22, 0x33]),
        Operation::Write(&[0x44u8, 0x55, 0x66]),
    ];

    spi.transaction(&mut ops).unwrap();

    loop {
        debug::exit(debug::EXIT_SUCCESS);
    }
}
