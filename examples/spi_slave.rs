#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
mod utilities;
use stm32h5xx_hal::{pac, prelude::*, spi};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(100.MHz())
        .pll1_q_ck(50.MHz())
        .freeze(pwrcfg, &dp.SBS);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();
    let hcs = gpiob.pb4.into_alternate();

    info!("");
    info!("stm32h5xx-hal example - SPI");
    info!("");

    // Initialise the SPI peripheral.
    let mut spi = dp.SPI2.spi_slave(
        (sck, miso, mosi, hcs),
        spi::Config::new(spi::MODE_0)
            // Specify that we use the hardware cs
            .hardware_cs(spi::HardwareCSMode::SlaveSelectInput),
        ccdr.peripheral.SPI2,
    );

    // This will write the contents of the buffer as long as a clock is provided
    spi.write(&[0u8, 1, 2, 3, 4, 5, 6]).unwrap();

    // This will read into the buffer as long as a clock is provided
    let read = &mut [0u8; 4];
    spi.read(read).unwrap();

    // This will read and write while a clock is provided
    let read = &mut [0u8; 3];
    spi.transfer(read, &[0x11, 0x22, 0x33]).unwrap();

    loop {
        cortex_m::asm::nop();
    }
}
