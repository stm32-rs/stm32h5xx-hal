#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;
use stm32h5xx_hal::{
    crc::{Config, ReverseInput},
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
    let mut crc = dp.CRC.crc(ccdr.peripheral.CRC);

    info!("stm32h5xx CRC Example");

    // Default config: CRC-32 Ethernet polynomial (0x04C11DB7),
    // init=0xFFFFFFFF, no reversal, no output XOR.
    const CHECK_DATA: &[u8] = b"rust_is_awesome";
    info!("CRC with default settings");
    crc.update(CHECK_DATA);
    let r1 = crc.finish();
    assert_eq!(r1, 0x673502ED);

    info!("Incremental updates with read_crc; does not reset the count");
    crc.update(&CHECK_DATA[..8]);
    let partial = crc.read_crc();
    crc.update(&CHECK_DATA[8..]);
    let full = crc.finish();
    assert_ne!(partial, full);
    // finish resets, so repeating the full input gives the same result
    assert_eq!(full, r1);

    info!("CRC with reversed I/O");
    let cfg = Config::new()
        .reverse_input(ReverseInput::WordReverse)
        .reverse_output(true);
    crc.set_config(&cfg);
    crc.update(CHECK_DATA);
    let r2 = crc.finish();
    assert_eq!(r2, 0x9A873CBF);

    info!("Independent data register (IDR) 32-bit scratch space unaffected by CRC reset");
    assert_eq!(crc.get_idr(), 0);
    crc.set_idr(0x12345678);
    assert_eq!(crc.get_idr(), 0x12345678);

    info!("CRC-32/ISO-HDLC with output_xor");
    let iso_hdlc = Config::new()
        .reverse_input(ReverseInput::ByteReverse)
        .reverse_output(true)
        .output_xor(0xFFFF_FFFF);
    crc.set_config(&iso_hdlc);
    crc.update(CHECK_DATA);
    let r3 = crc.finish();
    assert_eq!(r3, 0x9958656);

    info!("Custom 16-bit polynomial");
    let crc16 = Config::new()
        .polynomial(stm32h5xx_hal::crc::Polynomial::bits16_unchecked(0x1021))
        .initial_value(0xFFFF)
        .reverse_input(ReverseInput::NoReverse)
        .reverse_output(false);
    crc.set_config(&crc16);
    crc.update(CHECK_DATA);
    let r4 = crc.finish();
    assert_eq!(r4, 0x0000_d853);
    loop {}
}
