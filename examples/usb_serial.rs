//! CDC-ACM serial port example using polling in a busy loop.
#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::prelude::*;
use hal::pwr::PwrExt;
use hal::stm32;
use stm32_usbd::UsbBus;
use stm32h5xx_hal as hal;
use stm32h5xx_hal::usb::UsbExt;

use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();
    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let mut led = gpioa.pa5.into_push_pull_output();
    led.set_low();

    let usb_dm = gpioa.pa11.into_alternate();
    let usb_dp = gpioa.pa12.into_alternate();

    let usb = dp.USB.usb(
        ccdr.peripheral.USB, /*.kernel_clk_mux(USBSEL::Pll2Q)*/
        usb_dm,
        usb_dp,
    );
    let usb_bus = UsbBus::new(usb);

    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev =
        UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(USB_CLASS_CDC)
            .build();

    defmt::println!("Init done");

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                led.set_high();

                if let Ok(s) = str::from_utf8(&buf[0..count]) {
                    defmt::println!("{}", s);
                } else {
                    defmt::println!("{}", buf[0..count]);
                }

                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }

        led.set_low();
    }
}
