//! USB peripheral.
//!
//! Provides the required implementation for use of the [`stm32-usbd`] crate.

use crate::stm32::rcc::ccipr4::USBSEL;
pub use stm32_usbd::UsbBus;

use crate::gpio;
use crate::gpio::gpioa::{PA11, PA12};
use crate::rcc::rec;
use crate::stm32::{self, USB};
use core::fmt;
use stm32_usbd::UsbPeripheral;

/// Type for pin that can be the "D-" pin for the USB peripheral
pub type DmPin = PA11<gpio::Alternate<10>>;

/// Type for pin that can be the "D+" pin for the USB peripheral
pub type DpPin = PA12<gpio::Alternate<10>>;

pub trait UsbExt {
    fn usb(self, rec: rec::Usb, pin_dm: DmPin, pin_dp: DpPin) -> Peripheral;
}

impl UsbExt for stm32::USB {
    fn usb(self, rec: rec::Usb, pin_dm: DmPin, pin_dp: DpPin) -> Peripheral {
        if let USBSEL::Disable = rec.get_kernel_clk_mux() {
            rec.kernel_clk_mux(USBSEL::Hsi48);
        };

        Peripheral {
            _usb: self,
            pin_dm,
            pin_dp,
        }
    }
}

pub struct Peripheral {
    /// USB register block
    _usb: USB,
    /// Data negative pin
    pin_dm: DmPin,
    /// Data positive pin
    pin_dp: DpPin,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Peripheral {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Peripheral {{ usb: USB, pin_dm: {}, pin_dp: {}}}",
            self.pin_dm,
            self.pin_dp
        );
    }
}

impl fmt::Debug for Peripheral {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Peripheral")
            .field("usb", &"USB")
            .field("pin_dm", &self.pin_dm)
            .field("pin_dp", &self.pin_dp)
            .finish()
    }
}

// SAFETY: Implementation of Peripheral is thread-safe by using cricitcal sections to ensure
// mutually exclusive access to the USB peripheral
unsafe impl Sync for Peripheral {}

// SAFETY: The peripheral has the same regiter blockout as the STM32 USBFS
unsafe impl UsbPeripheral for Peripheral {
    const REGISTERS: *const () = USB::ptr().cast::<()>();
    const DP_PULL_UP_FEATURE: bool = true;
    const EP_MEMORY: *const () = 0x4001_6400 as _;
    const EP_MEMORY_SIZE: usize = 2048;
    const EP_MEMORY_ACCESS: stm32_usbd::MemoryAccess =
        stm32_usbd::MemoryAccess::Word32x1;

    fn enable() {
        cortex_m::interrupt::free(|_| {
            let rcc = unsafe { &*stm32::RCC::ptr() };

            #[cfg(any(feature = "h523_h533", feature = "h56x_h573"))]
            {
                let pwr = unsafe { &*stm32::PWR::ptr() };

                // Enable USB supply level detector
                pwr.usbscr().modify(|_, w| w.usb33den().set_bit());

                // Await good usb supply voltage
                while pwr.vmsr().read().usb33rdy().bit_is_clear() {}

                // Set bit to confirm that USB supply level is good
                pwr.usbscr().modify(|_, w| w.usb33sv().set_bit());
            }

            // Enable USB peripheral
            rcc.apb2enr().modify(|_, w| w.usben().set_bit());

            // Reset USB peripheral
            rcc.apb2rstr().modify(|_, w| w.usbrst().set_bit());
            rcc.apb2rstr().modify(|_, w| w.usbrst().clear_bit());
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32H503,523,533,56x and 573 it's
        // 1µs and this should wait for at least that long.
        // 250 Mhz is the highest frequency, so this ensures a minimum of 1µs wait time.
        cortex_m::asm::delay(250);
    }
}
