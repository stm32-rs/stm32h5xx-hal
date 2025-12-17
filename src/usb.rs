//! USB peripheral.
//!
//! Provides the required implementation for use of the [`stm32-usbd`] crate.

use crate::stm32::rcc::ccipr4::USBSEL;
pub use stm32_usbd::UsbBus;

use crate::gpio;
use crate::gpio::gpioa::{PA11, PA12};
use crate::rcc::{rec, ResetEnable};
use crate::stm32::{self, USB};
use core::marker::PhantomData;
use stm32_usbd::UsbPeripheral;

/// Type for pin that can be the "D-" pin for the USB peripheral
pub type DmPin = PA11<gpio::Alternate<10>>;

/// Type for pin that can be the "D+" pin for the USB peripheral
pub type DpPin = PA12<gpio::Alternate<10>>;

pub trait UsbExt {
    fn usb(self, rec: rec::Usb, pin_dm: DmPin, pin_dp: DpPin) -> UsbDevice;
}

impl UsbExt for stm32::USB {
    fn usb(self, rec: rec::Usb, _pin_dm: DmPin, _pin_dp: DpPin) -> UsbDevice {
        if let USBSEL::Disable = rec.get_kernel_clk_mux() {
            rec.kernel_clk_mux(USBSEL::Hsi48);
        };

        UsbDevice { _usb: self }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsbDevice {
    /// USB register block
    _usb: USB,
}

#[cfg(feature = "defmt")]
impl defmt::Format for UsbDevice {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "UsbDevice {{ usb: USB }}");
    }
}

// SAFETY: Implementation of UsbDevice is thread-safe by using cricitcal sections to ensure
// mutually exclusive access to the USB peripheral
unsafe impl Sync for UsbDevice {}

// SAFETY: The peripheral has the same regiter blockout as the STM32 USBFS
unsafe impl UsbPeripheral for UsbDevice {
    const REGISTERS: *const () = USB::ptr().cast::<()>();
    const DP_PULL_UP_FEATURE: bool = true;
    const EP_MEMORY: *const () = 0x4001_6400 as _;
    const EP_MEMORY_SIZE: usize = 2048;
    const EP_MEMORY_ACCESS: stm32_usbd::MemoryAccess =
        stm32_usbd::MemoryAccess::Word32x1;

    fn enable() {
        cortex_m::interrupt::free(|_| {
            #[cfg(any(feature = "h523_h533", feature = "h56x_h573"))]
            {
                // Safety: we are only touching the usbscr which
                // is specific for this peripheral. This together with
                // the critical section unsures exclusive access
                let pwr = unsafe { &*stm32::PWR::ptr() };

                // Enable USB supply level detector
                pwr.usbscr().modify(|_, w| w.usb33den().set_bit());

                // Await good usb supply voltage
                while pwr.vmsr().read().usb33rdy().bit_is_clear() {}

                // Set bit to confirm that USB supply level is good
                pwr.usbscr().modify(|_, w| w.usb33sv().set_bit());
            }

            // Reset and enable USB peripheral
            rec::Usb {
                _marker: PhantomData,
            }
            .reset()
            .enable();
        });
    }

    fn startup_delay() {
        // There is a chip specific startup delay. For STM32H503,523,533,56x and 573 it's
        // 1µs and this should wait for at least that long.
        // 250 Mhz is the highest frequency, so this ensures a minimum of 1µs wait time.
        cortex_m::asm::delay(250);
    }
}
