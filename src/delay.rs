//! Delay providers
//!
//! # Examples
//!
//! ## Delay
//!
//! ```no_run
//! let mut delay = Delay::new(core.SYST, device.clocks);
//!
//! delay.delay_ms(500);
//!
//! // Release SYST from the delay
//! let syst = delay.free();
//! ```
//!
//! # Examples
//!
//! - [Blinky](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs)

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use embedded_hal::delay::DelayNs;
use fugit::SecsDurationU64;

use crate::rcc::CoreClocks;

const SYSTICK_HCLK_DIV: u32 = 8;

pub trait DelayExt {
    fn delay(self, clocks: &CoreClocks) -> Delay;
}

impl DelayExt for SYST {
    fn delay(self, clocks: &CoreClocks) -> Delay {
        Delay::new(self, clocks)
    }
}

/// System timer (SysTick) as a delay provider
pub struct Delay {
    hclk_hz: u32,
    syst: SYST,
}
fn calc_rvr(ns: u32, hclk: u32) -> u32 {
    // Default is for SYSTICK to be fed by HCLK/8
    let ticks: u64 = (SecsDurationU64::secs(1) * SYSTICK_HCLK_DIV).to_nanos();
    ((ns as u64 * hclk as u64) / ticks) as u32
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: &CoreClocks) -> Self {
        syst.set_clock_source(SystClkSource::External);

        Delay {
            hclk_hz: clocks.hclk().raw(),
            syst,
        }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        let mut total_rvr = calc_rvr(ns, self.hclk_hz);

        while total_rvr != 0 {
            let current_rvr = if total_rvr <= MAX_RVR {
                total_rvr
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            total_rvr -= current_rvr;

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::calc_rvr;
    #[test]
    fn test_calc_rvr() {
        let rvr = calc_rvr(1000, 8_000_000);
        assert_eq!(rvr, 1);

        let rvr = calc_rvr(1000_000, 8_000_000);
        assert_eq!(rvr, 1000);

        let rvr = calc_rvr(1000_000, 10_000_000);
        assert_eq!(rvr, 1250);

        let rvr = calc_rvr(1000_000, 250_000_000);
        assert_eq!(rvr, 31250);
    }
}
