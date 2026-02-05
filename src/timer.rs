//! Timers
//!
//! The STM32H5 family includes a rich set of timers, with different sets of functionality. The
//! Basic timer simply allows for timer operation based on a counter overflow. The General-Purpose
//! timers also include Input Compare, Output Compare, and PWM operation. The Advanced Timers
//! include more advanced PWM functionality. See the RM0492/RM0481 Reference Manual for detailed
//! information of what timer provides what functionality.
//!
//! # Counter width
//! Different timers support different auto-reload counter widths which allow for longer time
//! periods between update events.
//!
//! # Basic Timer
//! The basic timer operation is exposed either as a periodic timer that generates a timeout at
//! a specific frequency or as a as a timer that counts ticks up to the maximum overflow value at
//! a specific frequency. In both cases, the timer can generate an interrupt on timer overflow.
//!
//! ## [`Timeout``] timer
//! The timeout timer runs continuously once started, and generates an overflow event at the
//! frequency specified. This can be used to trigger single or repeated events after specific
//! time periods.
//!
//! ### Starting with a overflow frequency
//! ```
//! let dp = ...;            // Device peripherals
//!
//! let timeout = dp.TIM1.timeout(ccdr.peripheral.TIM1, &ccdr.clocks);
//! timeout.start_with_frequency(2.Hz());
//! ```
//!
//! ### Starting with an overflow timeout
//! ```
//! let dp = ...;            // Device peripherals
//!
//! let timeout = dp.TIM1.timeout(ccdr.peripheral.TIM1, &ccdr.clocks);
//! timeout.start_with_timeout(500.millis());
//! timeout.wait_for_overflow();
//! ```
//!
//! ## Tick timer
//! The tick timer runs continuously, incrementing the timer counter register at the frequency
//! specified. This can be used to generate for timing purposes (e.g. a monotonic clock).
//!
//! ### Usage
//! ```
//! let dp = ...;            // Device peripherals
//!
//! let tick = dp.TIM1.tick(ccdr.peripheral.TIM1, &ccdr.clocks);
//! tick.start(1.MHz());
//! ```
//! # Examples
//!
//! - [Blinky using a Timer](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/blinky_timer.rs)

use core::fmt::Debug;

use crate::rcc::{CoreClocks, ResetEnable};
use crate::time::Hertz;

mod counters;
mod timer_def;

pub use counters::{Tick, Timeout};

/// Counter represents the word type used for setting the period of timer.
/// Consult RM0492/RM0481 for which timer supports which word sizes.
#[doc(hidden)]
pub trait Counter: Into<u32> + TryFrom<u32> + From<u16> + Debug {
    const MAX: Self;
}

impl Counter for u16 {
    const MAX: u16 = u16::MAX;
}

impl Counter for u32 {
    const MAX: u32 = u32::MAX;
}

#[doc(hidden)]
pub trait Instance: crate::Sealed + Sized {
    type Rec: ResetEnable;

    #[doc(hidden)]
    fn clock(clocks: &CoreClocks) -> Hertz;

    fn rec() -> Self::Rec;
}

/// External trait for hardware timers
pub trait TimerExt<TIM: Instance> {
    /// Configures a periodic timer
    ///
    /// Generates an overflow event at the timeout frequency. The timer
    /// can be started using either of [`Timeout::start_with_frequency`] or
    /// [`Timeout::start_with_timeout`] to schedule timeouts either at the
    /// frequency specified or after period specified, respectively.
    fn timeout(self, prec: TIM::Rec, clocks: &CoreClocks) -> Timeout<TIM>;

    /// Configures the timer to count up at the given frequency
    ///
    /// Counts from 0 to the counter's maximum value, then repeats.
    /// Because this only uses the timer prescaler, the frequency
    /// is rounded to a multiple of the timer's kernel clock.
    ///
    /// For example, calling `.start(1.MHz())` for a 16-bit timer will
    /// result in a timer that increments every microsecond and overflows every
    /// ~65 milliseconds
    fn tick(self, prec: TIM::Rec, clocks: &CoreClocks) -> Tick<TIM>;
}

/// Hardware timers
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
}

impl<TIM: Instance + Basic> TimerExt<TIM> for TIM {
    fn timeout(self, prec: TIM::Rec, clocks: &CoreClocks) -> Timeout<TIM> {
        Timer::new(self, prec, clocks).timeout()
    }

    fn tick(self, prec: TIM::Rec, clocks: &CoreClocks) -> Tick<TIM> {
        Timer::new(self, prec, clocks).tick()
    }
}

impl<TIM: Instance> Timer<TIM> {
    pub fn new(tim: TIM, prec: TIM::Rec, clocks: &CoreClocks) -> Self {
        // Enable and reset peripheral to a clean state
        let _ = prec.enable().reset();

        let clk = TIM::clock(clocks).raw();

        Timer { clk, tim }
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &TIM {
        &self.tim
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut TIM {
        &mut self.tim
    }
}

impl<TIM: Instance + Basic> Timer<TIM> {
    fn timeout(self) -> Timeout<TIM> {
        Timeout::new(self)
    }

    fn tick(self) -> Tick<TIM> {
        Tick::new(self)
    }

    /// Check whether the timeout has occurred and clear status flags if it has
    pub fn check_clear_overflow(&mut self) -> bool {
        if self.tim.is_timeout_complete() {
            self.tim.clear_timeout_flag();
            true
        } else {
            false
        }
    }

    /// Blocks until timeout occurs
    pub fn wait_for_overflow(&mut self) {
        while !self.check_clear_overflow() {}
    }

    /// Read the value of the timer counter
    pub fn counter_value(&self) -> u32 {
        self.tim.counter().into()
    }

    /// Sets the timer's prescaler and auto reload register so that the timer will reach
    /// the ARR after `ticks - 1` amount of timer clock ticks.
    ///
    /// ```
    /// // Set auto reload register to 50000 and prescaler to divide by 2.
    /// timer.set_timeout_ticks(100000);
    /// ```
    ///
    /// This function will round down if the prescaler is used to extend the range:
    /// ```
    /// // Set auto reload register to 50000 and prescaler to divide by 2.
    /// timer.set_timeout_ticks(100001);
    /// ```
    fn set_timeout_ticks(&mut self, ticks: u32) {
        let (psc, arr) = calculate_timeout_ticks_register_values(ticks);
        self.tim.set_prescalar(psc.into());
        self.tim.set_auto_reload(arr.into());
    }

    /// Enable timeout interrupt. This maps to the Update Event timeout
    pub fn enable_timeout_interrupt(&mut self) {
        self.tim.enable_timeout_interrupt();
    }

    /// Disable timeout interrupt. This maps to the Update Event timeout
    pub fn disable_timeout_interrupt(&mut self) {
        self.tim.disable_timeout_interrupt();
    }

    /// Cancel timer
    pub fn cancel(&mut self) {
        self.tim.pause();
        self.tim.disable_timeout_interrupt();
    }

    /// Releases the TIM peripheral
    fn free(mut self) -> (TIM, TIM::Rec) {
        self.cancel();

        (self.tim, TIM::rec())
    }
}

/// The Basic trait represents functionality common to all timers (Basic, General Purpose, and
/// Advanced-control Timers), as described in RM0492 and RM0481. It facilitates basic timeout
/// operations which are exposed on the [`Timer`] instance for all TIM peripherals.
#[doc(hidden)]
pub trait Basic {
    type Counter: Counter;
    fn set_prescalar(&mut self, psc: u16);

    fn set_auto_reload(&mut self, arr: Self::Counter);

    /// Applies frequency/timeout changes immediately
    ///
    /// The timer will normally update its prescaler and auto-reload
    /// value when its counter overflows. This function causes
    /// those changes to happen immediately. Also clears the counter.
    fn apply_freq(&mut self);

    /// Pauses the TIM peripheral
    fn pause(&mut self);

    /// Resume (unpause) the TIM peripheral
    fn resume(&mut self);

    /// Set Update Request Source to counter overflow/underflow only
    fn urs_counter_only(&mut self);

    /// Reset the counter of the TIM peripheral
    fn reset_counter(&mut self);

    /// Timer counter value
    fn counter(&self) -> Self::Counter;

    /// Enable timeout interrupt
    fn enable_timeout_interrupt(&mut self);

    /// Disable timeout interrupt
    fn disable_timeout_interrupt(&mut self);

    /// Check if Update Interrupt flag is cleared
    fn is_timeout_complete(&self) -> bool;

    /// Clears interrupt flag
    fn clear_timeout_flag(&mut self);
}

/// We want to have `ticks` amount of timer ticks before it reloads.
/// But `ticks` may have a higher value than what the timer can hold directly.
/// So we'll use the prescaler to extend the range.
///
/// To know how many times we would overflow with a prescaler of 1, we divide `ticks` by 2^(bit
/// width of the counter), the max amount of ticks per overflow. E.g: if the result is e.g. 3,
/// then we need to increase our range by 4 times to fit all the ticks. We can increase the range
/// enough by setting the prescaler to 3 (which will divide the clock freq by 4). Because every
/// tick is now 4x as long, we need to divide `ticks` by 4 to keep the same timeout.
///
/// This function returns the prescaler register value and auto reload register value.
fn calculate_timeout_ticks_register_values(ticks: u32) -> (u16, u16) {
    // Note (unwrap): Never panics because 32-bit value is shifted right by 16 bits,
    // resulting in a value that always fits in 16 bits.
    let psc = (ticks / (1 << u16::BITS)).try_into().unwrap();
    // Note (unwrap): Never panics because the divisor is always such that the result fits in 16 bits.
    // Also note that the timer counts `0..=arr`, so subtract 1 to get the correct period.
    let arr = u16::try_from(ticks / ((psc as u32) + 1))
        .unwrap()
        .saturating_sub(1);
    (psc, arr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeout_ticks_register_values() {
        assert_eq!(calculate_timeout_ticks_register_values(0), (0, 0));
        assert_eq!(calculate_timeout_ticks_register_values(50000), (0, 49999));
        assert_eq!(calculate_timeout_ticks_register_values(100000), (1, 49999));
        assert_eq!(calculate_timeout_ticks_register_values(65535), (0, 65534));
        assert_eq!(calculate_timeout_ticks_register_values(65536), (1, 32767));
        assert_eq!(
            calculate_timeout_ticks_register_values(1000000),
            (15, 62499)
        );
        assert_eq!(
            calculate_timeout_ticks_register_values(u32::MAX),
            (u16::MAX, u16::MAX - 1)
        );
    }
}
