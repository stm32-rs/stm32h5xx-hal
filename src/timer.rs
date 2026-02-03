//! Timers
//!
//! # Examples
//!
//! - [Blinky using a Timer](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/blinky_timer.rs)

use core::marker::PhantomData;

use crate::stm32::{TIM1, TIM2, TIM3, TIM6, TIM7};
#[cfg(feature = "rm0481")]
use crate::stm32::{/*TIM12,*/ TIM15, TIM4, TIM5, TIM8}; // TODO: TIM12 seems to be missing for 523's pac, re add once fixed
#[cfg(feature = "h56x_h573")]
use crate::stm32::{TIM13, TIM14, TIM16, TIM17};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

trait Counter: Into<u32> + TryFrom<u32> + From<u16> {
    const MAX: Self;
}

impl Counter for u16 {
    const MAX: u16 = u16::MAX;
}

impl Counter for u32 {
    const MAX: u32 = u32::MAX;
}

pub trait Instance: crate::Sealed + Sized {
    type Rec: ResetEnable;

    #[doc(hidden)]
    fn clock(clocks: &CoreClocks) -> Hertz;

    fn rec() -> Self::Rec;
}

macro_rules! timer {
    ($TIMX:ident: $cntType:ty, $ker_ck:ident) => { paste::item! {
        impl crate::Sealed for $TIMX {}

        impl Instance for $TIMX {
            type Rec = rec::[< $TIMX:lower:camel >];

            fn clock(clocks: &CoreClocks) -> Hertz {
                clocks.$ker_ck()
            }

            fn rec() -> Self::Rec {
                rec::[< $TIMX:lower:camel >] { _marker: PhantomData }
            }
        }

        impl Basic for $TIMX {
            type Counter = $cntType;
            fn set_prescalar(&mut self, psc: u16) {
                self.psc().write(|w| w.psc().set(psc));

            }

            fn set_arr(&mut self, arr: Self::Counter) {
                // #[allow(unused_unsafe)] // method is safe for some timers
                self.arr().write(|w| w.arr().set(arr.into()));
            }

            fn apply_freq(&mut self) {
                self.egr().write(|w| w.ug().update());
            }

            fn pause(&mut self) {
                self.cr1().modify(|_, w| w.cen().disabled());
            }

            fn resume(&mut self) {
                self.cr1().modify(|_, w| w.cen().enabled());
            }

            fn urs_counter_only(&mut self) {
                self.cr1().modify(|_, w| w.urs().counter_only());
            }

            /// Reset the counter of the TIM peripheral
            fn reset_counter(&mut self) {
                self.cnt().reset();
            }

            fn enable_timeout_interrupt(&mut self) {
                // Enable update event interrupt
                self.dier().write(|w| w.uie().enabled());
            }

            fn disable_timeout_interrupt(&mut self) {
                self.dier().write(|w| w.uie().disabled());
                interrupt_clear_clock_sync_delay!(self.dier());
            }


            fn is_timeout_complete(&mut self) -> bool {
                self.sr().read().uif().is_update_pending()
            }

            /// Clears interrupt flag
            fn clear_timeout_flag(&mut self) {
                self.sr().modify(|_, w| {
                    // Clears timeout event
                    w.uif().clear()
                });
                interrupt_clear_clock_sync_delay!(self.sr());
            }
        }
    }};
}

// Advanced Control
timer!(TIM1: u16, timy_ker_ck);

// General-purpose
timer!(TIM2: u32, timx_ker_ck);
timer!(TIM3: u16, timx_ker_ck);

// Basic
timer!(TIM6: u16, timx_ker_ck);
timer!(TIM7: u16, timx_ker_ck);

#[cfg(feature = "rm0481")]
mod rm0481 {
    use super::*;
    // Advanced-control
    timer!(TIM8: u16, timy_ker_ck);

    // General-purpose
    timer!(TIM4: u16, timx_ker_ck);
    timer!(TIM5: u32, timx_ker_ck);
    timer!(TIM15: u16, timy_ker_ck);
    //timer!(TIM12, u16, timx_ker_ck), // TODO: TIM12 seems to be missing for 523's pac, re add once fixed
}

#[cfg(feature = "h56x_h573")]
mod h56x_h573 {
    use super::*;
    // General-purpose
    timer!(TIM13: u16, timx_ker_ck);
    timer!(TIM14: u16, timx_ker_ck);
    timer!(TIM16: u16, timy_ker_ck);
    timer!(TIM17: u16, timy_ker_ck);
}

/// External trait for hardware timers
pub trait TimerExt<TIM: Instance> {
    /// Configures a periodic timer
    ///
    /// Generates an overflow event at the `timeout` frequency.
    fn timer(self, timeout: Hertz, prec: TIM::Rec, clocks: &CoreClocks)
        -> Timer<TIM>;

    /// Configures the timer to count up at the given frequency
    ///
    /// Counts from 0 to the counter's maximum value, then repeats.
    /// Because this only uses the timer prescaler, the frequency
    /// is rounded to a multiple of the timer's kernel clock.
    ///
    /// For example, calling `.tick_timer(1.MHz(), ..)` for a 16-bit timer will
    /// result in a timers that increments every microsecond and overflows every
    /// ~65 milliseconds
    fn tick_timer(
        self,
        frequency: Hertz,
        prec: TIM::Rec,
        clocks: &CoreClocks,
    ) -> Timer<TIM>;
}

/// Hardware timers
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
}

impl<TIM: Instance + Basic> TimerExt<TIM> for TIM {
    fn timer(self, timeout: Hertz,
                prec: TIM::Rec, clocks: &CoreClocks
    ) -> Timer<TIM> {
        let mut timer = Timer::new(self, prec, clocks);
        timer.start(timeout);
        timer
    }

    fn tick_timer(self, frequency: Hertz,
                     prec: TIM::Rec, clocks: &CoreClocks
    ) -> Timer<TIM> {
        let mut timer = Timer::new(self, prec, clocks);

        timer.tick_timer(frequency);

        timer
    }
}

impl<TIM: Instance> Timer<TIM> {
    /// Configures a TIM peripheral as a periodic count down timer,
    /// without starting it
    pub fn new(tim: TIM, prec: TIM::Rec, clocks: &CoreClocks) -> Self
    {
        // enable and reset peripheral to a clean state
        let _ = prec.enable().reset(); // drop, can be recreated by free method

        let clk = TIM::clock(clocks).raw();
            // .expect(concat!(stringify!(TIM), ": Input clock not running!")).raw();

        Timer {
            clk,
            tim,
        }
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

#[allow(private_bounds)]
impl<TIM: Instance + Basic> Timer<TIM> {
    /// Configures the timer's frequency and counter reload value
    /// so that it underflows at the timeout's frequency
    pub fn set_freq(&mut self, timeout: Hertz) {
        let ticks = self.clk / timeout.raw();

        self.set_timeout_ticks(ticks);
    }

    /// Sets the timer period from a time duration
    ///
    /// ```
    /// use stm32h5xx_hal::time::MilliSeconds;
    ///
    /// // Set timeout to 100ms
    /// let timeout = MilliSeconds::from_ticks(100).into_rate();
    /// timer.set_timeout(timeout);
    /// ```
    ///
    /// Alternatively, the duration can be set using the
    /// core::time::Duration type
    ///
    /// ```
    /// let duration = core::time::Duration::from_nanos(2_500);
    ///
    /// // Set timeout to 2.5µs
    /// timer.set_timeout(duration);
    /// ```
    pub fn set_timeout<T>(&mut self, timeout: T)
    where
        T: Into<core::time::Duration>
    {
        const NANOS_PER_SECOND: u64 = 1_000_000_000;
        let timeout = timeout.into();

        let clk = self.clk as u64;
        let ticks = u32::try_from(
            clk * timeout.as_secs() +
            clk * u64::from(timeout.subsec_nanos()) / NANOS_PER_SECOND,
        )
        .unwrap_or(u32::MAX);

        self.set_timeout_ticks(ticks.max(1));
    }

    pub fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
    {
        // Pause
        self.tim.pause();

        // Reset counter
        self.tim.reset_counter();

        // UEV event occours on next overflow
        self.tim.urs_counter_only();
        self.tim.clear_timeout_flag();

        // Set PSC and ARR
        self.set_freq(timeout.into());

        // Generate an update event to force an update of the ARR register. This ensures
        // the first timer cycle is of the specified duration.
        self.tim.apply_freq();

        // Start counter
        self.tim.resume()
    }

    /// Check whether the timeout has occurred and clear status flags if it has
    pub fn check_clear_timeout(&mut self) -> bool {
        if self.tim.is_timeout_complete() {
            self.tim.clear_timeout_flag();
            true
        } else {
            false
        }
    }

    /// Blocks until timeout occurs
    pub fn wait_for_timeout(&mut self) {
        while !self.check_clear_timeout() {}
    }

    fn tick_timer(&mut self, frequency: Hertz) {
        self.tim.pause();

        // UEV event occours on next overflow
        self.tim.urs_counter_only();
        self.tim.clear_timeout_flag();

        // Set PSC and ARR
        self.set_tick_freq(frequency);

        // Generate an update event to force an update of the ARR
        // register. This ensures the first timer cycle is of the
        // specified duration.
        self.tim.apply_freq();

        // Start counter
        self.tim.resume();
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
        self.tim.set_arr(arr.into());
    }

    /// Configures the timer to count up at the given frequency
    ///
    /// Counts from 0 to the counter's maximum value, then repeats.
    /// Because this only uses the timer prescaler, the frequency
    /// is rounded to a multiple of the timer's kernel clock.
    pub fn set_tick_freq(&mut self, frequency: Hertz) {
        let div = self.clk / frequency.raw();

        // TODO: This only works for frequencies high enough to result in a 16-bit divisor. Consider removing
        let psc = u16::try_from(div - 1).unwrap();
        self.tim.set_prescalar(psc);

        let counter_max = TIM::Counter::MAX;
        self.tim.set_arr(counter_max);
    }

    /// Enable timeout interrupt. This maps to the Update Event timeout
    pub fn enable_timeout_interrupt(&mut self) {
        self.tim.enable_timeout_interrupt();
    }

    /// Disable timeout interrupt. This maps to the Update Event timeout
    pub fn disable_timeout_interrupt(&mut self) {
        self.tim.disable_timeout_interrupt();
    }

    /// Releases the TIM peripheral
    pub fn free(mut self) -> (TIM, TIM::Rec) {
        // pause counter
        self.tim.pause();
        self.tim.disable_timeout_interrupt();

        (self.tim, TIM::rec())
    }
}

/// The Basic trait represents functionality common to all timers (Basic, General Purpose, and
/// Advanced-control Timers), as described in RM0492 and RM0481. It facilitates basic timeout
/// operations which are exposed on the [`Timer`] instance for all TIM peripherals.
trait Basic {
    type Counter: Counter;
    fn set_prescalar(&mut self, psc: u16);

    fn set_arr(&mut self, arr: Self::Counter);

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

    /// Enable timeout interrupt for
    fn enable_timeout_interrupt(&mut self);

    fn disable_timeout_interrupt(&mut self);

    /// Check if Update Interrupt flag is cleared
    fn is_timeout_complete(&mut self) -> bool;

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
    let arr = u16::try_from(ticks / ((psc as u32) + 1)).unwrap().saturating_sub(1);
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
