//! Timers
//!
//! # Examples
//!
//! - [Blinky using a Timer](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky_timer.rs)
//! - [64 bit microsecond timer](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/tick_timer.rs)

// TODO: on the h7x3 at least, only TIM2, TIM3, TIM4, TIM5 can support 32 bits.
// TIM1 is 16 bit.

use core::marker::PhantomData;

use crate::stm32::{TIM1, TIM2, TIM3, TIM6, TIM7};
#[cfg(feature = "rm0481")]
use crate::stm32::{
    TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM4, TIM5, TIM8,
}; // TODO: TIM12 seems to be missing for 523's pac, re add once fixed

use cast::{u16, u32};
use void::Void;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

/// Associate clocks with timers
pub trait GetClk {
    fn get_clk(clocks: &CoreClocks) -> Option<Hertz>;
}

/// Timers with CK_INT derived from rcc_tim[xy]_ker_ck
macro_rules! impl_tim_ker_ck {
    ($($ckX:ident: $($TIMX:ident),+)+) => {
        $(
            $(
                impl GetClk for $TIMX {
                    fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
                        Some(clocks.$ckX())
                    }
                }
            )+
        )+
    }
}
impl_tim_ker_ck! {
    timx_ker_ck: TIM2, TIM3, TIM6, TIM7
    timy_ker_ck: TIM1
}

#[cfg(feature = "rm0481")]
impl_tim_ker_ck! {
    timx_ker_ck: TIM4, TIM5, TIM12, TIM13, TIM14 // TODO: TIM12 seems to be missing for 523's pac, re add once fixed
    timy_ker_ck: TIM8, TIM15, TIM16, TIM17
}

/// External trait for hardware timers
pub trait TimerExt<TIM> {
    type Rec: ResetEnable;

    /// Configures a periodic timer
    ///
    /// Generates an overflow event at the `timeout` frequency.
    fn timer(self, timeout: Hertz, prec: Self::Rec, clocks: &CoreClocks)
        -> TIM;

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
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> TIM;
}

/// Hardware timers
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
}

/// Timer Events
///
/// Each event is a possible interrupt source, if enabled
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $Rec:ident, $cntType:ty),)+) => {
        $(
            impl embedded_hal_02::timer::Periodic for Timer<$TIMX> {}

            impl embedded_hal_02::timer::CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // Pause
                    self.pause();

                    // Reset counter
                    self.reset_counter();

                    // UEV event occours on next overflow
                    self.urs_counter_only();
                    self.clear_irq();

                    // Set PSC and ARR
                    self.set_freq(timeout.into());

                    // Generate an update event to force an update of the ARR register. This ensures
                    // the first timer cycle is of the specified duration.
                    self.apply_freq();

                    // Start counter
                    self.resume()
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.is_irq_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_irq();
                        Ok(())
                    }
                }
            }

            impl TimerExt<Timer<$TIMX>> for $TIMX {
                type Rec = rec::$Rec;

                fn timer(self, timeout: Hertz,
                            prec: Self::Rec, clocks: &CoreClocks
                ) -> Timer<$TIMX> {
                    use embedded_hal_02::timer::CountDown;

                    let mut timer = Timer::$timX(self, prec, clocks);
                    timer.start(timeout);
                    timer
                }

                fn tick_timer(self, frequency: Hertz,
                                 prec: Self::Rec, clocks: &CoreClocks
                ) -> Timer<$TIMX> {
                    let mut timer = Timer::$timX(self, prec, clocks);

                    timer.pause();

                    // UEV event occours on next overflow
                    timer.urs_counter_only();
                    timer.clear_irq();

                    // Set PSC and ARR
                    timer.set_tick_freq(frequency);

                    // Generate an update event to force an update of the ARR
                    // register. This ensures the first timer cycle is of the
                    // specified duration.
                    timer.apply_freq();

                    // Start counter
                    timer.resume();

                    timer
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer,
                /// without starting it
                pub fn $timX(tim: $TIMX, prec: rec::$Rec, clocks: &CoreClocks) -> Self
                {
                    // enable and reset peripheral to a clean state
                    let _ = prec.enable().reset(); // drop, can be recreated by free method

                    let clk = $TIMX::get_clk(clocks)
                        .expect(concat!(stringify!($TIMX), ": Input clock not running!")).raw();

                    Timer {
                        clk,
                        tim,
                    }
                }

                /// Configures the timer's frequency and counter reload value
                /// so that it underflows at the timeout's frequency
                pub fn set_freq(&mut self, timeout: Hertz) {
                    let ticks = self.clk / timeout.raw();

                    self.set_timeout_ticks(ticks);
                }

                /// Sets the timer period from a time duration
                ///
                /// ```
                /// use stm32h7xx_hal::time::MilliSeconds;
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
                    unsafe {
                        self.tim.psc().write(|w| w.psc().bits(psc));
                    }
                    #[allow(unused_unsafe)] // method is safe for some timers
                    self.tim.arr().write(|w| unsafe { w.bits(u32(arr)) });
                }

                /// Configures the timer to count up at the given frequency
                ///
                /// Counts from 0 to the counter's maximum value, then repeats.
                /// Because this only uses the timer prescaler, the frequency
                /// is rounded to a multiple of the timer's kernel clock.
                pub fn set_tick_freq(&mut self, frequency: Hertz) {
                    let div = self.clk / frequency.raw();

                    let psc = u16(div - 1).unwrap();
                    unsafe {
                        self.tim.psc().write(|w| w.psc().bits(psc));
                    }

                    let counter_max = u32(<$cntType>::MAX);
                    #[allow(unused_unsafe)] // method is safe for some timers
                    self.tim.arr().write(|w| unsafe { w.bits(counter_max) });
                }

                /// Applies frequency/timeout changes immediately
                ///
                /// The timer will normally update its prescaler and auto-reload
                /// value when its counter overflows. This function causes
                /// those changes to happen immediately. Also clears the counter.
                pub fn apply_freq(&mut self) {
                    self.tim.egr().write(|w| w.ug().set_bit());
                }

                /// Pauses the TIM peripheral
                pub fn pause(&mut self) {
                    self.tim.cr1().modify(|_, w| w.cen().clear_bit());
                }

                /// Resume (unpause) the TIM peripheral
                pub fn resume(&mut self) {
                    self.tim.cr1().modify(|_, w| w.cen().set_bit());
                }

                /// Set Update Request Source to counter overflow/underflow only
                pub fn urs_counter_only(&mut self) {
                    self.tim.cr1().modify(|_, w| w.urs().counter_only());
                }

                /// Reset the counter of the TIM peripheral
                pub fn reset_counter(&mut self) {
                    self.tim.cnt().reset();
                }

                /// Read the counter of the TIM peripheral
                pub fn counter(&self) -> u32 {
                    self.tim.cnt().read().cnt().bits().into()
                }

                /// Start listening for `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier().write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stop listening for `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Disable update event interrupt
                            self.tim.dier().write(|w| w.uie().clear_bit());
                            let _ = self.tim.dier().read();
                            let _ = self.tim.dier().read(); // Delay 2 peripheral clocks
                        }
                    }
                }

                /// Check if Update Interrupt flag is cleared
                pub fn is_irq_clear(&mut self) -> bool {
                    self.tim.sr().read().uif().bit_is_clear()
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    self.tim.sr().modify(|_, w| {
                        // Clears timeout event
                        w.uif().clear_bit()
                    });
                    let _ = self.tim.sr().read();
                    let _ = self.tim.sr().read(); // Delay 2 peripheral clocks
                }

                /// Releases the TIM peripheral
                pub fn free(mut self) -> ($TIMX, rec::$Rec) {
                    // pause counter
                    self.pause();

                    (self.tim, rec::$Rec { _marker: PhantomData })
                }

                /// Returns a reference to the inner peripheral
                pub fn inner(&self) -> &$TIMX {
                    &self.tim
                }

                /// Returns a mutable reference to the inner peripheral
                pub fn inner_mut(&mut self) -> &mut $TIMX {
                    &mut self.tim
                }
            }
        )+
    }
}

/// We want to have `ticks` amount of timer ticks before it reloads.
/// But `ticks` may have a higher value than what the timer can hold directly.
/// So we'll use the prescaler to extend the range.
///
/// To know how many times we would overflow with a prescaler of 1, we divide `ticks` by 2^16 (the max amount of ticks per overflow).
/// If the result is e.g. 3, then we need to increase our range by 4 times to fit all the ticks.
/// We can increase the range enough by setting the prescaler to 3 (which will divide the clock freq by 4).
/// Because every tick is now 4x as long, we need to divide `ticks` by 4 to keep the same timeout.
///
/// This function returns the prescaler register value and auto reload register value.
fn calculate_timeout_ticks_register_values(ticks: u32) -> (u16, u16) {
    // Note (unwrap): Never panics because 32-bit value is shifted right by 16 bits,
    // resulting in a value that always fits in 16 bits.
    let psc = u16(ticks / (1 << 16)).unwrap();
    // Note (unwrap): Never panics because the divisor is always such that the result fits in 16 bits.
    // Also note that the timer counts `0..=arr`, so subtract 1 to get the correct period.
    let arr = u16(ticks / (u32(psc) + 1)).unwrap().saturating_sub(1);
    (psc, arr)
}

hal! {
    // Advanced-control
    TIM1: (tim1, Tim1, u16),

    // General-purpose
    TIM2: (tim2, Tim2, u32),
    TIM3: (tim3, Tim3, u16),

    // Basic
    TIM6: (tim6, Tim6, u16),
    TIM7: (tim7, Tim7, u16),
}

#[cfg(feature = "rm0481")]
hal! {
    // Advanced-control
    TIM8: (tim8, Tim8, u16),

    // General-purpose
    TIM4: (tim4, Tim4, u16),
    TIM5: (tim5, Tim5, u32),

    // General-purpose
    //TIM12: (tim12, Tim12, u16), // TODO: TIM12 seems to be missing for 523's pac, re add once fixed

    // General-purpose
    TIM15: (tim15, Tim15, u16),
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
