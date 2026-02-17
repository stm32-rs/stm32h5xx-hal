use core::ops::{Deref, DerefMut};

use crate::time::{Hertz, NanoSeconds};

use super::{Basic, Counter, Instance, Timer};

/// Timeout timer
///
/// Used to trigger overflow events on the timer at the specified frequency or timeout
/// values.
/// - Use [`Timeout::start_with_frequency`] to trigger events at the given frequency
/// - Use [`Timeout::start_with_timeout`] to trigger events after the given timeout period
///
/// To wait on the overflow event use [`wait_for_overflow`](Timer::wait_for_overflow)
pub struct Timeout<TIM>(Timer<TIM>);

impl<TIM> Timeout<TIM> {
    pub(super) fn new(timer: Timer<TIM>) -> Self {
        Self(timer)
    }
}

impl<TIM> Deref for Timeout<TIM> {
    type Target = Timer<TIM>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<TIM> DerefMut for Timeout<TIM> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[allow(private_bounds)]
impl<TIM: Instance + Basic> Timeout<TIM> {
    fn start(&mut self, ticks: u32) {
        // Pause
        self.tim.pause();

        // Reset counter
        self.tim.reset_counter();

        // UEV event occours on next overflow
        self.tim.urs_counter_only();
        self.tim.clear_timeout_flag();

        // Set PSC and ARR
        self.set_timeout_ticks(ticks);

        // Generate an update event to force an update of the ARR register. This ensures
        // the first timer cycle is of the specified duration.
        self.tim.apply_freq();

        // Start counter
        self.tim.resume()
    }

    /// Start the timer so that it overflows at the given frequency
    ///
    /// # Example
    /// ```
    /// timer.start_with_frequency(1.kHz())
    /// ```
    pub fn start_with_frequency<T>(&mut self, frequency: T)
    where
        T: Into<Hertz>,
    {
        let frequency: Hertz = frequency.into();
        let ticks = self.clk / frequency.raw();
        self.start(ticks);
    }

    /// Start the timer so that it repeatedly overflows after the specified timeout
    ///
    /// # Example
    /// ```
    /// timer.start_with_timeout(1.millis())
    /// ```
    pub fn start_with_timeout<T>(&mut self, timeout_ns: T)
    where
        T: Into<NanoSeconds>,
    {
        let timeout: NanoSeconds = timeout_ns.into();
        let clk = Hertz::from_raw(self.clk);
        let clk_period: NanoSeconds = clk.into_duration();

        let ticks = timeout / clk_period;
        self.start(ticks);
    }

    /// Convert into [`Tick`] timer. Cancels any ongoing operation.
    pub fn into_tick_timer(mut self) -> Tick<TIM> {
        self.cancel();
        self.0.tick()
    }

    /// Cancels the timer and releases the TIM peripheral
    pub fn free(self) -> (TIM, TIM::Rec) {
        self.0.free()
    }
}

pub struct Tick<TIM>(Timer<TIM>);

impl<TIM> Tick<TIM> {
    pub(super) fn new(timer: Timer<TIM>) -> Self {
        Self(timer)
    }
}

impl<TIM> Deref for Tick<TIM> {
    type Target = Timer<TIM>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<TIM> DerefMut for Tick<TIM> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<TIM: Instance + Basic> Tick<TIM> {
    /// Start the tick timer
    pub fn start<T>(&mut self, frequency_hz: T)
    where
        T: Into<Hertz>,
    {
        self.tim.pause();

        // UEV event occours on next overflow
        self.tim.urs_counter_only();
        self.tim.clear_timeout_flag();

        // Set PSC and ARR
        self.set_tick_frequency(frequency_hz.into());

        // Generate an update event to force an update of the ARR
        // register. This ensures the first timer cycle is of the
        // specified duration.
        self.tim.apply_freq();

        // Start counter
        self.tim.resume();
    }

    /// Configures the timer to count up at the given frequency
    ///
    /// Counts from 0 to the counter's maximum value, then repeats.
    /// Because this only uses the timer prescaler, the frequency
    /// is rounded to a multiple of the timer's kernel clock.
    fn set_tick_frequency(&mut self, frequency: Hertz) {
        let div = self.clk / frequency.raw();

        // TODO: This only works for frequencies high enough to result in a 16-bit divisor. Consider removing
        let psc = u16::try_from(div - 1).unwrap();
        self.tim.set_prescalar(psc);

        self.tim.set_auto_reload(TIM::Counter::MAX);
    }

    /// Convert to [`Timeout`] timer. Cancels any ongoing operation.
    pub fn into_timeout_timer(mut self) -> Timeout<TIM> {
        self.cancel();
        self.0.timeout()
    }

    /// Cancels the timer and releases the TIM peripheral
    pub fn free(self) -> (TIM, TIM::Rec) {
        self.0.free()
    }
}
