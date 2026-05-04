use core::marker::PhantomData;

use crate::rcc::{rec, CoreClocks};
#[cfg(all(feature = "rm0481", not(feature = "stm32h523")))]
// TIM12 is not defined in the PAC for STM32H523. Remove this compiler switch when it is.
use crate::stm32::TIM12;
use crate::stm32::{TIM1, TIM2, TIM3, TIM6, TIM7};
#[cfg(feature = "rm0481")]
use crate::stm32::{TIM15, TIM4, TIM5, TIM8};

#[cfg(feature = "h56x_h573")]
use crate::stm32::{TIM13, TIM14, TIM16, TIM17};
use crate::time::Hertz;

use super::{Basic, Instance};

macro_rules! timer {
    ($TIMX:ident: $cntType:ty, $ker_ck:ident) => {
        paste::item! {
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

                fn set_auto_reload(&mut self, arr: Self::Counter) {
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

                fn counter(&self) -> Self::Counter {
                    self.cnt().read().cnt().bits()
                }

                fn enable_timeout_interrupt(&mut self) {
                    // Enable update event interrupt
                    self.dier().write(|w| w.uie().enabled());
                }

                fn disable_timeout_interrupt(&mut self) {
                    self.dier().write(|w| w.uie().disabled());
                    interrupt_clear_clock_sync_delay!(self.dier());
                }


                fn is_timeout_complete(&self) -> bool {
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
        }
    };
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
    #[cfg(not(feature = "stm32h523"))]
    timer!(TIM12: u16, timx_ker_ck);
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
