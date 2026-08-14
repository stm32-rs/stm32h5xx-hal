use core::{marker::PhantomData, ops::Deref};

use super::{marker, Edge, Pin, PinExt};
use crate::{
    gpio,
    pac::{Interrupt, EXTI},
};

pub trait ExtiExt {
    fn split(self) -> (Exti, ExtiChannels);
}

impl ExtiExt for EXTI {
    fn split(self) -> (Exti, ExtiChannels) {
        (
            Exti(self),
            ExtiChannels {
                ch0: ExtiChannel,
                ch1: ExtiChannel,
                ch2: ExtiChannel,
                ch3: ExtiChannel,
                ch4: ExtiChannel,
                ch5: ExtiChannel,
                ch6: ExtiChannel,
                ch7: ExtiChannel,
                ch8: ExtiChannel,
                ch9: ExtiChannel,
                ch10: ExtiChannel,
                ch11: ExtiChannel,
                ch12: ExtiChannel,
                ch13: ExtiChannel,
                ch14: ExtiChannel,
                ch15: ExtiChannel,
            },
        )
    }
}

pub struct Exti(pub(crate) EXTI);

impl Deref for Exti {
    type Target = EXTI;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[non_exhaustive]
pub struct ExtiChannel<const N: u8>;

pub struct ExtiChannels {
    pub ch0: ExtiChannel<0>,
    pub ch1: ExtiChannel<1>,
    pub ch2: ExtiChannel<2>,
    pub ch3: ExtiChannel<3>,
    pub ch4: ExtiChannel<4>,
    pub ch5: ExtiChannel<5>,
    pub ch6: ExtiChannel<6>,
    pub ch7: ExtiChannel<7>,
    pub ch8: ExtiChannel<8>,
    pub ch9: ExtiChannel<9>,
    pub ch10: ExtiChannel<10>,
    pub ch11: ExtiChannel<11>,
    pub ch12: ExtiChannel<12>,
    pub ch13: ExtiChannel<13>,
    pub ch14: ExtiChannel<14>,
    pub ch15: ExtiChannel<15>,
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// NVIC interrupt number of interrupt from this pin
    ///
    /// Used to unmask / enable the interrupt with [`cortex_m::peripheral::NVIC::unmask()`].
    /// This is also useful for all other [`cortex_m::peripheral::NVIC`] functions.
    pub const fn interrupt(&self) -> Interrupt {
        match N {
            0 => Interrupt::EXTI0,
            1 => Interrupt::EXTI1,
            2 => Interrupt::EXTI2,
            3 => Interrupt::EXTI3,
            4 => Interrupt::EXTI4,
            5 => Interrupt::EXTI5,
            6 => Interrupt::EXTI6,
            7 => Interrupt::EXTI7,
            8 => Interrupt::EXTI8,
            9 => Interrupt::EXTI9,
            10 => Interrupt::EXTI10,
            11 => Interrupt::EXTI11,
            12 => Interrupt::EXTI12,
            13 => Interrupt::EXTI13,
            14 => Interrupt::EXTI14,
            15 => Interrupt::EXTI15,
            _ => panic!("Unsupported pin number"),
        }
    }
}

/// External Interrupt Pin
pub trait ExtiPin<const P: char, const N: u8, M> {
    fn make_interrupt_source(
        self,
        _ch: ExtiChannel<N>,
        ch: &mut Exti,
    ) -> Pin<P, N, M, true>;
}

// TODO: Find better name
/// Only available on pins where interrupts have been enabled by the user
pub trait ExtiedPin<const N: u8> {
    fn trigger_on_edge(&mut self, exti: &mut Exti, level: Edge);
    fn enable_event(&mut self, exti: &mut Exti);
    fn disable_event(&mut self, exti: &mut Exti);
    fn enable_interrupt(&mut self, exti: &mut Exti);
    fn disable_interrupt(&mut self, exti: &mut Exti);
    fn clear_interrupt_pending_bit(&mut self, edge: Edge);
    fn check_interrupt(&self, edge: Edge) -> bool;
}

impl<const P: char, const N: u8, M> ExtiPin<P, N, M>
    for gpio::Pin<P, N, M, false>
where
    M: marker::Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    #[inline(always)]
    fn make_interrupt_source(
        self,
        _ch: ExtiChannel<N>,
        exti: &mut Exti,
    ) -> Pin<P, N, M, true> {
        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 8 * (i % 4);
        match i {
            0..=3 => {
                exti.exticr1().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                exti.exticr2().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                exti.exticr3().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                exti.exticr4().modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }

        Pin { _mode: PhantomData }
    }
}

impl<const P: char, const N: u8, M> ExtiedPin<N> for gpio::Pin<P, N, M, true> {
    /// Generate interrupt on rising edge, falling edge or both
    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut Exti, edge: Edge) {
        let i = N;
        match edge {
            Edge::Rising => {
                exti.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::Falling => {
                exti.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::RisingFalling => {
                exti.rtsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_event(&mut self, exti: &mut Exti) {
        exti.emr1()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << N)) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_event(&mut self, exti: &mut Exti) {
        exti.emr1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << N)) });
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut Exti) {
        exti.imr1()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << N)) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut Exti) {
        exti.imr1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << N)) });
    }

    /// Clear the interrupt pending bit for this pin
    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self, edge: Edge) {
        unsafe {
            let exti = &(*EXTI::ptr());

            let mask = 1 << N;
            match edge {
                Edge::Rising => exti.rpr1().write(|w| w.bits(mask)),
                Edge::Falling => exti.fpr1().write(|w| w.bits(mask)),
                _ => panic!("Must choose a rising or falling edge"),
            };
        }
    }

    /// Reads the interrupt pending bit for this pin
    #[inline(always)]
    fn check_interrupt(&self, edge: Edge) -> bool {
        unsafe {
            let exti = &(*EXTI::ptr());

            let bits = match edge {
                Edge::Rising => exti.rpr1().read().bits(),
                Edge::Falling => exti.fpr1().read().bits(),
                _ => panic!("Must choose a rising or falling edge"),
            };

            bits & (1 << N) != 0
        }
    }
}
