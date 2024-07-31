use super::{marker, Edge, Pin, PinExt};
use crate::pac::{Interrupt, EXTI};

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
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, exti: &mut EXTI);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge);
    fn enable_event(&mut self, exti: &mut EXTI);
    fn disable_event(&mut self, exti: &mut EXTI);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self, edge: Edge);
    fn check_interrupt(&self, edge: Edge) -> bool;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: marker::Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    #[inline(always)]
    fn make_interrupt_source(&mut self, exti: &mut EXTI) {
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
    }

    /// Generate interrupt on rising edge, falling edge or both
    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        let i = self.pin_id();
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
    fn enable_event(&mut self, exti: &mut EXTI) {
        exti.emr1()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_event(&mut self, exti: &mut EXTI) {
        exti.emr1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1()
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut EXTI) {
        exti.imr1()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self, edge: Edge) {
        unsafe {
            let exti = &(*EXTI::ptr());

            let mask = 1 << self.pin_id();
            match edge {
                Edge::Rising => exti.rpr1().write(|w| w.bits(mask)),
                Edge::Falling => exti.fpr1().write(|w| w.bits(mask)),
                _ => panic!("Must choose a rising or falling edge"),
            }
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

            bits & (1 << self.pin_id()) != 0
        }
    }
}
