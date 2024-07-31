use super::Gpio;

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $Rec:ident, $PEPin:ident, $port_id:expr, $PXn:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, [$($A:literal),*] $(, $MODE:ty)?),)+
    ]) => {
        #[doc=concat!("Port ", $port_id)]
        pub mod $gpiox {
            use crate::pac::$GPIOX;
            use crate::rcc::{rec, ResetEnable};

            /// GPIO parts
            pub struct Parts {
                $(
                    /// Pin
                    pub $pxi: $PXi $(<$MODE>)?,
                )+
            }

            impl crate::gpio::GpioExt for $GPIOX {
                type Parts = Parts;
                type Rec = rec::$Rec;

                fn split(self, prec: rec::$Rec) -> Parts {
                    prec.enable().reset();

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }

                fn split_without_reset(self, prec: rec::$Rec) -> Parts {
                    prec.enable();

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }

            #[doc=concat!("Common type for GPIO", $port_id, " related pins")]
            pub type $PXn<MODE> = crate::gpio::PartiallyErasedPin<$port_id, MODE>;

            $(
                #[doc=concat!("P", $port_id, $i, " pin")]
                pub type $PXi<MODE = crate::gpio::Analog> = crate::gpio::Pin<$port_id, $i, MODE>;

                $(
                    impl<MODE> crate::gpio::marker::IntoAf<$A> for $PXi<MODE> { }
                )*
            )+

        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

#[cfg(feature = "gpio-h503")]
pub use h503::*;

#[cfg(feature = "gpio-h503")]
mod h503 {
    use super::Gpio;

    gpio!(GPIOA, gpioa, Gpioa, PA, 'A', PAn, [
        PA0: (pa0, 0, [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
        PA1: (pa1, 1, [1, 4, 5, 6, 7, 8, 9, 11, 14, 15]),
        PA2: (pa2, 2, [1, 2, 3, 4, 6, 7, 8, 14, 15]),
        PA3: (pa3, 3, [1, 3, 4, 5, 6, 7, 8, 13, 14, 15]),
        PA4: (pa4, 4, [1, 3, 4, 5, 6, 7, 8, 10, 13, 14, 15]),
        PA5: (pa5, 5, [1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15]),
        PA6: (pa6, 6, [1, 2, 5, 15]),
        PA7: (pa7, 7, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15]),
        PA8: (pa8, 8, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
        PA9: (pa9, 9, [0, 1, 3, 4, 5, 7, 10, 13, 15]),
        PA10: (pa10, 10, [1, 3, 4, 7, 15]),
        PA11: (pa11, 11, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15]),
        PA12: (pa12, 12, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 14, 15]),
        PA13: (pa13, 13, [0, 1, 2, 7, 8, 9, 12, 14, 15], crate::gpio::Debugger),
        PA14: (pa14, 14, [0, 1, 2, 3, 4, 7, 8, 9, 14, 15], crate::gpio::Debugger),
        PA15: (pa15, 15, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15], crate::gpio::Debugger),
    ]);

    gpio!(GPIOB, gpiob, Gpiob, PB, 'B', PBn, [
        PB0: (pb0, 0, [1, 2, 4, 9, 14, 15]),
        PB1: (pb1, 1, [1, 2, 3, 5, 6, 9, 12, 14, 15]),
        PB2: (pb2, 2, [0, 1, 2, 4, 5, 6, 7, 9, 14, 15]),
        PB3: (pb3, 3, [0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15], crate::gpio::Debugger),
        PB4: (pb4, 4, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 13, 14, 15], crate::gpio::Debugger),
        PB5: (pb5, 5, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15]),
        PB6: (pb6, 6, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 13, 14, 15]),
        PB7: (pb7, 7, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 13, 14, 15]),
        PB8: (pb8, 8, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
        PB10: (pb10, 10, [1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 14, 15]),
        PB12: (pb12, 12, [1, 4, 5, 7, 8, 9, 15]),
        PB13: (pb13, 13, [1, 3, 4, 5, 7, 8, 9, 10, 11, 15]),
        PB14: (pb14, 14, [1, 2, 4, 5, 7, 8, 15]),
        PB15: (pb15, 15, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    ]);

    gpio!(GPIOC, gpioc, Gpioc, PC, 'C', PCn, [
        PC0: (pc0, 0, [1, 2, 5, 7, 15]),
        PC1: (pc1, 1, [0, 2, 4, 5, 7, 8, 14, 15]),
        PC2: (pc2, 2, [0, 4, 5, 7, 8, 15]),
        PC3: (pc3, 3, [0, 3, 4, 5, 7, 14, 15]),
        PC4: (pc4, 4, [1, 3, 4, 5, 7, 15]),
        PC5: (pc5, 5, [1, 5, 12, 14, 15]),
        PC6: (pc6, 6, [1, 2, 3, 4, 5, 8, 9, 13, 15]),
        PC7: (pc7, 7, [0, 1, 2, 3, 5, 6, 8, 9, 13, 15]),
        PC8: (pc8, 8, [0, 1, 2, 3, 4, 5, 8, 9, 13, 15]),
        PC9: (pc9, 9, [0, 1, 2, 3, 4, 5, 6, 7, 9, 13, 15]),
        PC10: (pc10, 10, [1, 3, 5, 6, 7, 8, 9, 13, 15]),
        PC11: (pc11, 11, [1, 3, 4, 5, 6, 7, 8, 14, 15]),
        PC12: (pc12, 12, [0, 1, 2, 3, 6, 7, 8, 14, 15]),
        PC13: (pc13, 13, [15]),
        PC14: (pc14, 14, [15]),
        PC15: (pc15, 15, [15]),
    ]);

    gpio!(GPIOD, gpiod, Gpiod, PD, 'D', PDn, [
        PD2: (pd2, 2, [0, 1, 2, 6, 7, 9, 14, 15]),
    ]);

    gpio!(GPIOH, gpioh, Gpioh, PH, 'H', PHn, [
        PH0: (ph0, 0, [15]),
        PH1: (ph1, 1, [15]),
    ]);

    impl<const P: char> Gpio<P> {
        pub(crate) const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
            match P {
                'A' => crate::pac::GPIOA::ptr(),
                'B' => crate::pac::GPIOB::ptr() as _,
                'C' => crate::pac::GPIOC::ptr() as _,
                'D' => crate::pac::GPIOD::ptr() as _,
                'H' => crate::pac::GPIOH::ptr() as _,
                _ => panic!("Unknown GPIO port"),
            }
        }
    }
}

#[cfg(feature = "gpio-h5x")]
pub use h5x::*;

#[cfg(feature = "gpio-h5x")]
mod h5x {
    use super::Gpio;

    impl<const P: char> Gpio<P> {
        pub(crate) const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
            match P {
                'A' => crate::pac::GPIOA::ptr(),
                'B' => crate::pac::GPIOB::ptr() as _,
                'C' => crate::pac::GPIOC::ptr() as _,
                'D' => crate::pac::GPIOD::ptr() as _,
                'E' => crate::pac::GPIOE::ptr() as _,
                'F' => crate::pac::GPIOF::ptr() as _,
                'G' => crate::pac::GPIOG::ptr() as _,
                'H' => crate::pac::GPIOH::ptr() as _,
                'I' => crate::pac::GPIOI::ptr() as _,
                _ => panic!("Unknown GPIO port"),
            }
        }
    }
}
