use core::marker::PhantomData;

use crate::gpio::{self, Alternate};
use crate::rcc::{rec, CoreClocks};
use crate::stm32::{rcc::ccipr1, usart1, RCC, USART1, USART2, USART3};
use crate::time::Hertz;

use super::{Instance, InstanceClock, NoCk, NoRx, NoTx, PinCk, PinRx, PinTx};

macro_rules! usart_pins {
    ($($USARTX:ty: TX: [$($TX:ty),*] RX: [$($RX:ty),*] CK: [$($CK:ty),*])+) => {
        $(
            $(
                impl PinTx<$USARTX> for $TX {}
            )*
            $(
                impl PinRx<$USARTX> for $RX {}
            )*
            $(
                impl PinCk<$USARTX> for $CK {}
            )*
        )+
    }
}

macro_rules! instance_clock {
    ($USARTX:ident: $pclk:ident, $cciprX:ident [$($pll:ident:$pll_clk:ident),+] [$($clk:ident$(:$ext:ident)?),*]) => { paste::item! {
        impl InstanceClock for $USARTX {
            fn clock(clocks: &CoreClocks) -> Hertz {
                let $cciprX = unsafe { (*RCC::ptr()).$cciprX().read() };

                match $cciprX.[<$USARTX:lower sel>]().variant() {
                    Some($cciprX::USARTSEL::Pclk) => Some(clocks.$pclk()),
                    $(
                        Some($cciprX::USARTSEL::[<$pll:camel $pll_clk:upper>]) => clocks.[< $pll:lower >]().[< $pll_clk:lower:snake _ck >](),
                    )+
                    $(
                        Some($cciprX::USARTSEL::[<$clk:camel $($ext)?>]) => clocks.[< $clk:lower _ck >](),
                    )*
                    _ => unreachable!(),
                }.expect("Source clock not enabled")
            }
        }
    }}
}

// Implemented by all USART instances
macro_rules! instances {
    ($($USARTX:ident),+) => { paste::item! {
        $(
            impl Instance for $USARTX {
                type Rec = rec::[<$USARTX:camel>];

                fn ptr() -> *const usart1::RegisterBlock {
                    <$USARTX>::ptr() as *const _
                }

                fn rec() -> Self::Rec {
                    rec::[< $USARTX:camel >] { _marker: PhantomData }
                }
            }

            impl crate::Sealed for $USARTX {}
        )+
    }};
}

instances!(USART1, USART2, USART3);

#[cfg(feature = "rm0492")]
mod rm492 {
    use super::*;

    instance_clock!(USART1: pclk2, ccipr1 [Pll2:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART2: pclk1, ccipr1 [Pll2:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART3: pclk1, ccipr1 [Pll2:Q] [Hsi:Ker, Csi:Ker, Lse]);

    usart_pins! {
        USART1:
            TX: [
                NoTx,
                gpio::PA2<Alternate<8>>,
                gpio::PA9<Alternate<7>>,
                gpio::PA12<Alternate<8>>,
                gpio::PA14<Alternate<7>>,
                gpio::PB6<Alternate<7>>,
                gpio::PB14<Alternate<4>>
            ]
            RX: [
                NoRx,
                gpio::PA1<Alternate<8>>,
                gpio::PA10<Alternate<7>>,
                gpio::PA11<Alternate<8>>,
                gpio::PA13<Alternate<7>>,
                gpio::PB7<Alternate<7>>,
                gpio::PB15<Alternate<4>>
            ]
            CK: [
                NoCk,
                gpio::PA3<Alternate<8>>,
                gpio::PA8<Alternate<7>>,
                gpio::PB8<Alternate<7>>,
                gpio::PB12<Alternate<8>>
            ]
        USART2:
            TX: [
                NoTx,
                gpio::PA2<Alternate<7>>,
                gpio::PA5<Alternate<9>>,
                gpio::PA8<Alternate<4>>,
                gpio::PA12<Alternate<4>>,
                gpio::PA14<Alternate<9>>,
                gpio::PB0<Alternate<9>>,
                gpio::PB4<Alternate<13>>,
                gpio::PC6<Alternate<13>>
            ]
            RX: [
                NoRx,
                gpio::PA3<Alternate<7>>,
                gpio::PA11<Alternate<4>>,
                gpio::PA13<Alternate<9>>,
                gpio::PA15<Alternate<9>>,
                gpio::PB1<Alternate<9>>,
                gpio::PB5<Alternate<13>>,
                gpio::PB15<Alternate<13>>,
                gpio::PC7<Alternate<13>>
            ]
            CK: [
                NoCk,
                gpio::PA4<Alternate<7>>,
                gpio::PA15<Alternate<4>>,
                gpio::PA1<Alternate<9>>,
                gpio::PB2<Alternate<9>>,
                gpio::PB6<Alternate<13>>,
                gpio::PC8<Alternate<13>>
            ]
        USART3:
            TX: [
                NoTx,
                gpio::PB10<Alternate<7>>,
                gpio::PC10<Alternate<7>>,
                gpio::PA4<Alternate<13>>,
                gpio::PA8<Alternate<13>>,
                gpio::PB3<Alternate<13>>,
                gpio::PB7<Alternate<13>>
            ]
            RX: [
                NoRx,
                gpio::PC4<Alternate<7>>,
                gpio::PC11<Alternate<7>>,
                gpio::PA3<Alternate<13>>,
                gpio::PA5<Alternate<13>>,
                gpio::PA12<Alternate<13>>,
                gpio::PA15<Alternate<13>>,
                gpio::PB8<Alternate<13>>
            ]
            CK: [
                NoCk,
                gpio::PB12<Alternate<7>>,
                gpio::PC12<Alternate<7>>,
                gpio::PA0<Alternate<13>>,
                gpio::PA7<Alternate<13>>,
                gpio::PA9<Alternate<13>>,
                gpio::PB10<Alternate<13>>
            ]
    }
}

// Note: pin data is taken from stm32h56x, stm32h573, stm32h523 and stm32h533 datasheets
#[cfg(feature = "rm0481")]
mod rm0481_common {
    use crate::stm32::{UART4, UART5, USART6};

    use super::*;

    instance_clock!(USART1: pclk2, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART2: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART3: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(UART4: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(UART5: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART6: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);

    instances! { UART4, UART5, USART6 }

    usart_pins! {
        USART1:
            TX: [
                NoTx,
                gpio::PA9<Alternate<7>>,
                gpio::PA15<Alternate<7>>,
                gpio::PB6<Alternate<7>>,
                gpio::PB14<Alternate<4>>
            ]
            RX: [
                NoRx,
                gpio::PA10<Alternate<7>>,
                gpio::PB7<Alternate<7>>,
                gpio::PB15<Alternate<4>>
            ]
            CK: [
                NoCk,
                gpio::PA8<Alternate<7>>
            ]
        USART2:
            TX: [
                NoTx,
                gpio::PA2<Alternate<7>>,
                gpio::PB0<Alternate<7>>,
                gpio::PD5<Alternate<7>>
            ]
            RX: [
                NoRx,
                gpio::PA3<Alternate<7>>,
                gpio::PD6<Alternate<7>>
            ]
            CK: [
                NoCk,
                gpio::PA4<Alternate<7>>,
                gpio::PD7<Alternate<7>>
            ]
        USART3:
            TX: [
                NoTx,
                gpio::PB10<Alternate<7>>,
                gpio::PC10<Alternate<7>>,
                gpio::PD8<Alternate<7>>
            ]
            RX: [
                NoRx,
                gpio::PB1<Alternate<7>>,
                gpio::PC4<Alternate<7>>,
                gpio::PD9<Alternate<7>>,
                gpio::PC11<Alternate<7>>
            ]
            CK: [
                NoCk,
                gpio::PB12<Alternate<7>>,
                gpio::PC12<Alternate<7>>,
                gpio::PD10<Alternate<7>>
            ]
        UART4:
            TX: [
                NoTx,
                gpio::PA0<Alternate<8>>,
                gpio::PA12<Alternate<6>>,
                gpio::PB9<Alternate<8>>,
                gpio::PC10<Alternate<8>>,
                gpio::PD1<Alternate<8>>,
                gpio::PD12<Alternate<8>>
            ]
            RX: [
                NoRx,
                gpio::PA1<Alternate<8>>,
                gpio::PA11<Alternate<6>>,
                gpio::PC11<Alternate<8>>,
                gpio::PD11<Alternate<8>>,
                gpio::PD0<Alternate<8>>,
                gpio::PB8<Alternate<8>>
            ]
            CK: [
                NoCk
            ]
        UART5:
            TX: [
                NoTx,
                gpio::PB13<Alternate<14>>,
                gpio::PC12<Alternate<8>>,
                gpio::PB3<Alternate<14>>,
                gpio::PB6<Alternate<14>>
            ]
            RX: [
                NoRx,
                gpio::PB12<Alternate<14>>,
                gpio::PB15<Alternate<14>>,
                gpio::PD2<Alternate<8>>,
                gpio::PB5<Alternate<14>>
            ]
            CK: [
                NoCk
            ]
        USART6:
            TX: [
                NoTx,
                gpio::PC6<Alternate<7>>,
                gpio::PG14<Alternate<7>>,
                gpio::PB5<Alternate<6>>
            ]
            RX: [
                NoRx,
                gpio::PC7<Alternate<7>>,
                gpio::PG9<Alternate<7>>,
                gpio::PB6<Alternate<6>>
            ]
            CK: [
                NoCk,
                gpio::PA1<Alternate<14>>,
                gpio::PG7<Alternate<7>>,
                gpio::PC8<Alternate<7>>
            ]
    }
}

#[cfg(feature = "h56x_h573")]
mod h56x_h573 {
    use crate::stm32::{
        rcc::ccipr2, UART12, UART7, UART8, UART9, USART10, USART11,
    };

    use super::*;

    instance_clock!(UART7: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(UART8: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(UART9: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART10: pclk1, ccipr1 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(USART11: pclk1, ccipr2 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instance_clock!(UART12: pclk1, ccipr2 [Pll2:Q, Pll3:Q] [Hsi:Ker, Csi:Ker, Lse]);
    instances!(UART7, UART8, UART9, USART10, USART11, UART12);

    usart_pins! {
        UART7:
            TX: [
                NoTx,
                gpio::PF7<Alternate<7>>,
                gpio::PE8<Alternate<7>>,
                gpio::PA15<Alternate<11>>,
                gpio::PB4<Alternate<11>>
            ]
            RX: [
                NoRx,
                gpio::PF6<Alternate<7>>,
                gpio::PE7<Alternate<7>>,
                gpio::PA8<Alternate<11>>,
                gpio::PB3<Alternate<11>>
            ]
            CK: [
                NoCk
            ]
        UART8:
            TX: [
                NoTx,
                gpio::PE2<Alternate<8>>,
                gpio::PH13<Alternate<7>>,
                gpio::PE1<Alternate<8>>
            ]
            RX: [
                NoRx,
                gpio::PE0<Alternate<8>>
            ]
            CK: [
                NoCk
            ]
        UART9:
            TX: [
                NoTx,
                gpio::PG1<Alternate<11>>,
                gpio::PD15<Alternate<11>>
            ]
            RX: [
                NoRx,
                gpio::PG0<Alternate<11>>,
                gpio::PD14<Alternate<11>>
            ]
            CK: [
                NoCk
            ]
        USART10:
            TX: [
                NoTx,
                gpio::PE3<Alternate<7>>,
                gpio::PG12<Alternate<6>>
            ]
            RX: [
                NoRx,
                gpio::PE2<Alternate<7>>,
                gpio::PG11<Alternate<6>>
            ]
            CK: [
                NoCk,
                gpio::PE15<Alternate<7>>,
                gpio::PG15<Alternate<6>>
            ]
        USART11:
            TX: [
                NoTx,
                gpio::PF3<Alternate<7>>,
                gpio::PA6<Alternate<7>>
            ]
            RX: [
                NoRx,
                gpio::PF4<Alternate<7>>,
                gpio::PA7<Alternate<7>>
            ]
            CK: [
                NoCk,
                gpio::PF2<Alternate<7>>,
                gpio::PB0<Alternate<7>>
            ]
        UART12:
            TX: [
                NoTx,
                gpio::PF2<Alternate<6>>,
                gpio::PE10<Alternate<6>>,
                gpio::PG3<Alternate<7>>
            ]
            RX: [
                NoRx,
                gpio::PF5<Alternate<6>>,
                gpio::PE9<Alternate<6>>,
                gpio::PG2<Alternate<7>>
            ]
            CK: [
                NoCk
            ]
    }
}
