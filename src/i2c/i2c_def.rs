use core::marker::PhantomData;

use crate::gpio::{self, Alternate, OpenDrain};
use crate::rcc::{rec, CoreClocks};
use crate::stm32::{self, i2c1, rcc::ccipr4};
use crate::time::Hertz;

use super::{Instance, PinScl, PinSda};

macro_rules! pins {
    ($($I2CX:ty: SCL: [$($SCL:ty),*] SDA: [$($SDA:ty),*])+) => {
        $(
            $(
                impl PinScl<$I2CX> for $SCL { }
            )*
            $(
                impl PinSda<$I2CX> for $SDA { }
            )*
        )+
    }
}

// Implemented by all I2C instances
macro_rules! i2c {
    ($I2CX:ty: $pclk:ident, $pllX:ident) => {

        paste::item! {
            impl Instance for $I2CX {
                type Rec = rec::[< $I2CX:lower:camel >];

                fn ptr() -> *const i2c1::RegisterBlock {
                    <$I2CX>::ptr() as *const _
                }

                fn clock(clocks: &CoreClocks) -> Hertz {
                    let ccipr4 = unsafe { (*stm32::RCC::ptr()).ccipr4().read() };

                    match ccipr4.[<$I2CX:lower sel>]().variant() {
                        ccipr4::I2CSEL::Pclk => Some(clocks.$pclk()),
                        ccipr4::I2CSEL::[< $pllX:camel R>] => clocks.[< $pllX:lower >]().r_ck(),
                        ccipr4::I2CSEL::HsiKer => clocks.hsi_ck(),
                        ccipr4::I2CSEL::CsiKer => clocks.csi_ck(),
                    }.expect("Source clock not enabled")
                }

                fn rec() -> Self::Rec {
                    rec::[< $I2CX:lower:camel >] { _marker: PhantomData }
                }
            }

            impl crate::Sealed for $I2CX {}
        }
    };
}

#[cfg(feature = "rm0492")]
mod rm492 {
    use super::*;
    use crate::stm32::{I2C1, I2C2};

    pins! {
        I2C1:
            SCL: [
                gpio::PB6<Alternate<4, OpenDrain>>,
                gpio::PB8<Alternate<4, OpenDrain>>,
                gpio::PC8<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB5<Alternate<11, OpenDrain>>,
                gpio::PB7<Alternate<4, OpenDrain>>,
                gpio::PB10<Alternate<11, OpenDrain>>,
                gpio::PC9<Alternate<4, OpenDrain>>
            ]

        I2C2:
            SCL: [
                gpio::PB3<Alternate<8, OpenDrain>>,
                gpio::PB5<Alternate<4, OpenDrain>>,
                gpio::PB10<Alternate<4, OpenDrain>>,
                gpio::PC6<Alternate<8, OpenDrain>>,
                gpio::PC10<Alternate<8, OpenDrain>>
            ]

            SDA: [
                gpio::PB3<Alternate<4, OpenDrain>>,
                gpio::PB4<Alternate<8, OpenDrain>>,
                gpio::PB8<Alternate<8, OpenDrain>>,
                gpio::PB13<Alternate<4, OpenDrain>>,
                gpio::PC7<Alternate<8, OpenDrain>>,
                gpio::PC11<Alternate<8, OpenDrain>>
            ]
    }

    i2c! { I2C1: pclk1, Pll2 }
    i2c! { I2C2: pclk1, Pll2 }
}

#[cfg(feature = "h523_h533")]
mod h523_h533 {
    use super::*;
    use crate::stm32::{I2C1, I2C2, I2C3};

    pins! {
        I2C1:
            SCL: [
                gpio::PB6<Alternate<4, OpenDrain>>,
                gpio::PB8<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB7<Alternate<4, OpenDrain>>,
                gpio::PB9<Alternate<4, OpenDrain>>
            ]

        I2C2:
            SCL: [
                gpio::PB10<Alternate<4, OpenDrain>>,
                gpio::PF1<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB3<Alternate<4, OpenDrain>>,
                gpio::PB12<Alternate<4, OpenDrain>>,
                gpio::PF0<Alternate<4, OpenDrain>>
            ]

        I2C3:
            SCL: [
                gpio::PA8<Alternate<4, OpenDrain>>,
                gpio::PD6<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB4<Alternate<9, OpenDrain>>,
                gpio::PC9<Alternate<4, OpenDrain>>,
                gpio::PD7<Alternate<4, OpenDrain>>
            ]
    }

    i2c! { I2C1: pclk1, Pll3 }
    i2c! { I2C2: pclk1, Pll3 }
    i2c! { I2C3: pclk3, Pll3 }
}

#[cfg(feature = "h56x_h573")]
mod h56x_h573 {
    use super::*;
    use crate::stm32::{I2C1, I2C2, I2C3, I2C4};

    pins! {
        I2C1:
            SCL: [
                gpio::PB6<Alternate<4, OpenDrain>>,
                gpio::PB8<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB7<Alternate<4, OpenDrain>>,
                gpio::PB9<Alternate<4, OpenDrain>>
            ]

        I2C2:
            SCL: [
                gpio::PB10<Alternate<4, OpenDrain>>,
                gpio::PF1<Alternate<4, OpenDrain>>,
                gpio::PH4<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB3<Alternate<4, OpenDrain>>,
                gpio::PB11<Alternate<4, OpenDrain>>,
                gpio::PB12<Alternate<4, OpenDrain>>,
                gpio::PF0<Alternate<4, OpenDrain>>,
                gpio::PH5<Alternate<4, OpenDrain>>
            ]

        I2C3:
            SCL: [
                gpio::PA8<Alternate<4, OpenDrain>>,
                gpio::PH7<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PC9<Alternate<4, OpenDrain>>,
                gpio::PH8<Alternate<4, OpenDrain>>
            ]

        I2C4:
            SCL: [
                gpio::PB6<Alternate<6, OpenDrain>>,
                gpio::PB8<Alternate<6, OpenDrain>>,
                gpio::PD12<Alternate<4, OpenDrain>>,
                gpio::PF5<Alternate<4, OpenDrain>>,
                gpio::PG7<Alternate<4, OpenDrain>>,
                gpio::PH11<Alternate<4, OpenDrain>>
            ]

            SDA: [
                gpio::PB7<Alternate<6, OpenDrain>>,
                gpio::PB9<Alternate<6, OpenDrain>>,
                gpio::PD13<Alternate<4, OpenDrain>>,
                gpio::PF15<Alternate<4, OpenDrain>>,
                gpio::PG6<Alternate<4, OpenDrain>>,
                gpio::PH12<Alternate<4, OpenDrain>>
            ]
    }

    i2c! { I2C1: pclk1, Pll3 }
    i2c! { I2C2: pclk1, Pll3 }
    i2c! { I2C3: pclk3, Pll3 }
    i2c! { I2C4: pclk3, Pll3 }
}
