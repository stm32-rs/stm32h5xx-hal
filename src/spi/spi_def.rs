use core::marker::PhantomData;

use super::{
    Instance, NoMiso, NoMosi, NoSck, PinHCS, PinMiso, PinMosi, PinSck,
    SupportedWordSize,
};
use crate::gpio::{self, Alternate};
use crate::rcc::{rec, CoreClocks};
use crate::stm32::{self, rcc::ccipr3, spi1, SPI1, SPI2, SPI3};
use crate::time::Hertz;

macro_rules! pins {
    ($($SPIX:ty:
       SCK: [$($( #[ $pmeta1:meta ] )* $SCK:ty),*]
       MISO: [$($( #[ $pmeta2:meta ] )* $MISO:ty),*]
       MOSI: [$($( #[ $pmeta3:meta ] )* $MOSI:ty),*]
       HCS: [$($( #[ $pmeta4:meta ] )* $HCS:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl PinSck<$SPIX> for $SCK {}
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl PinMiso<$SPIX> for $MISO {}
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl PinMosi<$SPIX> for $MOSI {}
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl PinHCS<$SPIX> for $HCS {}
            )*
        )+
    }
}

macro_rules! spi123_clock {
    ($SPIX:ident) => { paste::item! {
        fn clock(clocks: &CoreClocks) -> Hertz {
            let ccipr3 = unsafe { (*stm32::RCC::ptr()).ccipr3().read() };
            let ck_sel = ccipr3.[<$SPIX:lower sel>]().variant().expect("No source clock selected");
            match ck_sel {
                ccipr3::SPI123SEL::Pll1Q => clocks.pll1().q_ck(),
                ccipr3::SPI123SEL::Pll2P => clocks.pll2().p_ck(),
                ccipr3::SPI123SEL::Audioclk => clocks.audio_ck(),
                ccipr3::SPI123SEL::PerCk => clocks.per_ck(),
                #[cfg(feature = "rm0481")]
                ccipr3::SPI123SEL::Pll3P => clocks.pll3().p_ck(),
            }.expect("Source clock not enabled")
        }
    }}
}

#[allow(unused_macros)]
macro_rules! spi456_clock {
    ($SPIX:ident, $pclk:ident) => { paste::item! {
        fn clock(clocks: &CoreClocks) -> Hertz {
            let ccipr3 = unsafe { (*stm32::RCC::ptr()).ccipr3().read() };
            let ck_sel = ccipr3.[<$SPIX:lower sel>]().variant().expect("No source clock selected");
            match ck_sel {
                ccipr3::SPI456SEL::Pclk => Some(clocks.$pclk()),
                ccipr3::SPI456SEL::Pll2Q => clocks.pll2().q_ck(),
                ccipr3::SPI456SEL::Pll3Q => clocks.pll3().q_ck(),
                ccipr3::SPI456SEL::HsiKer => clocks.hsi_ck(),
                ccipr3::SPI456SEL::CsiKer => clocks.csi_ck(),
                ccipr3::SPI456SEL::Hse => clocks.hse_ck(),
            }.expect("Source clock not enabled")
        }
    }}
}

// Implemented by all SPI instances
macro_rules! instance {
    ($SPIX:ident: $Spi:ident[$($word:ty),+], $SPICLKSEL:ident$(, $pclk:ident)?) => { paste::item! {
        impl Instance for $SPIX {
            type Rec = rec::$Spi;

            fn ptr() -> *const spi1::RegisterBlock {
                <$SPIX>::ptr() as *const _
            }

            [< $SPICLKSEL:lower _clock >]!($SPIX$(, $pclk)?);

            fn rec() -> Self::Rec {
                rec::$Spi { _marker: PhantomData }
            }
        }

        impl crate::Sealed for $SPIX {}
        $(impl SupportedWordSize<$word> for $SPIX {})+
    }};

}

instance! { SPI1: Spi1[u8, u16, u32], SPI123 }
instance! { SPI2: Spi2[u8, u16, u32], SPI123 }
instance! { SPI3: Spi3[u8, u16, u32], SPI123 }

#[cfg(feature = "rm0492")]
mod rm492 {
    use super::*;

    pins! {
        SPI1:
            SCK: [
                NoSck,
                gpio::PA2<Alternate<4>>,
                gpio::PA5<Alternate<5>>,
                gpio::PA8<Alternate<12>>,
                gpio::PB3<Alternate<5>>,
                gpio::PC0<Alternate<5>>,
                gpio::PC5<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                gpio::PA0<Alternate<12>>,
                gpio::PA3<Alternate<4>>,
                gpio::PA6<Alternate<5>>,
                gpio::PA9<Alternate<4>>,
                gpio::PB4<Alternate<5>>,
                gpio::PC2<Alternate<4>>,
                gpio::PC10<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                gpio::PA4<Alternate<4>>,
                gpio::PA7<Alternate<5>>,
                gpio::PB5<Alternate<5>>,
                gpio::PC3<Alternate<4>>,
                gpio::PC7<Alternate<5>>
            ]
            HCS: [
                gpio::PA1<Alternate<4>>,
                gpio::PA4<Alternate<5>>,
                gpio::PA15<Alternate<5>>,
                gpio::PB8<Alternate<12>>,
                gpio::PC1<Alternate<4>>,
                gpio::PC8<Alternate<5>>
            ]
        SPI2:
            SCK: [
                NoSck,
                gpio::PA5<Alternate<7>>,
                gpio::PA9<Alternate<5>>,
                gpio::PA12<Alternate<5>>,
                gpio::PB2<Alternate<6>>,
                gpio::PB10<Alternate<5>>,
                gpio::PB13<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                gpio::PA7<Alternate<11>>,
                gpio::PA15<Alternate<7>>,
                gpio::PB5<Alternate<6>>,
                gpio::PB14<Alternate<5>>,
                gpio::PC2<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                gpio::PA8<Alternate<6>>,
                gpio::PB1<Alternate<6>>,
                gpio::PB15<Alternate<5>>,
                gpio::PC1<Alternate<5>>,
                gpio::PC3<Alternate<5>>
            ]
            HCS: [
                gpio::PA3<Alternate<5>>,
                gpio::PA8<Alternate<11>>,
                gpio::PA11<Alternate<5>>,
                gpio::PB4<Alternate<7>>,
                gpio::PB12<Alternate<5>>
            ]
        SPI3:
            SCK: [
                NoSck,
                gpio::PA1<Alternate<6>>,
                gpio::PA15<Alternate<10>>,
                gpio::PB3<Alternate<6>>,
                gpio::PB7<Alternate<6>>,
                gpio::PC10<Alternate<6>>
            ]
            MISO: [
                NoMiso,
                gpio::PA2<Alternate<6>>,
                gpio::PA4<Alternate<10>>,
                gpio::PB4<Alternate<6>>,
                gpio::PB15<Alternate<6>>,
                gpio::PC11<Alternate<6>>
            ]
            MOSI: [
                NoMosi,
                gpio::PA3<Alternate<6>>,
                gpio::PA5<Alternate<10>>,
                gpio::PB2<Alternate<7>>,
                gpio::PB5<Alternate<7>>,
                gpio::PC12<Alternate<6>>,
                gpio::PA9<Alternate<10>>
            ]
            HCS: [
                gpio::PA4<Alternate<6>>,
                gpio::PA15<Alternate<6>>,
                gpio::PB10<Alternate<6>>,
                gpio::PA0<Alternate<10>>,
                gpio::PD2<Alternate<6>>
            ]
    }
}

// Note: pin data is taken from stm32h56x, stm32h573, stm32h523 and stm32h533 datasheets
#[cfg(feature = "rm0481")]
mod rm0481_common {
    use super::*;
    use crate::stm32::{SPI1, SPI2, SPI3, SPI4};

    pins! {
        SPI1:
            SCK: [
                NoSck,
                gpio::PA5<Alternate<5>>,
                gpio::PB3<Alternate<5>>,
                gpio::PG11<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                gpio::PA6<Alternate<5>>,
                gpio::PB4<Alternate<5>>,
                gpio::PG9<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                gpio::PA7<Alternate<5>>,
                gpio::PB5<Alternate<5>>,
                #[cfg(feature = "h523_h533")]
                gpio::PB15<Alternate<6>>,
                gpio::PD7<Alternate<5>>
            ]
            HCS: [
                gpio::PA4<Alternate<5>>,
                gpio::PA15<Alternate<5>>,
                gpio::PG10<Alternate<5>>
            ]
        SPI2:
            SCK: [
                NoSck,
                gpio::PA9<Alternate<5>>,
                gpio::PA12<Alternate<5>>,
                gpio::PB10<Alternate<5>>,
                gpio::PB13<Alternate<5>>,
                gpio::PD3<Alternate<5>>,
                #[cfg(feature = "h56x_h573")]
                gpio::PI1<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                gpio::PB14<Alternate<5>>,
                gpio::PC2<Alternate<5>>,
                #[cfg(feature = "h56x_h573")]
                gpio::PI2<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                gpio::PB15<Alternate<5>>,
                gpio::PC1<Alternate<5>>,
                gpio::PC3<Alternate<5>>,
                gpio::PG1<Alternate<7>>,
                #[cfg(feature = "h56x_h573")]
                gpio::PI3<Alternate<5>>
            ]
            HCS: [
                gpio::PA3<Alternate<5>>,
                gpio::PA11<Alternate<5>>,
                #[cfg(feature = "h523_h533")]
                gpio::PB1<Alternate<5>>,
                gpio::PB4<Alternate<7>>,
                gpio::PB9<Alternate<5>>,
                gpio::PB12<Alternate<5>>,
                #[cfg(feature = "h56x_h573")]
                gpio::PI0<Alternate<5>>
            ]
        SPI3:
            SCK: [
                NoSck,
                #[cfg(feature = "h523_h533")]
                gpio::PB1<Alternate<4>>,
                gpio::PB3<Alternate<6>>,
                #[cfg(feature = "h523_h533")]
                gpio::PB9<Alternate<6>>,
                gpio::PC10<Alternate<6>>
            ]
            MISO: [
                NoMiso,
                #[cfg(feature = "h523_h533")]
                gpio::PB0<Alternate<5>>,
                gpio::PB4<Alternate<6>>,
                gpio::PC11<Alternate<6>>,
                #[cfg(feature = "h523_h533")]
                gpio::PD7<Alternate<6>>
            ]
            MOSI: [
                NoMosi,
                #[cfg(feature = "h523_h533")]
                gpio::PA3<Alternate<6>>,
                #[cfg(feature = "h523_h533")]
                gpio::PA4<Alternate<4>>,
                gpio::PB2<Alternate<7>>,
                gpio::PB5<Alternate<7>>,
                gpio::PC12<Alternate<6>>,
                gpio::PD6<Alternate<5>>,
                #[cfg(feature = "h523_h533")]
                gpio::PG8<Alternate<5>>
            ]
            HCS: [
                gpio::PA4<Alternate<6>>,
                gpio::PA15<Alternate<6>>,
                #[cfg(feature = "h523_h533")]
                gpio::PB8<Alternate<6>>
            ]
        SPI4:
            SCK: [
                NoSck,
                #[cfg(feature = "h523_h533")]
                gpio::PA0<Alternate<5>>,
                #[cfg(feature = "h523_h533")]
                gpio::PC5<Alternate<6>>,
                gpio::PE2<Alternate<5>>,
                gpio::PE12<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                #[cfg(feature = "h523_h533")]
                gpio::PC0<Alternate<6>>,
                #[cfg(feature = "h523_h533")]
                gpio::PB7<Alternate<5>>,
                gpio::PE5<Alternate<5>>,
                gpio::PE13<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                #[cfg(feature = "h523_h533")]
                gpio::PA8<Alternate<6>>,
                #[cfg(feature = "stm32h523")]
                gpio::PC1<Alternate<6>>,
                gpio::PE6<Alternate<5>>,
                gpio::PE14<Alternate<5>>
            ]
            HCS: [
                gpio::PE4<Alternate<5>>,
                gpio::PE11<Alternate<5>>
            ]

    }

    instance! { SPI4: Spi4[u8, u16], SPI456, pclk2 }
}

#[cfg(feature = "h56x_h573")]
mod h56x_h573 {
    use super::*;
    use crate::stm32::{SPI5, SPI6};
    pins! {
        SPI5:
            SCK: [
                NoSck,
                gpio::PF7<Alternate<5>>,
                gpio::PH6<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                gpio::PF8<Alternate<5>>,
                gpio::PH7<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                gpio::PF9<Alternate<5>>,
                gpio::PF11<Alternate<5>>,
                gpio::PH8<Alternate<5>>
            ]
            HCS: [
                gpio::PF6<Alternate<5>>,
                gpio::PH5<Alternate<5>>,
                gpio::PH9<Alternate<5>>
            ]
        SPI6:
            SCK: [
                NoSck,
                gpio::PA5<Alternate<8>>,
                gpio::PB3<Alternate<8>>,
                gpio::PC12<Alternate<5>>,
                gpio::PG13<Alternate<5>>
            ]
            MISO: [
                NoMiso,
                gpio::PA6<Alternate<8>>,
                gpio::PB4<Alternate<8>>,
                gpio::PG12<Alternate<5>>
            ]
            MOSI: [
                NoMosi,
                gpio::PA7<Alternate<8>>,
                gpio::PB5<Alternate<8>>,
                gpio::PG14<Alternate<5>>
            ]
            HCS: [
                gpio::PA0<Alternate<5>>,
                gpio::PA4<Alternate<8>>,
                gpio::PA15<Alternate<7>>,
                gpio::PG8<Alternate<5>>
            ]
    }
    instance! { SPI5: Spi5[u8, u16], SPI456, pclk3 }
    instance! { SPI6: Spi6[u8, u16], SPI456, pclk2 }
}
