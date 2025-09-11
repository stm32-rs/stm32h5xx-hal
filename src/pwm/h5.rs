use super::{
    BreakInput, Ch, ComplementaryDisabled, ComplementaryImpossible,
    ExternalTriggerPins, FaultPins, NPins, Pins, Pwm, C1, C2, C3, C4,
};
use crate::gpio::{self, Alternate};
use crate::pac;

// Pin definitions, mark which pins can be used with which timers and channels
macro_rules! pins {
    // Single channel timer
    ($($TIMX:ty:
        CH1($COMP1:ty): [$($( #[ $pmeta1:meta ] )* $CH1:ty),*]
        CH1N: [$($( #[ $pmeta2:meta ] )* $CH1N:ty),*]
        BRK:  [$($( #[ $pmeta3:meta ] )* $BRK:ty),*]
        BRK2: [$($( #[ $pmeta4:meta ] )* $BRK2:ty),*] // remove?
        ETR: [$($( #[ $pmeta5:meta ] )* $ETR:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl Pins<$TIMX, Ch<C1>, $COMP1> for $CH1 {
                    type Channel = Pwm<$TIMX, C1, $COMP1>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl NPins<$TIMX, Ch<C1>> for $CH1N {}
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl FaultPins<$TIMX,> for $BRK {
                    const INPUT: BreakInput = BreakInput::BreakIn;
                }
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl FaultPins<$TIMX> for $BRK2 {
                    const INPUT: BreakInput = BreakInput::BreakIn2;
                }
            )*
            $(
                $( #[ $pmeta5 ] )*
                impl ExternalTriggerPins<$TIMX> for $ETR {}
            )*
        )+
    };
    // Old single channel timer
    /*($($TIMX:ty: OUT: [$($OUT:ty),*])+) => {
        $(
            $(
                impl Pins<$TIMX, Ch<C1>, ComplementaryImpossible> for $OUT {
                    type Channel = Pwm<$TIMX, C1, ComplementaryImpossible>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
        )+
    };*/
    // Dual channel timer $pm
    ($($TIMX:ty:
        CH1($COMP1:ty): [$($( #[ $pmeta1:meta ] )* $CH1:ty),*]
        CH2($COMP2:ty): [$($( #[ $pmeta2:meta ] )* $CH2:ty),*]
        CH1N: [$($( #[ $pmeta3:meta ] )* $CH1N:ty),*]
        CH2N: [$($( #[ $pmeta4:meta ] )* $CH2N:ty),*]
        BRK:  [$($( #[ $pmeta5:meta ] )* $BRK:ty),*]
        BRK2: [$($( #[ $pmeta6:meta ] )* $BRK2:ty),*]
        ETR: [$($( #[ $pmeta7:meta ] )* $ETR:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl Pins<$TIMX, Ch<C1>, $COMP1> for $CH1 {
                    type Channel = Pwm<$TIMX, C1, $COMP1>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl Pins<$TIMX, Ch<C2>, $COMP2> for $CH2 {
                    type Channel = Pwm<$TIMX, C2, $COMP2>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl NPins<$TIMX, Ch<C1>> for $CH1N {}
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl NPins<$TIMX, Ch<C2>> for $CH2N {}
            )*
            $(
                $( #[ $pmeta5 ] )*
                impl FaultPins<$TIMX,> for $BRK {
                    const INPUT: BreakInput = BreakInput::BreakIn;
                }
            )*
            $(
                $( #[ $pmeta6 ] )*
                impl FaultPins<$TIMX> for $BRK2 {
                    const INPUT: BreakInput = BreakInput::BreakIn2;
                }
            )*
            $(
                $( #[ $pmeta7 ] )*
                impl ExternalTriggerPins<$TIMX> for $ETR {}
            )*
        )+
    };
    // Quad channel timers
    ($($TIMX:ty:
       CH1($COMP1:ty): [$($( #[ $pmeta1:meta ] )* $CH1:ty),*]
       CH2($COMP2:ty): [$($( #[ $pmeta2:meta ] )* $CH2:ty),*]
       CH3($COMP3:ty): [$($( #[ $pmeta3:meta ] )* $CH3:ty),*]
       CH4($COMP4:ty): [$($( #[ $pmeta4:meta ] )* $CH4:ty),*]
       CH1N: [$($( #[ $pmeta5:meta ] )* $CH1N:ty),*]
       CH2N: [$($( #[ $pmeta6:meta ] )* $CH2N:ty),*]
       CH3N: [$($( #[ $pmeta7:meta ] )* $CH3N:ty),*]
       CH4N: [$($( #[ $pmeta8:meta ] )* $CH4N:ty),*]
       BRK:  [$($( #[ $pmeta9:meta ] )* $BRK:ty),*]
       BRK2: [$($( #[ $pmeta10:meta ] )* $BRK2:ty),*]
       ETR: [$($( #[ $pmeta11:meta ] )* $ETR:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl Pins<$TIMX, Ch<C1>, $COMP1> for $CH1 {
                    type Channel = Pwm<$TIMX, C1, $COMP1>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl Pins<$TIMX, Ch<C2>, $COMP2> for $CH2 {
                    type Channel = Pwm<$TIMX, C2, $COMP2>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl Pins<$TIMX, Ch<C3>, $COMP3> for $CH3 {
                    type Channel = Pwm<$TIMX, C3, $COMP3>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl Pins<$TIMX, Ch<C4>, $COMP4> for $CH4 {
                    type Channel = Pwm<$TIMX, C4, $COMP4>;
                    fn split() -> Self::Channel {
                        Pwm::new()
                    }
                }
            )*
            $(
                $( #[ $pmeta5 ] )*
                impl NPins<$TIMX, Ch<C1>> for $CH1N {}
            )*
            $(
                $( #[ $pmeta6 ] )*
                impl NPins<$TIMX, Ch<C2>> for $CH2N {}
            )*
            $(
                $( #[ $pmeta7 ] )*
                impl NPins<$TIMX, Ch<C3>> for $CH3N {}
            )*
            $(
                $( #[ $pmeta8 ] )*
                impl NPins<$TIMX, Ch<C4>> for $CH4N {}
            )*
            $(
                $( #[ $pmeta9 ] )*
                impl FaultPins<$TIMX> for $BRK {
                    const INPUT: BreakInput = BreakInput::BreakIn;
                }
            )*
            $(
                $( #[ $pmeta10 ] )*
                impl FaultPins<$TIMX> for $BRK2 {
                    const INPUT: BreakInput = BreakInput::BreakIn2;
                }
            )*
            $(
                $( #[ $pmeta11 ] )*
                impl ExternalTriggerPins<$TIMX> for $ETR {}
            )*
        )+
    }
}

// Quad channel timers
#[cfg(feature = "rm0492")]
pins!(
    pac::TIM1:
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>,
            gpio::PA13<Alternate<1>>,
            gpio::PB1<Alternate<14>>,
            gpio::PB7<Alternate<14>>,
            gpio::PC6<Alternate<1>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>,
            gpio::PA14<Alternate<1>>,
            gpio::PC7<Alternate<1>>,
            gpio::PB4<Alternate<14>>,
            gpio::PB6<Alternate<14>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>,
            gpio::PB5<Alternate<1>>,
            gpio::PC8<Alternate<1>>,
            gpio::PA1<Alternate<14>>
        ]
        CH4(ComplementaryDisabled): [
            gpio::PA11<Alternate<1>>,
            gpio::PC9<Alternate<1>>,
            gpio::PA2<Alternate<14>>,
            gpio::PC12<Alternate<14>>
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>,
            gpio::PB13<Alternate<1>>,
            gpio::PA3<Alternate<14>>
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>,
            gpio::PB14<Alternate<1>>,
            gpio::PA4<Alternate<1>>,
            gpio::PB2<Alternate<1>>,
            gpio::PB7<Alternate<1>>
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>,
            gpio::PB15<Alternate<1>>,
            gpio::PB6<Alternate<1>>
        ]
        CH4N: [
            gpio::PC5<Alternate<1>>,
            gpio::PA8<Alternate<14>>,
            gpio::PA14<Alternate<14>>
        ]
        BRK: [
            gpio::PA6<Alternate<1>>,
            gpio::PB12<Alternate<1>>,
            gpio::PA4<Alternate<14>>,
            gpio::PB3<Alternate<14>>
        ]
        BRK2: [
            gpio::PB8<Alternate<1>>,
            gpio::PC10<Alternate<1>>,
            gpio::PC11<Alternate<14>>
        ]
        ETR: [
            gpio::PA12<Alternate<1>>,
            gpio::PC0<Alternate<1>>,
            gpio::PA13<Alternate<14>>,
            gpio::PB0<Alternate<14>>
        ]
    pac::TIM2:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>,
            gpio::PA5<Alternate<1>>,
            gpio::PA15<Alternate<1>>,
            gpio::PB2<Alternate<14>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>,
            gpio::PB3<Alternate<1>>,
            gpio::PC11<Alternate<1>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>,
            gpio::PB10<Alternate<1>>,
            gpio::PD2<Alternate<1>>,
            gpio::PA7<Alternate<14>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>,
            gpio::PC4<Alternate<1>>,
            gpio::PC12<Alternate<1>>,
            gpio::PA12<Alternate<14>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA0<Alternate<14>>,
            gpio::PA5<Alternate<14>>,
            gpio::PA15<Alternate<14>>,
            gpio::PD2<Alternate<14>>
        ]
    pac::TIM3:
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>,
            gpio::PB4<Alternate<2>>,
            gpio::PC6<Alternate<2>>,
            gpio::PA0<Alternate<2>>,
            gpio::PA14<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>,
            gpio::PB5<Alternate<2>>,
            gpio::PC7<Alternate<2>>,
            gpio::PA11<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>,
            gpio::PC8<Alternate<2>>,
            gpio::PA8<Alternate<2>>,
            gpio::PB6<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>,
            gpio::PC9<Alternate<2>>,
            gpio::PA12<Alternate<2>>,
            gpio::PB15<Alternate<14>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PD2<Alternate<2>>,
            gpio::PA2<Alternate<2>>,
            gpio::PB7<Alternate<2>>,
            gpio::PC1<Alternate<2>>
        ]
);

// Dual channel timers
#[cfg(feature = "h523_h533")]
pins!(
    // TODO: TIM12 seems to be missing for 523's pac, re add once fixed
    /*pac::TIM12:
        // According to Table 7. Timer features in the DS14540 Rev 2 and DS14539 Rev 2 datasheets TIM12 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PB14<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB15<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: []*/
    pac::TIM15:
        CH1(ComplementaryDisabled): [
            gpio::PA2<Alternate<4>>,
            gpio::PC12<Alternate<2>>,
            gpio::PE5<Alternate<4>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA3<Alternate<4>>,
            gpio::PE6<Alternate<4>>
        ]
        CH1N: [
            gpio::PA1<Alternate<4>>,
            gpio::PE4<Alternate<4>>
        ]
        CH2N: []
        BRK: [
            gpio::PA0<Alternate<4>>,
            gpio::PD2<Alternate<4>>,
            gpio::PE3<Alternate<4>>
        ]
        BRK2: []
        ETR: []
);

// Quad channel timers
#[cfg(feature = "h523_h533")]
pins!(
    pac::TIM1:
        // According to Table 7. Timer features in the DS14540 Rev 2 and DS14539 Rev 2 datasheets TIM8 has 3 complementary outputs
        // but according to Table 16. Alternate functions there are 4 such outputs
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>,
            gpio::PE9<Alternate<1>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>,
            gpio::PE11<Alternate<1>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>,
            gpio::PE13<Alternate<1>>
        ]
        CH4(ComplementaryDisabled): [
            gpio::PA11<Alternate<1>>,
            gpio::PE14<Alternate<1>>
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>,
            gpio::PB13<Alternate<1>>,
            gpio::PE8<Alternate<1>>
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>,
            gpio::PB14<Alternate<1>>,
            gpio::PE10<Alternate<1>>
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>,
            gpio::PB15<Alternate<1>>,
            gpio::PE12<Alternate<1>>
        ]
        CH4N: [
            gpio::PC5<Alternate<1>>,
            gpio::PD5<Alternate<1>>,
            gpio::PE15<Alternate<3>>
        ]
        BRK: [
            gpio::PA6<Alternate<1>>,
            gpio::PB12<Alternate<1>>,
            gpio::PE15<Alternate<1>>
        ]
        BRK2: [
            gpio::PE6<Alternate<1>>,
            gpio::PG4<Alternate<1>>
        ]
        ETR: [
            gpio::PA12<Alternate<1>>,
            gpio::PE7<Alternate<1>>,
            gpio::PG5<Alternate<1>>
        ]
    pac::TIM2:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>,
            gpio::PA5<Alternate<1>>,
            gpio::PA15<Alternate<1>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>,
            gpio::PB3<Alternate<1>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>,
            gpio::PB10<Alternate<1>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>,
            gpio::PC4<Alternate<1>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA0<Alternate<14>>,
            gpio::PA5<Alternate<14>>,
            gpio::PA15<Alternate<14>>
        ]
    pac::TIM3:
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>,
            gpio::PB4<Alternate<2>>,
            gpio::PC6<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>,
            gpio::PB5<Alternate<2>>,
            gpio::PC7<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>,
            gpio::PC8<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>,
            gpio::PC9<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PD2<Alternate<2>>
        ]
    pac::TIM4:
        CH1(ComplementaryImpossible): [
            gpio::PB6<Alternate<2>>,
            gpio::PD12<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB7<Alternate<2>>,
            gpio::PD13<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB8<Alternate<2>>,
            gpio::PD14<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB9<Alternate<2>>,
            gpio::PC2<Alternate<2>>,
            gpio::PD15<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PE0<Alternate<2>>
        ]
    pac::TIM5:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA4<Alternate<2>>
        ]
    pac::TIM8:
        // According to Table 7. Timer features in the DS14540 Rev 2 and DS14539 Rev 2 datasheets TIM8 has 3 complementary outputs
        // but according to Table 16. Alternate functions there are 4 such outputs
        CH1(ComplementaryDisabled): [
            gpio::PB10<Alternate<2>>,
            gpio::PC6<Alternate<3>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PB13<Alternate<2>>,
            gpio::PC7<Alternate<3>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PB12<Alternate<2>>,
            gpio::PC8<Alternate<3>>
        ]
        CH4(ComplementaryDisabled): [
            gpio::PC9<Alternate<3>>
        ]
        CH1N: [
            gpio::PA5<Alternate<3>>,
            gpio::PA7<Alternate<3>>
        ]
        CH2N: [
            gpio::PB0<Alternate<3>>,
            gpio::PB14<Alternate<3>>
        ]
        CH3N: [
            gpio::PB1<Alternate<3>>,
            gpio::PB15<Alternate<3>>
        ]
        CH4N: [
            gpio::PB2<Alternate<3>>,
            gpio::PD0<Alternate<3>>
        ]
        BRK: [
            gpio::PA6<Alternate<3>>,
            gpio::PG2<Alternate<3>>
        ]
        BRK2: [
            gpio::PA8<Alternate<3>>,
            gpio::PG3<Alternate<3>>
        ]
        ETR: [
            gpio::PA0<Alternate<3>>,
            gpio::PG8<Alternate<3>>
        ]
);

// Single channel timers
#[cfg(feature = "h56x_h573")]
pins!(
    pac::TIM13:
        // According to Table 7. Timer features in the DS14121 Rev 5 and DS14258 Rev 6 datasheets TIM13 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<9>>,
            gpio::PF8<Alternate<9>>
        ]
        CH1N: []
        BRK: []
        BRK2: []
        ETR: []
    pac::TIM14:
        // According to Table 7. Timer features in the DS14121 Rev 5 and DS14258 Rev 6 datasheets TIM14 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PA7<Alternate<9>>,
            gpio::PF9<Alternate<9>>
        ]
        CH1N: []
        BRK: []
        BRK2: []
        ETR: []
    pac::TIM16:
        CH1(ComplementaryDisabled): [
            gpio::PD8<Alternate<1>>,
            gpio::PF6<Alternate<1>>
        ]
        CH1N: [
            gpio::PB6<Alternate<1>>,
            gpio::PF8<Alternate<1>>
        ]
        BRK: [
            gpio::PB4<Alternate<1>>,
            gpio::PC0<Alternate<1>>,
            gpio::PF10<Alternate<1>>
        ]
        BRK2: []
        ETR: []
    pac::TIM17:
        CH1(ComplementaryDisabled): [
            gpio::PB9<Alternate<1>>,
            gpio::PC2<Alternate<1>>,
            gpio::PF7<Alternate<1>>
        ]
        CH1N: [
            gpio::PB7<Alternate<1>>,
            gpio::PF9<Alternate<1>>
        ]
        BRK: [
            gpio::PB5<Alternate<1>>,
            gpio::PG6<Alternate<1>>
        ]
        BRK2: []
        ETR: []
);

// Dual channel timers
#[cfg(feature = "h56x_h573")]
pins!(
    pac::TIM12:
        // According to Table 7. Timer features in the DS14121 Rev 5 and DS14258 Rev 6 datasheets TIM12 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PB14<Alternate<2>>,
            gpio::PH6<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB15<Alternate<2>>,
            gpio::PH9<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: []

    pac::TIM15:
        CH1(ComplementaryDisabled): [
            gpio::PA2<Alternate<4>>,
            gpio::PC12<Alternate<2>>,
            gpio::PE5<Alternate<4>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA3<Alternate<4>>,
            gpio::PE6<Alternate<4>>
        ]
        CH1N: [
            gpio::PA1<Alternate<4>>,
            gpio::PE4<Alternate<4>>
        ]
        CH2N: []
        BRK: [
            gpio::PA0<Alternate<4>>,
            gpio::PD2<Alternate<4>>,
            gpio::PE3<Alternate<4>>
        ]
        BRK2: []
        ETR: []

);

// Quad channel timers
#[cfg(feature = "h56x_h573")]
pins!(
        pac::TIM1:
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>,
            gpio::PE9<Alternate<1>>,
            gpio::PH11<Alternate<1>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>,
            gpio::PE11<Alternate<1>>,
            gpio::PH9<Alternate<1>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>,
            gpio::PE13<Alternate<1>>,
            gpio::PH7<Alternate<1>>
        ]
        CH4(ComplementaryDisabled): [
            gpio::PA11<Alternate<1>>,
            gpio::PE14<Alternate<1>>
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>,
            gpio::PB13<Alternate<1>>,
            gpio::PE8<Alternate<1>>,
            gpio::PH10<Alternate<1>>
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>,
            gpio::PB14<Alternate<1>>,
            gpio::PE10<Alternate<1>>,
            gpio::PH8<Alternate<1>>
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>,
            gpio::PB15<Alternate<1>>,
            gpio::PE12<Alternate<1>>,
            gpio::PH6<Alternate<1>>
        ]
        CH4N: [
            gpio::PC5<Alternate<1>>,
            gpio::PD5<Alternate<1>>,
            gpio::PE15<Alternate<3>>
        ]
        BRK: [
            gpio::PA6<Alternate<1>>,
            gpio::PB12<Alternate<1>>,
            gpio::PE15<Alternate<1>>,
            gpio::PH12<Alternate<1>>
        ]
        BRK2: [
            gpio::PE6<Alternate<1>>,
            gpio::PG4<Alternate<1>>
        ]
        ETR: [
            gpio::PA12<Alternate<1>>,
            gpio::PE7<Alternate<1>>,
            gpio::PG5<Alternate<1>>
        ]
    pac::TIM2:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>,
            gpio::PA5<Alternate<1>>,
            gpio::PA15<Alternate<1>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>,
            gpio::PB3<Alternate<1>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>,
            gpio::PB10<Alternate<1>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>,
            gpio::PC4<Alternate<1>>,
            gpio::PB11<Alternate<1>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA0<Alternate<14>>,
            gpio::PA5<Alternate<14>>,
            gpio::PA15<Alternate<14>>
        ]
    pac::TIM3:
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>,
            gpio::PB4<Alternate<2>>,
            gpio::PC6<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>,
            gpio::PB5<Alternate<2>>,
            gpio::PC7<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>,
            gpio::PC8<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>,
            gpio::PC9<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PD2<Alternate<2>>
        ]
    pac::TIM4:
        CH1(ComplementaryImpossible): [
            gpio::PB6<Alternate<2>>,
            gpio::PD12<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB7<Alternate<2>>,
            gpio::PD13<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB8<Alternate<2>>,
            gpio::PD14<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB9<Alternate<2>>,
            gpio::PC2<Alternate<2>>,
            gpio::PD15<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PE0<Alternate<2>>
        ]
    pac::TIM5:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<2>>,
            gpio::PH10<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<2>>,
            gpio::PH11<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<2>>,
            gpio::PH12<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<2>>,
            gpio::PI0<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA4<Alternate<2>>,
            gpio::PH8<Alternate<2>>
        ]
    pac::TIM8:
        CH1(ComplementaryDisabled): [
            gpio::PC6<Alternate<3>>,
            gpio::PH6<Alternate<3>>,
            gpio::PI5<Alternate<3>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PC7<Alternate<3>>,
            gpio::PH8<Alternate<3>>,
            gpio::PI6<Alternate<3>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PC8<Alternate<3>>,
            gpio::PH10<Alternate<3>>,
            gpio::PI7<Alternate<3>>
        ]
        CH4(ComplementaryDisabled): [
            gpio::PC9<Alternate<3>>,
            gpio::PI2<Alternate<3>>
        ]
        CH1N: [
            gpio::PA5<Alternate<3>>,
            gpio::PA7<Alternate<3>>,
            gpio::PH7<Alternate<3>>,
            gpio::PH13<Alternate<3>>
        ]
        CH2N: [
            gpio::PB0<Alternate<3>>,
            gpio::PB14<Alternate<3>>,
            gpio::PH9<Alternate<3>>,
            gpio::PH14<Alternate<3>>
        ]
        CH3N: [
            gpio::PB1<Alternate<3>>,
            gpio::PB15<Alternate<3>>,
            gpio::PH11<Alternate<3>>,
            gpio::PH15<Alternate<3>>
        ]
        CH4N: [
            gpio::PB2<Alternate<3>>,
            gpio::PD0<Alternate<3>>,
            gpio::PH12<Alternate<10>>
        ]
        BRK: [
            gpio::PA6<Alternate<3>>,
            gpio::PG2<Alternate<3>>,
            gpio::PH12<Alternate<3>>,
            gpio::PI4<Alternate<3>>
        ]
        BRK2: [
            gpio::PA8<Alternate<3>>,
            gpio::PG3<Alternate<3>>,
            gpio::PI1<Alternate<3>>
        ]
        ETR: [
            gpio::PA0<Alternate<3>>,
            gpio::PG8<Alternate<3>>,
            gpio::PI3<Alternate<3>>
        ]
);
