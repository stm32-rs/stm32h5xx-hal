// Pin definitions, mark which pins can be used with which timers and channels
macro_rules! pins {
    // Single channel timer
    ($($TIMX:ty: OUT: [$($OUT:ty),*])+) => {
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
    };
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

#[cfg(feature = "rm0492")]
pins!(
    pac::TIM1: //503
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>, //503
            gpio::PA13<Alternate<1>>, //503
            gpio::PB1<Alternate<14>>, //503
            gpio::PB7<Alternate<14>>, //503
            gpio::PC6<Alternate<1>>, //503
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>, //503
            gpio::PA14<Alternate<1>>, //503
            gpio::PC7<Alternate<1>>, //503
            gpio::PB4<Alternate<14>>, //503
            gpio::PB6<Alternate<14>>, //503
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>, //503
            gpio::PB5<Alternate<1>>, //503
            gpio::PC8<Alternate<1>>, //503
            gpio::PA1<Alternate<14>>, //503
        ]
        CH4(ComplementaryDisabled): [
            gpio::PA11<Alternate<1>>, //503
            gpio::PC9<Alternate<1>>, //503
            gpio::PA2<Alternate<14>>, //503
            gpio::PC12<Alternate<14>>, //503
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>, //503
            gpio::PB13<Alternate<1>>, //503
            gpio::PA3<Alternate<14>>, //503
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>, //503
            gpio::PB14<Alternate<1>>, //503
            gpio::PA4<Alternate<1>>, //503
            gpio::PB2<Alternate<1>>, //503
            gpio::PB7<Alternate<1>>, //503
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>, //503
            gpio::PB15<Alternate<1>>, //503
            gpio::PB6<Alternate<1>>, //503
        ]
        CH4N: [
            gpio::PC5<Alternate<1>>, //503
            gpio::PA8<Alternate<14>>, //503
            gpio::PA14<Alternate<14>>, //503
        ]
        BRK: [
            gpio::PA6<Alternate<1>>, //503
            gpio::PB12<Alternate<1>>, //503
            gpio::PA4<Alternate<14>>, //503
            gpio::PB3<Alternate<14>>, //503
        ]
        BRK2: [
            gpio::PB8<Alternate<1>>, //503
            gpio::PC10<Alternate<1>>, //503
            gpio::PC11<Alternate<14>>, //503
        ]
        ETR: [
            gpio::PA12<Alternate<1>>, //503
            gpio::PC0<Alternate<1>>, //503
            gpio::PA13<Alternate<14>>, //503
            gpio::PB0<Alternate<14>>, //503
        ]
    pac::TIM2: //503
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>, //503
            gpio::PA5<Alternate<1>>, //503
            gpio::PA15<Alternate<1>>, //503
            gpio::PB2<Alternate<14>>, //503
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>, //503
            gpio::PB3<Alternate<1>>, //503
            gpio::PC11<Alternate<1>>, //503
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>, //503
            gpio::PB10<Alternate<1>>, //503
            gpio::PD2<Alternate<1>>, //503
            gpio::PA7<Alternate<14>>, //503
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>, //503
            gpio::PC4<Alternate<1>>, //503
            gpio::PC12<Alternate<1>>, //503
            gpio::PA12<Alternate<14>>, //503
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA0<Alternate<14>>, //503
            gpio::PA5<Alternate<14>>, //503
            gpio::PA15<Alternate<14>>, //503
            gpio::PD2<Alternate<14>>, //503
        ]
    pac::TIM3: //503
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>, //503
            gpio::PB4<Alternate<2>>, //503
            gpio::PC6<Alternate<2>>, //503
            gpio::PA0<Alternate<2>>, //503
            gpio::PA14<Alternate<2>>, //503
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>, //503
            gpio::PB5<Alternate<2>>, //503
            gpio::PC7<Alternate<2>>, //503
            gpio::PA11<Alternate<2>>, //503
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>, //503
            gpio::PC8<Alternate<2>>, //503
            gpio::PA8<Alternate<2>>, //503
            gpio::PB6<Alternate<2>>, //503
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>, //503
            gpio::PC9<Alternate<2>>, //503
            gpio::PA12<Alternate<2>>, //503
            gpio::PB15<Alternate<14>>, //503
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PD2<Alternate<2>>, //503
            gpio::PA2<Alternate<2>>, //503
            gpio::PB7<Alternate<2>>, //503
            gpio::PC1<Alternate<2>>, //503
        ]
);

#[cfg(feature = "h523_h533")]
pins!(
    pac::TIM1: //523,533
        // According to Table 7. Timer features in the DS14540 Rev 2 and DS14539 Rev 2 datasheets TIM8 has 3 complementary outputs
        // but according to Table 16. Alternate functions there are 4 such outputs
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>, //523,533
            gpio::PE9<Alternate<1>>, //523,533
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>, //523,533
            gpio::PE11<Alternate<1>>, //523,533
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>, //523,533
            gpio::PE13<Alternate<1>>, //523,533
        ]
        CH4(ComplementaryDisabled): [
            gpio::PA11<Alternate<1>>, //523,533
            gpio::PE14<Alternate<1>>, //523,533
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>, //523,533
            gpio::PB13<Alternate<1>>, //523,533
            gpio::PE8<Alternate<1>>, //523,533
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>, //523,533
            gpio::PB14<Alternate<1>>, //523,533
            gpio::PE10<Alternate<1>>, //523,533
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>, //523,533
            gpio::PB15<Alternate<1>>, //523,533
            gpio::PE12<Alternate<1>>, //523,533
        ]
        CH4N: [
            gpio::PC5<Alternate<1>>, //523,533
            gpio::PD5<Alternate<1>>, //523,533
            gpio::PE15<Alternate<3>>, //523,533
        ]
        BRK: [
            gpio::PA6<Alternate<1>>, //523,533
            gpio::PB12<Alternate<1>>, //523,533
            gpio::PE15<Alternate<1>>, //523,533
        ]
        BRK2: [
            gpio::PE6<Alternate<1>>, //523,533
            gpio::PG4<Alternate<1>>, //523,533
        ]
        ETR: [
            gpio::PA12<Alternate<1>>, //523,533
            gpio::PE7<Alternate<1>>, //523,533
            gpio::PG5<Alternate<1>>, //523,533
        ]
    pac::TIM2: //523,533
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>, //523,533
            gpio::PA5<Alternate<1>>, //523,533
            gpio::PA15<Alternate<1>>, //523,533
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>, //523,533
            gpio::PB3<Alternate<1>>, //523,533
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>, //523,533
            gpio::PB10<Alternate<1>>, //523,533
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>, //523,533
            gpio::PC4<Alternate<1>>, //523,533
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA0<Alternate<14>>, //523,533
            gpio::PA5<Alternate<14>>, //523,533
            gpio::PA15<Alternate<14>>, //523,533
        ]
    pac::TIM3: //523,533
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>, //523,533
            gpio::PB4<Alternate<2>>, //523,533
            gpio::PC6<Alternate<2>>, //523,533
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>, //523,533
            gpio::PB5<Alternate<2>>, //523,533
            gpio::PC7<Alternate<2>>, //523,533
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>, //523,533
            gpio::PC8<Alternate<2>>, //523,533
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>, //523,533
            gpio::PC9<Alternate<2>>, //523,533
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PD2<Alternate<2>>, //523,533
        ]
    pac::TIM4: //523,533
        CH1(ComplementaryImpossible): [
            gpio::PB6<Alternate<2>>, //523,533
            gpio::PD12<Alternate<2>>, //523,533
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB7<Alternate<2>>, //523,533
            gpio::PD13<Alternate<2>>, //523,533
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB8<Alternate<2>>, //523,533
            gpio::PD14<Alternate<2>>, //523,533
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB9<Alternate<2>>, //523,533
            gpio::PC2<Alternate<2>>, //523,533
            gpio::PD15<Alternate<2>>, //523,533
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PE0<Alternate<2>>, //523,533
        ]
    pac::TIM5: //523,533,563,573
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<2>>, //523,533
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<2>>, //523,533
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<2>>, //523,533
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<2>>, //523,533
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA4<Alternate<2>>, //523,533
        ]
    pac::TIM8: //523,533
        // According to Table 7. Timer features in the DS14540 Rev 2 and DS14539 Rev 2 datasheets TIM8 has 3 complementary outputs
        // but according to Table 16. Alternate functions there are 4 such outputs
        CH1(ComplementaryDisabled): [
            gpio::PB10<Alternate<2>>, //523,533
            gpio::PC6<Alternate<3>>, //523,533
        ]
        CH2(ComplementaryDisabled): [
            gpio::PB13<Alternate<2>>, //523,533
            gpio::PC7<Alternate<3>>, //523,533
        ]
        CH3(ComplementaryDisabled): [
            gpio::PB12<Alternate<2>>, //523,533
            gpio::PC8<Alternate<3>>, //523,533
        ]
        CH4(ComplementaryDisabled): [
            gpio::PC9<Alternate<3>>, //523,533
        ]
        CH1N: [
            gpio::PA5<Alternate<3>>, //523,533
            gpio::PA7<Alternate<3>>, //523,533
        ]
        CH2N: [
            gpio::PB0<Alternate<3>>, //523,533
            gpio::PB14<Alternate<3>>, //523,533
        ]
        CH3N: [
            gpio::PB1<Alternate<3>>, //523,533
            gpio::PB15<Alternate<3>>, //523,533
        ]
        CH4N: [
            gpio::PB2<Alternate<3>>, //523,533
            gpio::PD0<Alternate<3>>, //523,533
        ]
        BRK: [
            gpio::PA6<Alternate<3>>, //523,533
            gpio::PG2<Alternate<3>>, //523,533
        ]
        BRK2: [
            gpio::PA8<Alternate<3>>, //523,533
            gpio::PG3<Alternate<3>>, //523,533
        ]
        ETR: [
            gpio::PA0<Alternate<3>>, //523,533
            gpio::PG8<Alternate<3>>, //523,533
        ]
    pac::TIM12: //523,533
        // According to Table 7. Timer features in the DS14540 Rev 2 and DS14539 Rev 2 datasheets TIM12 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PB14<Alternate<2>>, //523,533
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB15<Alternate<2>>, //523,533
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: []
    pac::TIM15: //523,533
        CH1(ComplementaryDisabled): [
            gpio::PA2<Alternate<4>>, //523,533
            gpio::PC12<Alternate<2>>, //523,533
            gpio::PE5<Alternate<4>>, //523,533
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA3<Alternate<4>>, //523,533
            gpio::PE6<Alternate<4>>, //523,533
        ]
        CH1N: [
            gpio::PA1<Alternate<4>>, //523,533
            gpio::PE4<Alternate<4>>, //523,533
        ]
        CH2N: []
        BRK: [
            gpio::PA0<Alternate<4>>, //523,533
            gpio::PD2<Alternate<4>>, //523,533
            gpio::PE3<Alternate<4>>, //523,533
        ]
        BRK2: []
        ETR: []
);

#[cfg(feature = "h56x_h573")]
pins!(
    pac::TIM1: //563,573
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>, //563,573
            gpio::PE9<Alternate<1>>, //563,573
            gpio::PH11<Alternate<1>>, //563,573
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>, //563,573
            gpio::PE11<Alternate<1>>, //563,573
            gpio::PH9<Alternate<1>>, //563,573
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>, //563,573
            gpio::PE13<Alternate<1>>, //563,573
            gpio::PH7<Alternate<1>>, //563,573
        ]
        CH4(ComplementaryDisabled): [
            gpio::PA11<Alternate<1>>, //563,573
            gpio::PE14<Alternate<1>>, //563,573
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>, //563,573
            gpio::PB13<Alternate<1>>, //563,573
            gpio::PE8<Alternate<1>>, //563,573
            gpio::PH10<Alternate<1>>, //563,573
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>, //563,573
            gpio::PB14<Alternate<1>>, //563,573
            gpio::PE10<Alternate<1>>, //563,573
            gpio::PH8<Alternate<1>>, //563,573
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>, //563,573
            gpio::PB15<Alternate<1>>, //563,573
            gpio::PE12<Alternate<1>>, //563,573
            gpio::PH6<Alternate<1>>, //563,573
        ]
        CH4N: [
            gpio::PC5<Alternate<1>>, //563,573
            gpio::PD5<Alternate<1>>, //563,573
            gpio::PE15<Alternate<3>>, //563,573
        ]
        BRK: [
            gpio::PA6<Alternate<1>>, //563,573
            gpio::PB12<Alternate<1>>, //563,573
            gpio::PE15<Alternate<1>>, //563,573
            gpio::PH12<Alternate<1>>, //563,573
        ]
        BRK2: [
            gpio::PE6<Alternate<1>>, //563,573
            gpio::PG4<Alternate<1>>, //563,573
        ]
        ETR: [
            gpio::PA12<Alternate<1>>, //563,573
            gpio::PE7<Alternate<1>>, //563,573
            gpio::PG5<Alternate<1>>, //563,573
        ]
    pac::TIM2: //563,573
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>, //563,573
            gpio::PA5<Alternate<1>>, //563,573
            gpio::PA15<Alternate<1>>, //563,573
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>, //563,573
            gpio::PB3<Alternate<1>>, //563,573
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>, //563,573
            gpio::PB10<Alternate<1>>, //563,573
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>, //563,573
            gpio::PC4<Alternate<1>>, //563,573
            gpio::PB11<Alternate<1>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA0<Alternate<14>>, //563,573
            gpio::PA5<Alternate<14>>, //563,573
            gpio::PA15<Alternate<14>>, //563,573
        ]
    pac::TIM3: //563,573
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>, //563,573
            gpio::PB4<Alternate<2>>, //563,573
            gpio::PC6<Alternate<2>>, //563,573
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>, //563,573
            gpio::PB5<Alternate<2>>, //563,573
            gpio::PC7<Alternate<2>>, //563,573
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>, //563,573
            gpio::PC8<Alternate<2>>, //563,573
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>, //563,573
            gpio::PC9<Alternate<2>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PD2<Alternate<2>>, //563,573
        ]
    pac::TIM4: //563,573
        CH1(ComplementaryImpossible): [
            gpio::PB6<Alternate<2>>, //563,573
            gpio::PD12<Alternate<2>>, //563,573
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB7<Alternate<2>>, //563,573
            gpio::PD13<Alternate<2>>, //563,573
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB8<Alternate<2>>, //563,573
            gpio::PD14<Alternate<2>>, //563,573
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB9<Alternate<2>>, //563,573
            gpio::PC2<Alternate<2>>, //563,573
            gpio::PD15<Alternate<2>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PE0<Alternate<2>>, //563,573
        ]
    pac::TIM5: //563,573
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<2>>, //563,573
            gpio::PH10<Alternate<2>>, //563,573
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<2>>, //563,573
            gpio::PH11<Alternate<2>>, //563,573
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<2>>, //563,573
            gpio::PH12<Alternate<2>>, //563,573
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<2>>, //563,573
            gpio::PI0<Alternate<2>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: [
            gpio::PA4<Alternate<2>>, //563,573
            gpio::PH8<Alternate<2>>, //563,573
        ]
    pac::TIM8: //563,573
        CH1(ComplementaryDisabled): [
            gpio::PC6<Alternate<3>>, //563,573
            gpio::PH6<Alternate<3>>, //563,573
            gpio::PI5<Alternate<3>>, //563,573
        ]
        CH2(ComplementaryDisabled): [
            gpio::PC7<Alternate<3>>, //563,573
            gpio::PH8<Alternate<3>>, //563,573
            gpio::PI6<Alternate<3>>, //563,573
        ]
        CH3(ComplementaryDisabled): [
            gpio::PC8<Alternate<3>>, //563,573
            gpio::PH10<Alternate<3>>, //563,573
            gpio::PI7<Alternate<3>>, //563,573
        ]
        CH4(ComplementaryDisabled): [
            gpio::PC9<Alternate<3>>, //563,573
            gpio::PI2<Alternate<3>>, //563,573
        ]
        CH1N: [
            gpio::PA5<Alternate<3>>, //563,573
            gpio::PA7<Alternate<3>>, //563,573
            gpio::PH7<Alternate<3>>, //563,573
            gpio::PH13<Alternate<3>>, //563,573
        ]
        CH2N: [
            gpio::PB0<Alternate<3>>, //563,573
            gpio::PB14<Alternate<3>>, //563,573
            gpio::PH9<Alternate<3>>, //563,573
            gpio::PH14<Alternate<3>>, //563,573
        ]
        CH3N: [
            gpio::PB1<Alternate<3>>, //563,573
            gpio::PB15<Alternate<3>>, //563,573
            gpio::PH11<Alternate<3>>, //563,573
            gpio::PH15<Alternate<3>>, //563,573
        ]
        CH4N: [
            gpio::PB2<Alternate<3>>, //563,573
            gpio::PD0<Alternate<3>>, //563,573
            gpio::PH12<Alternate<10>>, //563,573
        ]
        BRK: [
            gpio::PA6<Alternate<3>>, //563,573
            gpio::PG2<Alternate<3>>, //563,573
            gpio::PH12<Alternate<3>>, //563,573
            gpio::PI4<Alternate<3>>, //563,573
        ]
        BRK2: [
            gpio::PA8<Alternate<3>>, //563,573
            gpio::PG3<Alternate<3>>, //563,573
            gpio::PI1<Alternate<3>>, //563,573
        ]
        ETR: [
            gpio::PA0<Alternate<3>>, //563,573
            gpio::PG8<Alternate<3>>, //563,573
            gpio::PI3<Alternate<3>>, //563,573
        ]
    pac::TIM12: //563,573
        // According to Table 7. Timer features in the DS14121 Rev 5 and DS14258 Rev 6 datasheets TIM12 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PB14<Alternate<2>>, //563,573
            gpio::PH6<Alternate<2>>, //563,573
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB15<Alternate<2>>, //563,573
            gpio::PH9<Alternate<2>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: []
    pac::TIM13: //563,573
        // According to Table 7. Timer features in the DS14121 Rev 5 and DS14258 Rev 6 datasheets TIM13 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<9>>, //563,573
            gpio::PF8<Alternate<9>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: []
    pac::TIM14: //563,573
        // According to Table 7. Timer features in the DS14121 Rev 5 and DS14258 Rev 6 datasheets TIM14 has 1 complementary output
        // but according to Table 16. Alternate functions there exists no such output
        CH1(ComplementaryImpossible): [
            gpio::PA7<Alternate<9>>, //563,573
            gpio::PF9<Alternate<9>>, //563,573
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
        ETR: []
    pac::TIM15: //563,573
        CH1(ComplementaryDisabled): [
            gpio::PA2<Alternate<4>>, //563,573
            gpio::PC12<Alternate<2>>, //563,573
            gpio::PE5<Alternate<4>>, //563,573
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA3<Alternate<4>>, //563,573
            gpio::PE6<Alternate<4>>, //563,573
        ]
        CH1N: [
            gpio::PA1<Alternate<4>>, //563,573
            gpio::PE4<Alternate<4>>, //563,573
        ]
        CH2N: []
        BRK: [
            gpio::PA0<Alternate<4>>, //563,573
            gpio::PD2<Alternate<4>>, //563,573
            gpio::PE3<Alternate<4>>, //563,573
        ]
        BRK2: []
        ETR: []
    pac::TIM16: //563,573
    CH1(ComplementaryDisabled): [
        gpio::PD8<Alternate<1>>, //563,573
        gpio::PF6<Alternate<1>>, //563,573
    ]
    CH1N: [
        gpio::PB6<Alternate<1>>, //563,573
        gpio::PF8<Alternate<1>>, //563,573
    ]
    CH2N: []
    BRK: [
        gpio::PB4<Alternate<1>>, //563,573
        gpio::PC0<Alternate<1>>, //563,573
        gpio::PF10<Alternate<1>>, //563,573
    ]
    BRK2: []
    ETR: []
pac::TIM17: //563,573
    CH1(ComplementaryDisabled): [
        gpio::PB9<Alternate<1>>, //563,573
        gpio::PC2<Alternate<1>>, //563,573
        gpio::PF7<Alternate<1>>, //563,573
    ]
    CH1N: [
        gpio::PB7<Alternate<1>>, //563,573
        gpio::PF9<Alternate<1>>, //563,573
    ]
    CH2N: []
    BRK: [
        gpio::PB5<Alternate<1>>, //563,573
        gpio::PG6<Alternate<1>>, //563,573
    ]
    BRK2: []
    ETR: []
);
