use super::{Adc, Disabled, Temperature, Vbat, Vddcore, Vrefint};
use crate::gpio::{self, Analog};

#[cfg(feature = "rm0492")]
use crate::stm32::{ADC1, ADC1 as ADCC};

#[cfg(feature = "rm0481")]
use crate::stm32::{ADC1, ADC2, ADCC};

macro_rules! adc_pins {
    ($ADC:ident, $($input:ty => $chan:expr),+ $(,)*) => {
        $(
            impl embedded_hal_02::adc::Channel<$ADC> for $input {
                type ID = u8;

                fn channel() -> u8 {
                    $chan
                }
            }
        )+
    };
}

macro_rules! adc_internal {
    ([$INT_ADC:ident, $INT_ADC_COMMON:ident]; $($input:ty => ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            impl $input {
                pub fn new() -> Self {
                    Self {}
                }

                /// Enables the internal voltage/sensor
                /// ADC must be disabled.
                pub fn enable(&mut self, _adc: &mut Adc<$INT_ADC, Disabled>) {
                    // TODO: This is not safe since we do not hold both adcs
                    let common = unsafe { ADCC::steal() };

                    common.ccr().modify(|_, w| w.$en().bit(true));
                }
                /// Disables the internal voltage/sdissor
                /// ADC must be disabled.
                pub fn disable(&mut self, _adc: &mut Adc<$INT_ADC, Disabled>) {
                    // TODO: This is not safe since we do not hold both adcs
                    let common = unsafe { ADCC::steal() };

                    common.ccr().modify(|_, w| w.$en().bit(false));
                }
            }

            adc_pins!($INT_ADC, $input => $chan);
        )+
    };
}

#[cfg(feature = "rm0492")]
impl Vddcore {
    pub fn enable(_adc: &Adc<ADC1, Disabled>) {
        let adc = unsafe { ADC1::steal() };

        adc.or().modify(|_, w| w.op1().set_bit());
    }

    pub fn disable(_adc: &Adc<ADC1, Disabled>) {
        let adc = unsafe { ADC1::steal() };

        adc.or().modify(|_, w| w.op1().clear_bit());
    }
}
#[cfg(feature = "rm0492")]
adc_pins!(ADC1, Vddcore => 16);

#[cfg(feature = "rm0481")]
impl Vddcore {
    pub fn enable(_adc: &Adc<ADC2, Disabled>) {
        let adc2 = unsafe { ADC1::steal() };

        adc2.or().modify(|_, w| w.op0().bit(true));
    }

    pub fn disable(_adc: &Adc<ADC2, Disabled>) {
        let adc2 = unsafe { ADC1::steal() };

        adc2.or().modify(|_, w| w.op0().bit(false));
    }
}
#[cfg(feature = "rm0481")]
adc_pins!(ADC2, Vddcore => 17);

#[cfg(feature = "rm0492")]
adc_internal!(
    [ADC1, ADCC];

    Temperature => (16, tsen),
    Vrefint => (17, vrefen),
    Vbat => (2, vbaten),
);

#[cfg(feature = "rm0481")]
adc_internal!(
    [ADC1, ADCC];

    Temperature => (16, tsen),
    Vrefint => (17, vrefen),
);

#[cfg(feature = "rm0481")]
adc_internal!(
    [ADC2, ADCC];

    Vbat => (16, vbaten),
);

macro_rules! adc_pins_common {
    ($($input:ty => $chan:expr),+ $(,)*) => {$(
        adc_pins!(ADC1, $input => $chan);

        #[cfg(feature = "rm0481")]
        adc_pins!(ADC2, $input => $chan);
    )*};
}

#[cfg(any(
    feature = "stm32h503",
    feature = "stm32h523",
    feature = "stm32h533"
))]
adc_pins_common!(
    gpio::PC0<Analog> => 10,
    gpio::PC1<Analog> => 11,
    gpio::PC2<Analog> => 12,
    gpio::PC3<Analog> => 13,

    gpio::PA0<Analog> => 0,
    gpio::PA1<Analog> => 1,
    gpio::PA2<Analog> => 14,
    gpio::PA3<Analog> => 15,
    gpio::PA4<Analog> => 18,
    gpio::PA5<Analog> => 19,
    gpio::PA6<Analog> => 3,
    gpio::PA7<Analog> => 7,

    gpio::PC4<Analog> => 4,
    gpio::PC5<Analog> => 8,
    gpio::PB0<Analog> => 9,
    gpio::PB1<Analog> => 5,
);

#[cfg(feature = "stm32h523")]
adc_pins!(
    ADC1,
    gpio::PF11<Analog> => 2,
    gpio::PF12<Analog> => 6,

);

#[cfg(feature = "stm32h523")]
adc_pins!(
    ADC2,
    gpio::PF13<Analog> => 2,
    gpio::PF14<Analog> => 6,
);
