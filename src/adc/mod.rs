//! Analog to Digital Converter (ADC)
//!
//! ADC1 and ADC2 share a reset line. To initialise both of them, use the
//! [`adc12`] method.
//!
//! # Examples
//!
//! - [Reading a voltage using ADC1](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/adc.rs)
//! - [Reading a temperature using ADC2](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/temperature.rs)
//!
//! Originally from https://github.com/stm32-rs/stm32h7xx-hal
mod h5;

#[cfg(feature = "eh-02")]
use core::convert::Infallible;
use core::marker::PhantomData;
use core::ops::Deref;

use embedded_hal::delay::DelayNs;

use crate::rcc::rec::AdcDacClkSelGetter;

#[cfg(feature = "rm0492")]
use crate::stm32::{ADC1, ADC1 as ADCC};

#[cfg(feature = "rm0481")]
use crate::stm32::{ADC1, ADC2, ADCC};

use crate::pwr::{self, VoltageScale};
//use crate::rcc::rec::AdcClkSelGetter;
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

/// Marker trait for all ADC peripherals
pub trait Instance:
    crate::Sealed + Deref<Target = crate::stm32::adc1::RegisterBlock>
{
}

impl crate::Sealed for ADC1 {}

#[cfg(feature = "rm0481")]
impl crate::Sealed for ADC2 {}

impl Instance for ADC1 {}

#[cfg(feature = "rm0481")]
impl Instance for ADC2 {}

pub trait AdcCommonExt {
    fn claim(
        self,
        f_adc: Hertz,
        prec: rec::Adc,
        clocks: &CoreClocks,
        pwrcfg: &pwr::PowerConfiguration,
    ) -> AdcCommon;
}

impl AdcCommonExt for ADCC {
    fn claim(
        self,
        f_adc: Hertz,
        prec: rec::Adc,
        clocks: &CoreClocks,
        pwrcfg: &pwr::PowerConfiguration,
    ) -> AdcCommon {
        // Check adc_ker_ck_input
        kernel_clk_unwrap(&prec, clocks);

        // Enable AHB clock
        let prec = prec.enable();

        // Reset peripheral
        let prec = prec.reset();

        let _f_adc =
            AdcCommon::configure_clock(&self, f_adc, prec, clocks, pwrcfg); // ADC12_COMMON
        #[cfg(feature = "defmt")]
        defmt::trace!("Set f_adc to: {}", _f_adc);
        #[cfg(feature = "log")]
        log::trace!("Set f_adc to: {}", _f_adc);

        AdcCommon {}
    }
}

/// Type for initialized `ADC12_COMMON` or `ADC345_COMMON`
///
/// See [`AdcCommon::claim`]
#[non_exhaustive]
pub struct AdcCommon {}

impl AdcCommon {
    /// Sets the clock configuration for this ADC. This is common
    /// between ADC1 and ADC2, so the prec block is used to ensure
    /// this method can only be called on one of the ADCs (or both,
    /// using the [adc12](#method.adc12) method).
    ///
    /// Only `CKMODE[1:0]` = 0 is supported
    fn configure_clock(
        adcc: &ADCC,
        f_adc: Hertz,
        prec: rec::Adc,
        clocks: &CoreClocks,
        pwrcfg: &pwr::PowerConfiguration,
    ) -> Hertz {
        let ker_ck_input = kernel_clk_unwrap(&prec, clocks);

        let max_adc_ker_ck_analog = 75_000_000;
        let (max_ker_ck, max_ker_ck_input) = match pwrcfg.vos {
            VoltageScale::Scale0 => (125_000_000, 250_000_000),
            VoltageScale::Scale1 => (100_000_000, 200_000_000),
            VoltageScale::Scale2 => (75_000_000, 150_000_000),
            VoltageScale::Scale3 => (50_000_000, 100_000_000),
        };
        assert!(ker_ck_input.raw() <= max_ker_ck_input,
                "Kernel clock violates maximum frequency defined in Reference Manual. \
                    Can result in erroneous ADC readings");

        let f_adc = Self::configure_clock_unchecked(adcc, f_adc, prec, clocks);

        // Maximum ADC clock speed. With BOOST = 0 there is a no
        // minimum frequency given in part datasheets
        assert!(f_adc.raw() <= max_ker_ck);
        assert!(f_adc.raw() <= max_adc_ker_ck_analog);

        f_adc
    }

    /// No clock checks
    fn configure_clock_unchecked(
        adcc: &ADCC,
        f_adc: Hertz,
        prec: rec::Adc,
        clocks: &CoreClocks,
    ) -> Hertz {
        let ker_ck = kernel_clk_unwrap(&prec, clocks);

        let f_target = f_adc.raw();

        let (divider, presc) = match ker_ck.raw().div_ceil(f_target) {
            1 => (1, 0b0000),
            2 => (2, 0b0001),
            3..=4 => (4, 0b0010),
            5..=6 => (6, 0b0011),
            7..=8 => (8, 0b0100),
            9..=10 => (10, 0b0101),
            11..=12 => (12, 0b0110),
            13..=16 => (16, 0b0111),
            17..=32 => (32, 0b1000),
            33..=64 => (64, 0b1001),
            65..=128 => (128, 0b1010),
            129..=256 => (256, 0b1011),
            _ => panic!("Selecting the ADC clock required a prescaler > 256, \
                            which is not possible in hardware. Either increase the ADC \
                            clock frequency or decrease the kernel clock frequency"),
        };
        adcc.ccr().modify(|_, w| unsafe { w.presc().bits(presc) });

        Hertz::from_raw(ker_ck.raw() / divider)
    }

    fn setup_adc<ADC: Instance>(
        adc: ADC,
        delay: &mut impl DelayNs,
        resolution: Resolution,
    ) -> Adc<ADC, Disabled> {
        // Consume ADC register block, produce ADC1/2 with default settings
        let adc = Adc::<ADC, PoweredDown>::default_from_rb(adc);

        // Power Up, Preconfigure and Calibrate
        let mut adc = adc.power_up_and_calibrate(delay);

        // From RM0481:
        // This option bit must be set to 1 when ADCx_INP0 or ADCx_INN1 channel is selected
        adc.rb.or().modify(|_, w| w.op0().set_bit());

        // Set resolution
        adc.rb
            .cfgr()
            .modify(|_, w| unsafe { w.res().bits(resolution as _) });

        adc.set_resolution(resolution);
        adc
    }

    /// Initialise ADC
    ///
    /// Sets all configurable parameters to one-shot defaults,
    /// performs a boot-time calibration.
    #[cfg(feature = "rm0481")]
    pub fn claim_and_configure<ADC: Instance>(
        &self,
        adc: ADC,
        delay: &mut impl DelayNs,
        resolution: Resolution,
    ) -> Adc<ADC, Disabled> {
        Self::setup_adc(adc, delay, resolution)
    }

    /// Initialise ADC
    ///
    /// Sets all configurable parameters to one-shot defaults,
    /// performs a boot-time calibration.
    #[cfg(feature = "rm0492")]
    pub fn claim_and_configure(
        self,
        delay: &mut impl DelayNs,
        resolution: Resolution,
    ) -> Adc<ADC1, Disabled> {
        let adcc = unsafe { ADC1::steal() };
        Self::setup_adc::<ADC1>(adcc, delay, resolution)
    }
}

#[cfg(feature = "defmt")]
use defmt::{assert, panic};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum Resolution {
    TwelveBit = 0b00,
    TenBit = 0b01,
    EightBit = 0b10,
    SixBit = 0b11,
}

trait NumberOfBits {
    fn number_of_bits(&self) -> u32;
}

impl NumberOfBits for Resolution {
    fn number_of_bits(&self) -> u32 {
        match *self {
            Resolution::SixBit => 6,
            Resolution::EightBit => 8,
            Resolution::TenBit => 10,
            Resolution::TwelveBit => 12,
        }
    }
}

/// Enabled ADC (type state)
pub struct Enabled;
/// Disabled ADC (type state)
///
/// Disabled but powered on
pub struct Disabled;

/// Powered down ADC (type state)
pub struct PoweredDown;

pub trait ED {}
impl ED for Enabled {}
impl ED for Disabled {}

pub struct Adc<ADC, ED> {
    rb: ADC,
    sample_time: AdcSampleTime,
    resolution: Resolution,
    current_channel: Option<u8>,
    _enabled: PhantomData<ED>,
}

/// ADC DMA modes
///
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcDmaMode {
    OneShot,
    Circular,
}

/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
//
// Refer to RM0433 Rev 7 - Chapter 25.4.13
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(non_camel_case_types)]
pub enum AdcSampleTime {
    /// 2.5 cycles sampling time
    T_2_5,
    /// 6.5 cycles sampling time
    T_6_5,
    /// 12.5 cycles sampling time
    T_12_5,
    /// 24.5 cycles sampling time
    T_24_5,
    /// 47.5 cycles sampling time
    T_47_5,
    /// 92.5 cycles sampling time
    T_92_5,
    /// 247.5 cycles sampling time
    T_247_5,
    /// 640.5 cycles sampling time
    #[default]
    T_640_5,
}

/// The place in the sequence a given channel should be captured
#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
pub enum Sequence {
    /// 1
    One,
    /// 2
    Two,
    /// 3
    Three,
    /// 4
    Four,
    /// 5
    Five,
    /// 6
    Six,
    /// 7
    Seven,
    /// 8
    Eight,
    /// 9
    Nine,
    /// 10
    Ten,
    /// 11
    Eleven,
    /// 12
    Twelve,
    /// 13
    Thirteen,
    /// 14
    Fourteen,
    /// 15
    Fifteen,
    /// 16
    Sixteen,
}

impl From<Sequence> for u8 {
    fn from(s: Sequence) -> u8 {
        match s {
            Sequence::One => 0,
            Sequence::Two => 1,
            Sequence::Three => 2,
            Sequence::Four => 3,
            Sequence::Five => 4,
            Sequence::Six => 5,
            Sequence::Seven => 6,
            Sequence::Eight => 7,
            Sequence::Nine => 8,
            Sequence::Ten => 9,
            Sequence::Eleven => 10,
            Sequence::Twelve => 11,
            Sequence::Thirteen => 12,
            Sequence::Fourteen => 13,
            Sequence::Fifteen => 14,
            Sequence::Sixteen => 15,
        }
    }
}

impl From<u8> for Sequence {
    fn from(bits: u8) -> Self {
        match bits {
            0 => Sequence::One,
            1 => Sequence::Two,
            2 => Sequence::Three,
            3 => Sequence::Four,
            4 => Sequence::Five,
            5 => Sequence::Six,
            6 => Sequence::Seven,
            7 => Sequence::Eight,
            8 => Sequence::Nine,
            9 => Sequence::Ten,
            10 => Sequence::Eleven,
            11 => Sequence::Twelve,
            12 => Sequence::Thirteen,
            13 => Sequence::Fourteen,
            14 => Sequence::Fifteen,
            15 => Sequence::Sixteen,
            _ => unimplemented!(),
        }
    }
}

// Refer to RM0433 Rev 7 - Chapter 25.4.13
impl From<AdcSampleTime> for u8 {
    fn from(val: AdcSampleTime) -> u8 {
        match val {
            AdcSampleTime::T_2_5 => 0b000,
            AdcSampleTime::T_6_5 => 0b001,
            AdcSampleTime::T_12_5 => 0b010,
            AdcSampleTime::T_24_5 => 0b011,
            AdcSampleTime::T_47_5 => 0b100,
            AdcSampleTime::T_92_5 => 0b101,
            AdcSampleTime::T_247_5 => 0b110,
            AdcSampleTime::T_640_5 => 0b111,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalOffset(u8);

impl AdcCalOffset {
    pub fn value(self) -> u8 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalLinear([u32; 6]);

impl AdcCalLinear {
    pub fn value(self) -> [u32; 6] {
        self.0
    }
}

/// Vref internal signal
#[derive(Default)]
pub struct Vrefint;
/// Vbat/4 (Vbat pin input voltage divided by 4) internal signal
#[derive(Default)]
pub struct Vbat;
/// Internal temperature sensor
#[derive(Default)]
pub struct Temperature;
/// Internal digital core voltage
#[derive(Default)]
pub struct Vddcore;

/// Stored ADC config can be restored using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredConfig(AdcSampleTime, Resolution);

#[cfg(feature = "defmt")]
impl defmt::Format for StoredConfig {
    fn format(&self, fmt: defmt::Formatter) {
        let StoredConfig(sample_time, res) = &self;
        defmt::write!(fmt, "StoredConfig({:?}, {:?})", sample_time, res)
    }
}

pub trait AdcChannel<ADC: Instance> {
    const CH: u8;
}

/// Returns the frequency of the current adc_ker_ck
///
/// # Panics
///
/// Panics if the kernel clock is not running
fn kernel_clk_unwrap(
    prec: &impl AdcDacClkSelGetter,
    clocks: &CoreClocks,
) -> Hertz {
    match prec.get_kernel_clk_mux() {
        Some(rec::AdcDacClkSel::Hclk) => clocks.hclk(),
        Some(rec::AdcDacClkSel::Sys) => clocks.sys_ck(),
        Some(rec::AdcDacClkSel::Pll2R) => clocks
            .pll2()
            .r_ck()
            .expect("ADC: PLL2R clock must be enabled"),
        Some(rec::AdcDacClkSel::Hse) => {
            clocks.hse_ck().expect("ADC: HSE clock must be enabled")
        }
        Some(rec::AdcDacClkSel::HsiKer) => {
            clocks.hsi_ck().expect("ADC: HSI clock must be enabled")
        }
        Some(rec::AdcDacClkSel::CsiKer) => {
            clocks.csi_ck().expect("ADC: CSI clock must be enabled")
        }
        None => unreachable!(),
    }
}

impl<ADC: Instance> Adc<ADC, PoweredDown> {
    /// Creates ADC with default settings
    fn default_from_rb(rb: ADC) -> Self {
        Self {
            rb,
            sample_time: AdcSampleTime::default(),
            resolution: Resolution::TwelveBit,
            current_channel: None,
            _enabled: PhantomData,
        }
    }

    /// Disables Deeppowerdown-mode and enables voltage regulator
    ///
    /// Note: After power-up, a [`calibration`](#method.calibrate) shall be run
    fn power_up(&mut self, delay: &mut impl DelayNs) {
        // Refer to RM0433 Rev 7 - Chapter 25.4.6
        self.rb
            .cr()
            .modify(|_, w| w.deeppwd().clear_bit().advregen().set_bit());
        delay.delay_us(10);
    }

    pub fn power_up_and_calibrate(
        mut self,
        delay: &mut impl DelayNs,
    ) -> Adc<ADC, Disabled> {
        self.power_up(delay);

        let mut adc = Adc {
            rb: self.rb,
            sample_time: self.sample_time,
            resolution: self.resolution,
            current_channel: self.current_channel,
            _enabled: PhantomData,
        };
        adc.calibrate();
        adc
    }
}

impl<ADC: Instance> Adc<ADC, Disabled> {
    /// Enables Deeppowerdown-mode and disables voltage regulator
    ///
    /// Note: This resets the [`calibration`](#method.calibrate) of the ADC
    pub fn power_down(self) -> Adc<ADC, PoweredDown> {
        // Refer to RM0433 Rev 7 - Chapter 25.4.6
        self.rb
            .cr()
            .modify(|_, w| w.deeppwd().set_bit().advregen().clear_bit());

        Adc {
            rb: self.rb,
            sample_time: self.sample_time,
            resolution: self.resolution,
            current_channel: self.current_channel,
            _enabled: PhantomData,
        }
    }

    /// Calibrates the ADC in single channel mode
    ///
    /// Note: The ADC must be disabled
    fn calibrate(&mut self) {
        // Refer to RM0433 Rev 7 - Chapter 25.4.8
        self.check_calibration_conditions();

        let cal = |is_diff_mode| {
            // single channel (INNx equals to V_ref-)
            self.rb.cr().modify(|_, w| w.adcaldif().bit(is_diff_mode));
            // calibrate
            self.rb.cr().modify(|_, w| w.adcal().set_bit());
            while self.rb.cr().read().adcal().bit_is_set() {}
        };

        // Calibrate for single ended
        cal(false);

        // Calibrate for diff mode
        cal(true);
    }

    fn check_calibration_conditions(&self) {
        let cr = self.rb.cr().read();
        if cr.aden().bit_is_set() {
            panic!("Cannot start calibration when the ADC is enabled");
        }
        if cr.deeppwd().bit_is_set() {
            panic!("Cannot start calibration when the ADC is in deeppowerdown-mode");
        }
        if cr.advregen().bit_is_clear() {
            panic!("Cannot start calibration when the ADC voltage regulator is disabled");
        }
    }

    /// Configuration process immediately after enabling the ADC
    fn configure(&mut self) {
        // Single conversion mode, Software trigger
        // Refer to RM0433 Rev 7 - Chapters 25.4.15, 25.4.19
        self.rb.cfgr().modify(|_, w| {
            w.cont().clear_bit().exten().disabled().discen().set_bit()
        });

        // TODO: Enable boost mode in PWR_PMCR for highest possible clock frequency when low vdd
    }

    /// Enable ADC
    pub fn enable(mut self) -> Adc<ADC, Enabled> {
        // Refer to RM0433 Rev 7 - Chapter 25.4.9
        self.rb.isr().modify(|_, w| w.adrdy().clear());
        self.rb.cr().modify(|_, w| w.aden().set_bit());
        while self.rb.isr().read().adrdy().bit_is_clear() {}
        self.rb.isr().modify(|_, w| w.adrdy().clear());

        self.configure();

        Adc {
            rb: self.rb,
            sample_time: self.sample_time,
            resolution: self.resolution,
            current_channel: None,
            _enabled: PhantomData,
        }
    }
}

impl<ADC: Instance> Adc<ADC, Enabled> {
    fn stop_regular_conversion(&mut self) {
        self.rb.cr().modify(|_, w| w.adstp().set_bit());
        while self.rb.cr().read().adstp().bit_is_set() {}
    }

    fn stop_injected_conversion(&mut self) {
        self.rb.cr().modify(|_, w| w.jadstp().set_bit());
        while self.rb.cr().read().jadstp().bit_is_set() {}
    }

    pub fn configure_ch(
        &mut self,
        ch: u8,
        sequence: Sequence,
        sample_time: AdcSampleTime,
    ) {
        //Check the sequence is long enough
        self.rb.sqr1().modify(|r, w| unsafe {
            let prev: Sequence = r.l().bits().into();
            if prev < sequence {
                w.l().bits(sequence.into())
            } else {
                w
            }
        });
        let reg_i = u8::from(sequence) / 4;
        let i = u8::from(sequence) % 4;

        //Set the channel in the right sequence field
        match reg_i {
            0 => self.rb.sqr1().modify(|_, w| unsafe { w.sq(i).bits(ch) }),
            1 => self.rb.sqr2().modify(|_, w| unsafe { w.sq(i).bits(ch) }),
            2 => self.rb.sqr3().modify(|_, w| unsafe { w.sq(i).bits(ch) }),
            3 => self.rb.sqr4().modify(|_, w| unsafe { w.sq(i).bits(ch) }),
            _ => unreachable!(),
        };

        //Set the sample time for the channel
        let st = u8::from(sample_time);
        unsafe {
            match ch {
                0..=9 => self.rb.smpr1().modify(|_, w| w.smp(ch).bits(st)),
                10.. => self.rb.smpr2().modify(|_, w| w.smp(ch - 10).bits(st)),
            };
        }
    }

    /// Configure a channel for sampling.
    /// It will make sure the sequence is at least as long as the `sequence` provided.
    /// # Arguments
    /// * `channel` - channel to configure
    /// * `sequence` - where in the sequence to sample the channel. Also called rank in some STM docs/code
    /// * `sample_time` - how long to sample for. See datasheet and ref manual to work out how long you need\
    ///   to sample for at a given ADC clock frequency
    pub fn configure_channel<CHANNEL>(
        &mut self,
        _channel: &CHANNEL,
        sequence: Sequence,
        sample_time: AdcSampleTime,
    ) where
        CHANNEL: AdcChannel<ADC>,
    {
        self.configure_ch(CHANNEL::CH, sequence, sample_time);
    }

    /// Start conversion
    ///
    /// This method starts a conversion sequence
    #[inline(always)]
    fn start_conversion(&mut self) {
        //Start conversion
        self.rb.cr().modify(|_, w| w.adstart().set_bit());
    }

    /// Block until the conversion is completed and return to configured
    pub fn wait_for_conversion_sequence(&mut self) {
        while !self.rb.isr().read().eoc().bit_is_set() {}
    }

    pub fn convert<PIN: AdcChannel<ADC>>(
        &mut self,
        pin: &PIN,
        sample_time: AdcSampleTime,
    ) -> u16 {
        self.configure_channel(pin, Sequence::One, sample_time);
        self.start_conversion();
        //Wait for the sequence to complete
        self.wait_for_conversion_sequence();
        self.current_sample()
    }

    /// Read sample
    ///
    /// `nb::Error::WouldBlock` in case the conversion is still
    /// progressing.
    // Refer to RM0433 Rev 7 - Chapter 25.4.16
    #[cfg(feature = "eh-02")]
    pub fn read_sample(&mut self) -> nb::Result<u16, Infallible> {
        self.current_channel
            .expect("No channel was selected, use start_conversion first");

        // Wait for the conversion to finished
        if self.rb.isr().read().eoc().is_not_complete() {
            return Err(nb::Error::WouldBlock);
        }

        self.current_channel = None;

        // Retrieve result
        let result = self.current_sample();
        Ok(result)
    }

    /// Read the current value in the data register
    ///
    /// This simply returns whatever value is in the data register without blocking.
    /// Use [Self::read_sample] if you want to wait for any ongoing conversion to finish.
    ///
    /// NOTE: Depending on OVRMOD the data register acts like a FIFO queue with three stages
    pub fn current_sample(&mut self) -> u16 {
        self.rb.dr().read().rdata().bits()
    }

    /// Disable ADC
    pub fn disable(mut self) -> Adc<ADC, Disabled> {
        let cr = self.rb.cr().read();
        // Refer to RM0433 Rev 7 - Chapter 25.4.9
        if cr.adstart().bit_is_set() {
            self.stop_regular_conversion();
        }
        if cr.jadstart().bit_is_set() {
            self.stop_injected_conversion();
        }

        self.rb.cr().modify(|_, w| w.addis().set_bit());
        while self.rb.cr().read().aden().bit_is_set() {}

        Adc {
            rb: self.rb,
            sample_time: self.sample_time,
            resolution: self.resolution,
            current_channel: None,
            _enabled: PhantomData,
        }
    }
}

impl<ADC: Instance, ED> Adc<ADC, ED> {
    /// Save current ADC config
    pub fn save_cfg(&mut self) -> StoredConfig {
        StoredConfig(self.get_sample_time(), self.get_resolution())
    }

    /// Restore saved ADC config
    pub fn restore_cfg(&mut self, cfg: StoredConfig) {
        self.set_sample_time(cfg.0);
        self.set_resolution(cfg.1);
    }

    /// Reset the ADC config to default, return existing config
    pub fn default_cfg(&mut self) -> StoredConfig {
        let cfg = self.save_cfg();
        self.set_sample_time(AdcSampleTime::default());
        self.set_resolution(Resolution::TwelveBit);
        cfg
    }

    /// Get ADC samping time
    pub fn get_sample_time(&self) -> AdcSampleTime {
        self.sample_time
    }

    /// Get ADC sampling resolution
    pub fn get_resolution(&self) -> Resolution {
        self.resolution
    }

    /// Set ADC sampling time
    ///
    /// Options can be found in [AdcSampleTime].
    pub fn set_sample_time(&mut self, t_samp: AdcSampleTime) {
        self.sample_time = t_samp;
    }

    /// Set ADC sampling resolution
    pub fn set_resolution(&mut self, res: Resolution) {
        self.resolution = res;
    }

    /// Returns the largest possible sample value for the current ADC configuration
    ///
    /// Using this value as the denominator when calculating
    /// transfer functions results in a gain error, and thus should
    /// be avoided. Use the [slope](#method.slope) method instead.
    #[deprecated(since = "0.12.0", note = "See the slope() method instead")]
    pub fn max_sample(&self) -> u32 {
        (1 << self.get_resolution().number_of_bits()) - 1
    }

    /// Returns the slope for the current ADC configuration. 1 LSB = Vref / slope
    ///
    /// This value can be used in calcuations involving the transfer function of
    /// the ADC. For example, to calculate an estimate for the
    /// applied voltage of an ADC channel referenced to voltage
    /// `vref`
    ///
    /// ```
    /// let v = adc.read(&ch).unwrap() as f32 * vref / adc.slope() as f32;
    /// ```
    pub fn slope(&self) -> u32 {
        1 << self.get_resolution().number_of_bits()
    }

    /// Returns the offset calibration value for single ended channel
    pub fn read_offset_calibration_value(&self) -> AdcCalOffset {
        AdcCalOffset(self.rb.calfact().read().calfact_s().bits())
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &ADC {
        &self.rb
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut ADC {
        &mut self.rb
    }
}

#[cfg(feature = "eh-02")]
impl<ADC: Instance, PIN> embedded_hal_02::adc::OneShot<ADC, u16, PIN>
    for Adc<ADC, Enabled>
where
    PIN: embedded_hal_02::adc::Channel<ADC, ID = u8>,
{
    type Error = Infallible;

    // TODO: We are not really non-blocking
    fn read(&mut self, _pin: &mut PIN) -> nb::Result<u16, Infallible> {
        self.configure_ch(PIN::channel(), Sequence::One, self.sample_time);
        self.start_conversion();
        //Wait for the sequence to complete
        self.wait_for_conversion_sequence();
        Ok(self.current_sample())
    }
}
