//! Example of using ADC1 and ADC2 together
//!
//! This is not available for H503 since it only has ADC1
//!
//! For an example of using ADC1 alone, see examples/adc.rs

#![no_main]
#![no_std]

use cortex_m_rt::entry;

use stm32h5xx_hal::{
    adc::{self, AdcCommonExt, AdcSampleTime},
    delay::Delay,
    pac,
    prelude::*,
    rcc::rec::AdcDacClkSel,
};
use utilities::logger::info;

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    // We need to configure a clock for adc_ker_ck_input. The default
    // adc_ker_ck_input is pll2_p_ck, but we will use per_ck. per_ck is sourced
    // from the 64MHz HSI
    //
    // adc_ker_ck_input is then divided by the ADC prescaler to give f_adc. The
    // maximum f_adc is 50MHz
    let mut ccdr = rcc
        .sys_ck(192.MHz())
        .pll1_q_ck(64.MHz())
        .freeze(&pwrcfg, &dp.SBS);

    // Switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adcdac_clk_mux(AdcDacClkSel::HsiKer);

    info!("");
    info!("stm32h5xx-hal example - ADC1 and ADC2");
    info!("");

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);

    // Setup adc common
    let adcc =
        dp.ADCC
            .claim(4.MHz(), ccdr.peripheral.ADC, &ccdr.clocks, &pwrcfg);

    // Set up individual adc's
    let adc1 = adcc.claim_and_configure(
        dp.ADC1,
        &mut delay,
        adc::Resolution::TwelveBit,
    );
    let adc2 = adcc.claim_and_configure(
        dp.ADC2,
        &mut delay,
        adc::Resolution::TwelveBit,
    );

    let mut adc1 = adc1.enable();
    let mut adc2 = adc2.enable();

    // Setup GPIOC
    // NOTE: PC2 and PC3 are only pinned out on TFBGA packages!!
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let pc2 = gpioc.pc2.into_analog(); // AIN 12
    let pc3 = gpioc.pc3.into_analog(); // AIN 13

    loop {
        let data_pc2 = adc1.convert(&pc2, AdcSampleTime::default());
        let data_pc3 = adc2.convert(&pc3, AdcSampleTime::default());
        // voltage = reading * (vref/resolution)
        info!("ADC readings: {} {}", data_pc2, data_pc3);
    }
}
