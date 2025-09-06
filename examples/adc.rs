//! Example of reading a voltage with ADC1
//!
//! For an example of using ADC1 and ADC2 together, see examples/adc12.rs

#![no_main]
#![no_std]

use cortex_m_rt::entry;

use embedded_hal_02::adc::OneShot;
use stm32h5xx_hal::{
    adc::{self, AdcCommonExt},
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
    info!("stm32h5xx-hal example - ADC");
    info!("");

    let mut delay = Delay::new(cp.SYST, &ccdr.clocks);

    #[cfg(feature = "rm0481")]
    let adc = dp
        .ADCC
        .claim(4.MHz(), ccdr.peripheral.ADC, &ccdr.clocks, &pwrcfg)
        .claim_and_configure(dp.ADC1, &mut delay);

    #[cfg(feature = "rm0492")]
    let adc = dp
        .ADC1
        .claim(4.MHz(), ccdr.peripheral.ADC, &ccdr.clocks, &pwrcfg)
        .claim_and_configure(&mut delay);

    // Setup ADC
    let mut adc1 = adc::Adc::new(
        dp.ADC1,
        4.MHz(),
        &mut delay,
        ccdr.peripheral.ADC,
        &ccdr.clocks,
        &pwrcfg,
    );

    let mut temp = adc::Temperature::new();
    temp.enable(&mut adc1);
    let mut adc1: adc::Adc<
        stm32h5::Periph<pac::adc1::RegisterBlock, 1107460096>,
        adc::Enabled,
    > = adc1.enable();

    // We can't use ADC2 here because ccdr.peripheral.ADC12 has been
    // consumed. See examples/adc12.rs

    // Setup GPIOC
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Configure pc0 as an analog input
    let mut channel = gpioc.pc0.into_analog(); // ANALOG IN 10

    loop {
        let data = adc1.read(&mut channel).unwrap();
        // voltage = reading * (vref/resolution)
        info!(
            "ADC reading: {}, voltage for nucleo: {}V. Temp reading: {}",
            data,
            data as f32 * (3.3 / adc1.slope() as f32),
            adc1.read(&mut temp).unwrap()
        );
    }
}
