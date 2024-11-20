//! Reset and Clock Control
//!
//! This module configures the RCC unit to provide set frequencies for
//! the input to the `sys_ck`, the High-performance Bus (AHB) `hclk`,
//! the Peripheral (APB) Buses `pclkN` and the peripheral clock `per_ck`.
//!
//! See Figure 31 "Clock tree" in Reference Manual
//! RM0492 for more information (p 250).
//!
//! HSI is 64 MHz.
//! CSI is 4 MHz.
//! HSI48 is 48MHz.
//!
//! # Usage
//!
//! This peripheral must be used alongside the
//! [`PWR`](../pwr/index.html) peripheral to freeze voltage scaling of the
//! device.
//!
//! A builder pattern is used to specify the state and frequency of
//! possible clocks. The `freeze` method configures the RCC peripheral
//! in a best-effort attempt to generate these clocks. The actual
//! clocks configured are returned in `ccdr.clocks`.
//!
//! No clock specification overrides another. However supplying some
//! clock specifications may influence multiple resulting clocks,
//! including those corresponding to other clock specifications. This
//! is particularly the case for PLL clocks, where the frequencies of
//! adjacent 'P', 'Q, and 'R' clock outputs must have a simple integer
//! fraction relationship.
//!
//! Some clock specifications imply other clock specifications, as follows:
//!
//! * `use_hse(a)` implies `sys_ck(a)`
//!
//! * `sys_ck(b)` implies `pll1_p_ck(b)` unless `b` equals HSI or
//!   `use_hse(b)` was specified
//!
//! * `pll1_p_ck(c)` implies `pll1_r_ck(c/2)`, including when
//!   `pll1_p_ck` was implied by `sys_ck(c)` or `mco2_from_pll1_p_ck(c)`.
//!
//! Implied clock specifications can always be overridden by explicitly
//! specifying that clock. If this results in a configuration that cannot
//! be achieved by hardware, `freeze` will panic.
//!
//! # Examples
//!
//! - [Simple RCC example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/rcc.rs).
//! - [Fractional PLL configuration](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/fractional-pll.rs)
//! - [MCO example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/mco.rs)
//!
//! Simple example:
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let pwrcfg = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .sys_ck(96.MHz())
//!         .pclk1(48.MHz())
//!         .freeze(pwrcfg, &dp.SBS);
//! ```
//!
//! A more complex example, involving the PLL:
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let pwrcfg = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .sys_ck(200.MHz()) // Implies pll1_p_ck
//!         // For non-integer values, round up. `freeze` will never
//!         // configure a clock faster than that specified.
//!         .pll1_q_ck(33_333_334.hz())
//!         .freeze(pwrcfg, &dp.SBS);
//! ```
//!
//! A much more complex example, indicative of real usage with a
//! significant fraction of the STM32H5's capabilities.
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let pwrcfg = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .use_hse(25.MHz()) // XTAL X1
//!         .sys_ck(250.MHz())
//!         .pll1_r_ck(50.MHz())
//!         .pll1_q_ck(100.MHz())
//!         .hclk(250.MHz())
//!         .pll2_strategy(PllConfigStrategy::Fractional)
//!         .pll2_p_ck(240.MHz())
//!         .pll2_q_ck(48.MHz())
//!         .pll2_r_ck(26_666_667.Hz())
//!         .freeze(pwrcfg, &dp.SBS);
//!```
//!
//! # Peripherals
//!
//! The `freeze()` method returns a [Core Clocks Distribution and Reset
//! (CCDR)](struct.Ccdr.html) object. This singleton tells you how the core
//! clocks were actually configured (in [CoreClocks](struct.CoreClocks.html))
//! and allows you to configure the remaining peripherals (see
//! [PeripheralREC](crate::rcc::rec::struct.PeripheralREC.html)).
//!
//!```rust
//! let ccdr = ...; // Returned by `freeze()`, see examples above
//!
//! // Runtime confirmation that hclk really is 200MHz
//! assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000);
//!
//! // Panics if pll1_q_ck is not running
//! let _ = ccdr.clocks.pll1_q_ck().unwrap();
//!
//! // Enable the clock to a peripheral and reset it
//! ccdr.peripheral.FDCAN.enable().reset();
//!```
//!
//! The [PeripheralREC](struct.PeripheralREC.html) members implement move
//! semantics, so once you have passed them to a constructor they cannot be
//! modified again in safe Rust.
//!
#![deny(missing_docs)]

use crate::pwr::PowerConfiguration;
use crate::pwr::VoltageScale as Voltage;
#[cfg(feature = "rm0481")]
use crate::stm32::rcc::pll3cfgr::PLL3SRC;
use crate::stm32::rcc::{
    ccipr5::CKPERSEL, cfgr1::SW, cfgr1::TIMPRE, cfgr2::HPRE,
    cfgr2::PPRE1 as PPRE, pll1cfgr::PLL1SRC, pll2cfgr::PLL2SRC,
};
use crate::stm32::{RCC, SBS};
use crate::time::Hertz;

use core_clocks::PllClocks;
#[cfg(feature = "log")]
use log::debug;

mod core_clocks;
mod mco;
mod pll;
pub mod rec;
mod reset_reason;

pub use core_clocks::CoreClocks;
pub use pll::{PllConfig, PllConfigStrategy};
pub use rec::{LowPowerMode, PeripheralREC, ResetEnable};
pub use reset_reason::ResetReason;

use mco::{MCO1Config, MCO2Config, MCO1, MCO2};

/// Configuration of the core clocks
pub struct Config {
    hse: Option<u32>,
    bypass_hse: bool,
    lse: Option<u32>,
    sys_ck: Option<u32>,
    per_ck: Option<u32>,
    audio_ck: Option<u32>,
    rcc_hclk: Option<u32>,
    rcc_pclk1: Option<u32>,
    rcc_pclk2: Option<u32>,
    rcc_pclk3: Option<u32>,
    mco1: MCO1Config,
    mco2: MCO2Config,
    pll1: PllConfig,
    pll2: PllConfig,
    #[cfg(feature = "rm0481")]
    pll3: PllConfig,
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the
    /// other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            config: Config {
                hse: None,
                bypass_hse: false,
                lse: None,
                sys_ck: None,
                per_ck: None,
                audio_ck: None,
                rcc_hclk: None,
                rcc_pclk1: None,
                rcc_pclk2: None,
                rcc_pclk3: None,
                mco1: MCO1Config::default(),
                mco2: MCO2Config::default(),
                pll1: PllConfig::default(),
                pll2: PllConfig::default(),
                #[cfg(feature = "rm0481")]
                pll3: PllConfig::default(),
            },
            rb: self,
        }
    }
}

/// Constrained RCC peripheral
///
/// Generated by calling `constrain` on the PAC's RCC peripheral.
///
/// ```rust
/// let dp = stm32::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// ```
pub struct Rcc {
    config: Config,
    pub(crate) rb: RCC,
}

impl Rcc {
    /// Gets and clears the reason of why the mcu was reset
    pub fn get_reset_reason(&mut self) -> ResetReason {
        reset_reason::get_reset_reason(&mut self.rb)
    }
}

/// Core Clock Distribution and Reset (CCDR)
///
/// Generated when the RCC is frozen. The configuration of the Sys_Ck `sys_ck`,
/// AHB clocks `hclk`, APB clocks `pclkN` and PLL outputs `pllN_X_ck` are
/// frozen. However the distribution of some clocks may still be modified and
/// peripherals enabled / reset by passing this object to other implementations
/// in this stack.
pub struct Ccdr {
    /// A record of the frozen core clock frequencies
    pub clocks: CoreClocks,

    /// Peripheral reset / enable / kernel clock control
    pub peripheral: PeripheralREC,
}

const HSI: u32 = 32_000_000; // Hz
const CSI: u32 = 4_000_000; // Hz
const HSI48: u32 = 48_000_000; // Hz
const LSI: u32 = 32_000; // Hz

const MAX_SYSCLK_FREQ_HZ: u32 = 250_000_000;

/// Setter defintion for pclk 1 - 4
macro_rules! pclk_setter {
    ($($name:ident: $pclk:ident,)+) => {
        $(
            /// Set the peripheral clock frequency for APB
            /// peripherals.
            #[must_use]
            pub fn $name(mut self, freq: Hertz) -> Self {
                assert!(freq.raw() <= MAX_SYSCLK_FREQ_HZ,
                    "Max frequency is {MAX_SYSCLK_FREQ_HZ}Hz");
                self.config.$pclk = Some(freq.raw());
                self
            }
        )+
    };
}

/// Setter definition for pll 1 - 3 p, q, r
macro_rules! pll_setter {
    ($($pll:ident: [ $($name:ident: $ck:ident,)+ ],)+) => {
        $(
            $(
                /// Set the target clock frequency for PLL output
                #[must_use]
                pub fn $name(mut self, freq: Hertz) -> Self {
                    self.config.$pll.$ck = Some(freq.raw());
                    self
                }
            )+
        )+
    };
}

/// Setter definition for pll 1 - 3 strategy
macro_rules! pll_strategy_setter {
    ($($pll:ident: $name:ident,)+) => {
        $(
            /// Set the PLL divider strategy to be used when the PLL
            /// is configured
            #[must_use]
            pub fn $name(mut self, strategy: PllConfigStrategy) -> Self
            {
                self.config.$pll.strategy = strategy;
                self
            }
        )+
    }
}

impl Rcc {
    /// Uses HSE (external oscillator) instead of HSI (internal RC
    /// oscillator) as the clock source. Will result in a hang if an
    /// external oscillator is not connected or it fails to start.
    #[must_use]
    pub fn use_hse(mut self, freq: Hertz) -> Self {
        self.config.hse = Some(freq.raw());
        self
    }

    /// Use an external clock signal rather than a crystal oscillator,
    /// bypassing the XTAL driver.
    #[must_use]
    pub fn bypass_hse(mut self) -> Self {
        self.config.bypass_hse = true;
        self
    }

    /// Set SYSCLK frequency
    #[must_use]
    pub fn sys_ck(mut self, freq: Hertz) -> Self {
        assert!(
            freq.raw() <= MAX_SYSCLK_FREQ_HZ,
            "Max frequency is {MAX_SYSCLK_FREQ_HZ}Hz"
        );
        self.config.sys_ck = Some(freq.raw());
        self
    }

    /// Set SYSCLK frequency - ALIAS
    #[must_use]
    pub fn sysclk(self, freq: Hertz) -> Self {
        self.sys_ck(freq)
    }

    /// Set peripheral clock frequency
    #[must_use]
    pub fn per_ck(mut self, freq: Hertz) -> Self {
        self.config.per_ck = Some(freq.raw());
        self
    }

    /// Set low speed external clock frequency
    pub fn lse_ck(mut self, freq: Hertz) -> Self {
        self.config.lse = Some(freq.raw());
        self
    }

    /// Set external AUDIOCLK frequency
    #[must_use]
    pub fn audio_ck(mut self, freq: Hertz) -> Self {
        self.config.audio_ck = Some(freq.raw());
        self
    }

    /// Set the peripheral clock frequency for AHB peripherals.
    #[must_use]
    pub fn hclk(mut self, freq: Hertz) -> Self {
        assert!(
            freq.raw() <= MAX_SYSCLK_FREQ_HZ,
            "Max frequency is {MAX_SYSCLK_FREQ_HZ}Hz"
        );
        self.config.rcc_hclk = Some(freq.raw());
        self
    }

    pclk_setter! {
        pclk1: rcc_pclk1,
        pclk2: rcc_pclk2,
        pclk3: rcc_pclk3,
    }

    pll_setter! {
        pll1: [
            pll1_p_ck: p_ck,
            pll1_q_ck: q_ck,
            pll1_r_ck: r_ck,
        ],
        pll2: [
            pll2_p_ck: p_ck,
            pll2_q_ck: q_ck,
            pll2_r_ck: r_ck,
        ],
    }

    #[cfg(feature = "rm0481")]
    pll_setter! {
        pll3: [
            pll3_p_ck: p_ck,
            pll3_q_ck: q_ck,
            pll3_r_ck: r_ck,
        ],
    }

    pll_strategy_setter! {
        pll1: pll1_strategy,
        pll2: pll2_strategy,
    }

    #[cfg(feature = "rm0481")]
    pll_strategy_setter! {
        pll3: pll3_strategy,
    }
}

/// Divider calculator for pclk 1 - 4
///
/// Also calulate tim[xy]_ker_clk if there are timers on this bus
macro_rules! ppre_calculate {
    ($(($ppre:ident, $bits:ident): ($self: ident, $hclk: ident, $pclk: ident
                                    $(,$rcc_tim_ker_clk:ident, $timpre:ident)*),)+) => {
        $(
            // Get intended rcc_pclkN frequency
            let $pclk: u32 = $self.config
                .$pclk
                .unwrap_or($hclk);

            // Calculate suitable divider
            let ($bits, $ppre) = match $hclk.div_ceil($pclk)
            {
                0 => unreachable!(),
                1 => (PPRE::Div1, 1 as u8),
                2 => (PPRE::Div2, 2),
                3..=5 => (PPRE::Div4, 4),
                6..=11 => (PPRE::Div8, 8),
                _ => (PPRE::Div16, 16),
            };

            // Calculate real APBn clock
            let $pclk = $hclk / u32::from($ppre);

            $(
                let $rcc_tim_ker_clk = match ($bits, &$timpre)
                {
                    (PPRE::Div4, TIMPRE::DefaultX2) => $hclk / 2,
                    (PPRE::Div8, TIMPRE::DefaultX4) => $hclk / 2,
                    (PPRE::Div8, TIMPRE::DefaultX2) => $hclk / 4,
                    (PPRE::Div16, TIMPRE::DefaultX4) => $hclk / 4,
                    (PPRE::Div16, TIMPRE::DefaultX2) => $hclk / 8,
                    _ => $hclk,
                };
            )*
        )+
    };
}

impl Rcc {
    fn flash_setup(rcc_hclk: u32, vos: Voltage) {
        use crate::stm32::FLASH;
        // ACLK in MHz, round down and subtract 1 from integers. eg.
        // 61_999_999 -> 61MHz
        // 62_000_000 -> 61MHz
        // 62_000_001 -> 62MHz
        let rcc_hclk_mhz = Hertz::from_raw(rcc_hclk - 1).to_MHz();

        // See RM00492 Table 20
        let (wait_states, progr_delay) = match vos {
            // VOS 0 range VCORE 1.25V - 1.35V
            Voltage::Scale0 => match rcc_hclk_mhz {
                0..=41 => (0, 0),
                42..=83 => (1, 0),
                84..=125 => (2, 1),
                126..=167 => (3, 1),
                168..=209 => (4, 2),
                210..=250 => (5, 2),
                _ => (7, 3),
            },
            // VOS 1 range VCORE 1.15V - 1.25V
            Voltage::Scale1 => match rcc_hclk_mhz {
                0..=33 => (0, 0),
                34..=67 => (1, 0),
                68..=101 => (2, 1),
                102..=135 => (3, 1),
                136..=169 => (4, 2),
                170..=200 => (5, 2),
                _ => (7, 3),
            },
            // VOS 2 range VCORE 1.05V - 1.15V
            Voltage::Scale2 => match rcc_hclk_mhz {
                0..=29 => (0, 0),
                30..=59 => (1, 0),
                60..=89 => (2, 1),
                90..=119 => (3, 1),
                120..=150 => (4, 2),
                _ => (7, 3),
            },
            // VOS 3 range VCORE 0.95V - 1.05V
            Voltage::Scale3 => match rcc_hclk_mhz {
                0..=19 => (0, 0),
                20..=39 => (1, 0),
                40..=59 => (2, 1),
                60..=79 => (3, 1),
                80..=100 => (4, 2),
                _ => (7, 3),
            },
        };

        let flash = unsafe { &(*FLASH::ptr()) };
        // Adjust flash wait states
        flash.acr().write(|w| unsafe {
            w.wrhighfreq().bits(progr_delay).latency().bits(wait_states)
        });
        while flash.acr().read().latency().bits() != wait_states {}
    }

    /// Setup sys_ck
    /// Returns sys_ck frequency, and a pll1_p_ck
    fn sys_ck_setup(&mut self) -> (Hertz, bool) {
        // Compare available with wanted clocks
        let srcclk = self.config.hse.unwrap_or(HSI); // Available clocks
        let sys_ck = self.config.sys_ck.unwrap_or(srcclk);

        if sys_ck != srcclk {
            // The requested system clock is not the immediately available
            // HSE/HSI clock. Perhaps there are other ways of obtaining
            // the requested system clock (such as `HSIDIV`) but we will
            // ignore those for now.
            //
            // Therefore we must use pll1_p_ck
            let pll1_p_ck = match self.config.pll1.p_ck {
                Some(p_ck) => {
                    assert!(p_ck == sys_ck,
                            "Error: Cannot set pll1_p_ck independently as it must be used to generate sys_ck");
                    Some(p_ck)
                }
                None => Some(sys_ck),
            };
            self.config.pll1.p_ck = pll1_p_ck;

            (Hertz::from_raw(sys_ck), true)
        } else {
            // sys_ck is derived directly from a source clock
            // (HSE/HSI). pll1_p_ck can be as requested
            (Hertz::from_raw(sys_ck), false)
        }
    }

    /// Freeze the core clocks, returning a Core Clocks Distribution
    /// and Reset (CCDR) structure. The actual frequency of the clocks
    /// configured is returned in the `clocks` member of the CCDR
    /// structure.
    ///
    /// Note that `freeze` will never result in a clock _faster_ than
    /// that specified. It may result in a clock that is a factor of [1,
    /// 2) slower.
    ///
    /// `sbs` is required to enable the I/O compensation cell.
    ///
    /// # Panics
    ///
    /// If a clock specification cannot be achieved within the
    /// hardware specification then this function will panic. This
    /// function may also panic if a clock specification can be
    /// achieved, but the mechanism for doing so is not yet
    /// implemented here.
    pub fn freeze(mut self, pwrcfg: PowerConfiguration, sbs: &SBS) -> Ccdr {
        // We do not reset RCC here. This routine must assert when
        // the previous state of the RCC peripheral is unacceptable.

        // config modifications ----------------------------------------
        // (required for self-consistency and usability)

        // if needed for mco, set sys_ck / pll1_p / pll1_q / pll2_p
        self.mco1_setup();
        self.mco2_setup();

        // sys_ck from PLL if needed, else HSE or HSI
        let (sys_ck, sys_use_pll1_p) = self.sys_ck_setup();

        // self is now immutable ----------------------------------------
        let rcc = &self.rb;

        // Configure PLL1
        let (pll1_p_ck, pll1_q_ck, pll1_r_ck) =
            self.pll1_setup(rcc, &self.config.pll1);
        // Configure PLL2
        let (pll2_p_ck, pll2_q_ck, pll2_r_ck) =
            self.pll2_setup(rcc, &self.config.pll2);

        #[cfg(feature = "rm0481")]
        let (pll3_p_ck, pll3_q_ck, pll3_r_ck) =
            self.pll3_setup(rcc, &self.config.pll3);

        let sys_ck = if sys_use_pll1_p {
            pll1_p_ck.unwrap() // Must have been set by sys_ck_setup
        } else {
            sys_ck
        };

        // hsi_ck = HSI. This routine does not support HSIDIV != 1. To
        // do so it would need to ensure all PLLxON bits are clear
        // before changing the value of HSIDIV
        let hsi = HSI;
        assert!(
            rcc.cr().read().hsion().is_on(),
            "HSI oscillator must be on!"
        );
        assert!(
            rcc.cr().read().hsidiv().is_div2(),
            "HSI oscillator divider is not 2: {:?}",
            rcc.cr().read().hsidiv().variant()
        );

        let csi = CSI;
        let hsi48 = HSI48;

        // Enable LSI for RTC, IWDG, AWU, or MCO2
        let lsi = LSI;
        rcc.bdcr().modify(|_, w| w.lsion().enabled());
        while rcc.bdcr().read().lsirdy().is_not_ready() {}

        // per_ck from HSI by default
        let (per_ck, ckpersel) =
            match (self.config.per_ck == self.config.hse, self.config.per_ck) {
                (true, Some(hse)) => (hse, CKPERSEL::Hse), // HSE
                (_, Some(CSI)) => (csi, CKPERSEL::CsiKer), // CSI
                _ => (hsi, CKPERSEL::HsiKer),              // HSI
            };

        // Timer prescaler selection
        let timpre = TIMPRE::DefaultX2;

        // Get AHB clock or sensible default
        let rcc_hclk = self.config.rcc_hclk.unwrap_or(sys_ck.raw());

        // Estimate divisor
        let (hpre_bits, hpre_div) = match sys_ck.raw().div_ceil(rcc_hclk) {
            0 => unreachable!(),
            1 => (HPRE::Div1, 1),
            2 => (HPRE::Div2, 2),
            3..=5 => (HPRE::Div4, 4),
            6..=11 => (HPRE::Div8, 8),
            12..=39 => (HPRE::Div16, 16),
            40..=95 => (HPRE::Div64, 64),
            96..=191 => (HPRE::Div128, 128),
            192..=383 => (HPRE::Div256, 256),
            _ => (HPRE::Div512, 512),
        };

        // Calculate real AHB clock
        let rcc_hclk = sys_ck.raw() / hpre_div;

        // Calculate ppreN dividers and real rcc_pclkN frequencies
        ppre_calculate! {
            (ppre1, ppre1_bits): (self, rcc_hclk, rcc_pclk1, rcc_timx_ker_ck, timpre),
            (ppre2, ppre2_bits): (self, rcc_hclk, rcc_pclk2, rcc_timy_ker_ck, timpre),
            (ppre3, ppre3_bits): (self, rcc_hclk, rcc_pclk3),
        }

        // Calculate MCO dividers and real MCO frequencies
        let mco1_in = match self.config.mco1.source {
            // We set the required clock earlier, so can unwrap() here.
            MCO1::Hsi => HSI,
            MCO1::Lse => unimplemented!(),
            MCO1::Hse => self.config.hse.unwrap(),
            MCO1::Pll1Q => pll1_q_ck.unwrap().raw(),
            MCO1::Hsi48 => HSI48,
        };
        let (mco_1_pre, mco1_ck) =
            self.config.mco1.calculate_prescaler(mco1_in);

        let mco2_in = match self.config.mco2.source {
            // We set the required clock earlier, so can unwrap() here.
            MCO2::Sysclk => sys_ck.raw(),
            MCO2::Pll2P => pll2_p_ck.unwrap().raw(),
            MCO2::Hse => self.config.hse.unwrap(),
            MCO2::Pll1P => pll1_p_ck.unwrap().raw(),
            MCO2::Csi => CSI,
            MCO2::Lsi => LSI,
        };
        let (mco_2_pre, mco2_ck) =
            self.config.mco2.calculate_prescaler(mco2_in);

        // Start switching clocks here! ----------------------------------------

        // Flash setup
        Self::flash_setup(rcc_hclk, pwrcfg.vos);

        // Ensure CSI is on and stable
        rcc.cr().modify(|_, w| w.csion().on());
        while rcc.cr().read().csirdy().is_not_ready() {}

        // Ensure HSI48 is on and stable
        rcc.cr().modify(|_, w| w.hsi48on().on());
        while rcc.cr().read().hsi48rdy().is_not_ready() {}

        // Set the MCO outputs.
        //
        // It is highly recommended to configure these bits only after
        // reset, before enabling the external oscillators and the PLLs.
        rcc.cfgr1().modify(|_, w| {
            w.mco1sel()
                .variant(self.config.mco1.source)
                .mco1pre()
                .set(mco_1_pre)
                .mco2sel()
                .variant(self.config.mco2.source)
                .mco2pre()
                .set(mco_2_pre)
        });

        // HSE
        let hse_ck = match self.config.hse {
            Some(hse) => {
                // Ensure HSE is on and stable
                rcc.cr().modify(|_, w| {
                    w.hseon().on().hsebyp().bit(self.config.bypass_hse)
                });
                while rcc.cr().read().hserdy().is_not_ready() {}

                Some(Hertz::from_raw(hse))
            }
            None => None,
        };

        let lse_ck = self.config.lse.map(Hertz::from_raw);

        let audio_ck = self.config.audio_ck.map(Hertz::from_raw);

        // PLL
        let (pll1src, pll2src) = if self.config.hse.is_some() {
            (PLL1SRC::Hse, PLL2SRC::Hse)
        } else {
            (PLL1SRC::Hsi, PLL2SRC::Hsi)
        };
        rcc.pll1cfgr().modify(|_, w| w.pll1src().variant(pll1src));
        rcc.pll2cfgr().modify(|_, w| w.pll2src().variant(pll2src));
        #[cfg(feature = "rm0481")]
        {
            let pll3src = if self.config.hse.is_some() {
                PLL3SRC::Hse
            } else {
                PLL3SRC::Hsi
            };
            rcc.pll3cfgr().modify(|_, w| w.pll3src().variant(pll3src));
        }

        // PLL1
        if pll1_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr().modify(|_, w| w.pll1on().on());
            while rcc.cr().read().pll1rdy().is_not_ready() {}
        }

        // PLL2
        if pll2_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr().modify(|_, w| w.pll2on().on());
            while rcc.cr().read().pll2rdy().is_not_ready() {}
        }

        // PLL3
        #[cfg(feature = "rm0481")]
        if pll3_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr().modify(|_, w| w.pll3on().on());
            while rcc.cr().read().pll3rdy().is_not_ready() {}
        }

        // Core Prescaler / AHB Prescaler / APBx Prescalers
        rcc.cfgr2().modify(|_, w| {
            w.hpre()
                .variant(hpre_bits)
                .ppre1()
                .variant(ppre1_bits)
                .ppre2()
                .variant(ppre2_bits)
                .ppre3()
                .variant(ppre3_bits)
        });

        // Ensure core prescaler value is valid before future lower
        // core voltage
        while rcc.cfgr2().read().hpre().variant() != hpre_bits {}

        // Peripheral Clock (per_ck)
        rcc.ccipr5().modify(|_, w| w.ckpersel().variant(ckpersel));

        // Set timer clocks prescaler setting
        rcc.cfgr1().modify(|_, w| w.timpre().variant(timpre));

        // Select system clock source
        let swbits = match (sys_use_pll1_p, self.config.hse.is_some()) {
            (true, _) => SW::Pll1,
            (false, true) => SW::Hse,
            _ => SW::Hsi,
        };
        rcc.cfgr1().modify(|_, w| w.sw().variant(swbits));
        while rcc.cfgr1().read().sws().bits() != swbits.into() {}

        // IO compensation cell - Requires CSI clock and SYSCFG
        assert!(rcc.cr().read().csirdy().is_ready());
        rcc.apb3enr().modify(|_, w| w.sbsen().enabled());

        // Enable the compensation cell, using back-bias voltage code
        // provide by the cell.

        // PAC note: using the unmodified SBS field definitions as they're not defined for the
        // rm0481 MCUs at this time
        sbs.cccsr().modify(|_, w| {
            w.en1()
                .set_bit()
                .cs1()
                .clear_bit()
                .en2()
                .set_bit()
                .cs2()
                .clear_bit()
        });
        while sbs.cccsr().read().rdy1().bit_is_clear()
            || sbs.cccsr().read().rdy2().bit_is_clear()
        {}

        // This section prints the final register configuration for the main RCC registers:
        // - System Clock and PLL Source MUX
        // - PLL configuration
        // - System Prescalers
        // Does not include peripheral/MCO/RTC clock MUXes
        #[cfg(feature = "log")]
        {
            debug!("--- RCC register settings");

            let cfgr1 = rcc.cfgr1().read();
            debug!(
                "CFGR1 register: SWS (System Clock Mux)={:?}",
                cfgr1.sws().variant()
            );

            let cfgr2 = rcc.cfgr2().read();
            debug!(
                "CFGR2 register: HPRE={:?} PPRE1={:?} PPRE2={:?} PPRE3={:?}",
                cfgr2.hpre().variant(),
                cfgr2.ppre1().variant(),
                cfgr2.ppre2().variant(),
                cfgr2.ppre3().variant(),
            );

            let pll1cfgr = rcc.pll1cfgr().read();
            debug!(
                "PLL1CFGR register: PLL1SRC={:?} PLL1RGE={:?} PLL1FRACEN={:?} PLL1VCOSEL={:?} PLL1M={:#x} PLL1PEN={:?} PLL1QEN={:?} PLL1REN={:?}",
                pll1cfgr.pll1src().variant(),
                pll1cfgr.pll1rge().variant(),
                pll1cfgr.pll1fracen().variant(),
                pll1cfgr.pll1vcosel().variant(),
                pll1cfgr.pll1m().bits(),
                pll1cfgr.pll1pen().variant(),
                pll1cfgr.pll1qen().variant(),
                pll1cfgr.pll1ren().variant(),
            );

            let pll1divr = rcc.pll1divr().read();
            debug!(
                "PLL1DIVR register: PLL1N={:#x} PLL1P={:#x} PLL1Q={:#x} PLL1R={:#x}",
                pll1divr.pll1n().bits(),
                pll1divr.pll1p().bits(),
                pll1divr.pll1q().bits(),
                pll1divr.pll1r().bits(),
            );

            let pll1fracr = rcc.pll1fracr().read();
            debug!(
                "PLL1FRACR register: FRACN1={:#x}",
                pll1fracr.pll1fracn().bits(),
            );

            let pll2cfgr = rcc.pll2cfgr().read();
            debug!(
                "PLL2CFGR register: PLL2SRC={:?} PLL2RGE={:?} PLL2FRACEN={:?} PLL2VCOSEL={:?} PLL2M={:#x} PLL2PEN={:?} PLL2QEN={:?} PLL2REN={:?}",
                pll2cfgr.pll2src().variant(),
                pll2cfgr.pll2rge().variant(),
                pll2cfgr.pll2fracen().variant(),
                pll2cfgr.pll2vcosel().variant(),
                pll2cfgr.pll2m().bits(),
                pll2cfgr.pll2pen().variant(),
                pll2cfgr.pll2qen().variant(),
                pll2cfgr.pll2ren().variant(),
            );

            let pll2divr = rcc.pll2divr().read();
            debug!(
                "PLL2DIVR register: PLL2N={:#x} PLL2P={:#x} PLL2Q={:#x} PLL2R={:#x}",
                pll2divr.pll2n().bits(),
                pll2divr.pll2p().bits(),
                pll2divr.pll2q().bits(),
                pll2divr.pll2r().bits(),
            );

            let pll2fracr = rcc.pll2fracr().read();
            debug!(
                "PLL2FRACR register: FRACN2={:#x}",
                pll2fracr.pll2fracn().bits(),
            );

            #[cfg(feature = "rm0481")]
            {
                let pll3cfgr = rcc.pll3cfgr().read();
                debug!(
                    "PLL3CFGR register: PLL3SRC={:?} PLL3RGE={:?} PLL3FRACEN={:?} PLL3VCOSEL={:?} PLL3M={:#x} PLL3PEN={:?} PLL3QEN={:?} PLL3REN={:?}",
                    pll3cfgr.pll3src().variant(),
                    pll3cfgr.pll3rge().variant(),
                    pll3cfgr.pll3fracen().variant(),
                    pll3cfgr.pll3vcosel().variant(),
                    pll3cfgr.pll3m().bits(),
                    pll3cfgr.pll3pen().variant(),
                    pll3cfgr.pll3qen().variant(),
                    pll3cfgr.pll3ren().variant(),
                );

                let pll3divr = rcc.pll3divr().read();
                debug!(
                    "PLL3DIVR register: PLL3N={:#x} PLL3P={:#x} PLL3Q={:#x} PLL3R={:#x}",
                    pll3divr.pll3n().bits(),
                    pll3divr.pll3p().bits(),
                    pll3divr.pll3q().bits(),
                    pll3divr.pll3r().bits(),
                );

                let pll3fracr = rcc.pll3fracr().read();
                debug!(
                    "PLL3FRACR register: FRACN2={:#x}",
                    pll3fracr.pll3fracn().bits(),
                );
            }
        }

        let pll1 = PllClocks::new(pll1_p_ck, pll1_q_ck, pll1_r_ck);
        let pll2 = PllClocks::new(pll2_p_ck, pll2_q_ck, pll2_r_ck);
        #[cfg(feature = "rm0481")]
        let pll3 = PllClocks::new(pll3_p_ck, pll3_q_ck, pll3_r_ck);

        // Return frozen clock configuration
        Ccdr {
            clocks: CoreClocks {
                hclk: Hertz::from_raw(rcc_hclk),
                pclk1: Hertz::from_raw(rcc_pclk1),
                pclk2: Hertz::from_raw(rcc_pclk2),
                pclk3: Hertz::from_raw(rcc_pclk3),
                ppre1,
                ppre2,
                ppre3,
                csi_ck: Some(Hertz::from_raw(csi)),
                hsi_ck: Some(Hertz::from_raw(hsi)),
                hsi48_ck: Some(Hertz::from_raw(hsi48)),
                lsi_ck: Some(Hertz::from_raw(lsi)),
                per_ck: Some(Hertz::from_raw(per_ck)),
                hse_ck,
                lse_ck,
                audio_ck,
                mco1_ck,
                mco2_ck,
                pll1,
                pll2,
                #[cfg(feature = "rm0481")]
                pll3,
                timx_ker_ck: Hertz::from_raw(rcc_timx_ker_ck),
                timy_ker_ck: Hertz::from_raw(rcc_timy_ker_ck),
                sys_ck,
            },
            peripheral: unsafe {
                // unsafe: we consume self which was a singleton, hence
                // we can safely create a singleton here
                PeripheralREC::new_singleton()
            },
        }
    }
}
