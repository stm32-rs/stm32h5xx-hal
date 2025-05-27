//! Phase Locked Loop Configuration

use core::ops::RangeInclusive;

use super::{Rcc, HSI};
use crate::stm32::rcc::pll1cfgr::PLL1VCOSEL as VCOSEL;
use crate::stm32::RCC;
use crate::time::Hertz;

const FRACN_DIVISOR: f32 = 8192.0; // 2 ** 13
const FRACN_MAX: f32 = FRACN_DIVISOR - 1.0;

const PLL_N_MIN: u32 = 4;
const PLL_N_MAX: u32 = 512;
const PLL_OUT_DIV_MAX: u32 = 128;
const PLL_M_MAX: u32 = 63;

/// Strategies for configuring a Phase Locked Loop (PLL)
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum PllConfigStrategy {
    /// No fractional component
    Integer,
    /// PLL configured with fractional divider
    Fractional,
}

/// Configuration of a Phase Locked Loop (PLL)
pub struct PllConfig {
    pub(super) strategy: PllConfigStrategy,
    pub(super) p_ck: Option<u32>,
    pub(super) q_ck: Option<u32>,
    pub(super) r_ck: Option<u32>,
}
impl Default for PllConfig {
    fn default() -> PllConfig {
        PllConfig {
            strategy: PllConfigStrategy::Integer,
            p_ck: None,
            q_ck: None,
            r_ck: None,
        }
    }
}

#[derive(Debug)]
struct VcoRange {
    output_range: RangeInclusive<u32>,
    input_range: RangeInclusive<u32>,
    vcosel: VCOSEL,
}

const VCO_RANGE_MEDIUM: VcoRange = VcoRange {
    output_range: 150_000_000..=420_000_000,
    input_range: 1_000_000..=2_000_000,
    vcosel: VCOSEL::MediumVco,
};
const VCO_RANGE_WIDE: VcoRange = VcoRange {
    output_range: 128_000_000..=560_000_000,
    input_range: 2_000_000..=16_000_000,
    vcosel: VCOSEL::WideVco,
};

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct PllOutput {
    ck: u32,
    div: u32,
}

#[derive(Debug)]
struct PllSetup {
    vco_range: VcoRange,
    vco_out_target: u32,
    ref_ck: u32,
    pll_m: u32,
    pll_p: Option<PllOutput>,
    pll_q: Option<PllOutput>,
    pll_r: Option<PllOutput>,
}

/// Calculate VCO output divider (p-divider). Choose the highest VCO
/// frequency to give specified output.
///
/// Returns *target* VCO frequency and p-divider
///
fn vco_output_divider_setup(
    pllsrc: u32,
    pllcfg: &PllConfig,
    p_odd_allowed: bool,
) -> PllSetup {
    let outputs = [
        (pllcfg.p_ck, p_odd_allowed),
        (pllcfg.q_ck, true),
        (pllcfg.r_ck, true),
    ]
    .into_iter()
    .filter(|it| it.0.is_some())
    // Multiply output for PLL where P cannot be odd, so that the max output frequency
    // considered is PLLP * 2
    .map(|it| {
        if it.1 {
            it.0.unwrap()
        } else {
            it.0.unwrap() * 2
        }
    });

    let (min_output, max_output): (u32, u32) =
        outputs.fold((0, 0), |minmax, ck| {
            let min = if minmax.0 == 0 { ck } else { minmax.0.min(ck) };
            let max = minmax.1.max(ck);
            (min, max)
        });

    let vco_ranges = [VCO_RANGE_MEDIUM, VCO_RANGE_WIDE];

    // Use max output frequency required for VCO out to minimize dividers
    assert!(
        min_output >= (max_output / PLL_OUT_DIV_MAX),
        "VCO cannot be configured for given PLL spec"
    );

    let range = vco_ranges
        .into_iter()
        .find(|range| range.output_range.end() > &max_output)
        .expect("Target frequency is higher than PLL max frequency");

    let vco_max: u32 = *range.output_range.end();
    let min_div = vco_max / max_output;
    let vco_out_target = max_output * min_div;

    let vco_out_target = if (vco_out_target / min_output) > PLL_OUT_DIV_MAX {
        let f = (vco_out_target / min_output).div_ceil(PLL_OUT_DIV_MAX);
        vco_out_target / f
    } else {
        vco_out_target
    };
    assert!(range.output_range.contains(&vco_out_target));

    let pll_x_p = pllcfg.p_ck.map(|ck| PllOutput {
        ck,
        div: vco_out_target / ck,
    });
    let pll_x_q = pllcfg.q_ck.map(|ck| PllOutput {
        ck,
        div: vco_out_target / ck,
    });
    let pll_x_r = pllcfg.r_ck.map(|ck| PllOutput {
        ck,
        div: vco_out_target / ck,
    });

    // Input divisor, resulting in a reference clock in the
    // range 2 to 16 MHz.
    let pll_x_m_min = pllsrc.div_ceil(*range.input_range.end());
    let pll_x_m_max = (pllsrc / range.input_range.start()).min(PLL_M_MAX);

    // Iterative search for the lowest m value that minimizes
    // the difference between requested and actual VCO frequency
    let pll_x_m = (pll_x_m_min..=pll_x_m_max)
        .min_by_key(|pll_x_m| {
            let ref_x_ck = pllsrc / pll_x_m;

            // Feedback divider. Integer only
            let pll_x_n = vco_out_target / ref_x_ck;

            vco_out_target as i32 - (ref_x_ck * pll_x_n) as i32
        })
        .unwrap();

    assert!(pll_x_m <= PLL_M_MAX);

    // Calculate resulting reference clock
    let ref_ck = pllsrc / pll_x_m;
    assert!(range.input_range.contains(&ref_ck));

    PllSetup {
        vco_range: range,
        vco_out_target,
        ref_ck,
        pll_m: pll_x_m,
        pll_p: pll_x_p,
        pll_q: pll_x_q,
        pll_r: pll_x_r,
    }
}

/// Setup PFD input frequency and VCO output frequency
///
macro_rules! vco_setup {
    ($pllsrc:ident, $pllcfg:expr, $rcc:ident, $pllX:ident, $odd_allowed:expr)
    => { paste::item! {{

         // VCO output frequency limits
         let pll_setup = vco_output_divider_setup($pllsrc, $pllcfg, $odd_allowed);

         // Configure VCO
         $rcc.[<$pllX cfgr>]().modify(|_, w| {
            match pll_setup.vco_range.vcosel {
                VCOSEL::MediumVco => w.[<$pllX vcosel>]().medium_vco(),
                VCOSEL::WideVco => w.[<$pllX vcosel>]().wide_vco(),
            }
         });
         $rcc.[<$pllX cfgr>]().modify(|_, w| {
             match pll_setup.ref_ck {
                1_000_000 ..= 1_999_999=> w.[<$pllX rge>]().range1(),
                2_000_000 ..= 3_999_999 => w.[<$pllX rge>]().range2(),
                4_000_000 ..= 7_999_999 => w.[<$pllX rge>]().range4(),
                 _ => w.[<$pllX rge>]().range8(),
             }
         });

         pll_setup
     }}};
}

macro_rules! pll_divider_setup {
    (($pllX:ident, $rcc:expr, $pll:expr, $vco_ck:expr) $(,$d:ident: $pll_x_d:expr)*) => {
        paste::item! {{
            $(
                let [<$d _ck_hz>] = if let Some(pll_x_d) = $pll_x_d {
                    // Bounds check

                    // Setup divider
                    $rcc.[<$pllX divr>]().modify(|_, w|
                        w.[<$pllX $d>]().set((pll_x_d.div - 1) as u8)
                    );
                    $rcc.[<$pllX cfgr>]().modify(|_, w| w.[<$pllX $d en>]().enabled());
                    Some(Hertz::from_raw($vco_ck / pll_x_d.div))
                } else {
                    $rcc.[< $pllX cfgr>]().modify(|_, w| w.[<$pllX $d en>]().disabled());
                    None
                };
            )*
            ($([<$d _ck_hz>],)*)
        }}
    };
}

macro_rules! pll_setup {
    ($pllX:ident, $odd_allowed:expr) => {
        paste::item! {
            /// PLL Setup
            /// Returns (Option(pllX_p_ck), Option(pllX_q_ck), Option(pllX_r_ck))

            pub(super) fn [< $pllX _setup >] (
                &self,
                rcc: &RCC,
                pll: &PllConfig,
            ) -> (Option<Hertz>, Option<Hertz>, Option<Hertz>) {
                // PLL sourced from either HSE or HSI
                let pllsrc = self.config.hse.unwrap_or(HSI);
                assert!(pllsrc > 0);

                // PLL output. Use P, Q or R ck in that order of preference
                if pll.p_ck.or(pll.q_ck.or(pll.r_ck)).is_none() {
                    return (None, None, None);
                }

                let pll_setup = vco_setup!(pllsrc, pll, rcc, $pllX, $odd_allowed);

                // Feedback divider. Integer only
                let pll_x_n = pll_setup.vco_out_target / pll_setup.ref_ck;

                // Write dividers
                rcc.[< $pllX cfgr >]().modify(|_, w| unsafe {
                    w.[< $pllX m >]()
                    .bits(pll_setup.pll_m as u8) });  // ref prescaler

                // unsafe as not all values are permitted: see RM0492
                assert!(pll_x_n >= PLL_N_MIN);
                assert!(pll_x_n <= PLL_N_MAX);
                rcc.[<$pllX divr>]().modify(|_, w| unsafe { w.[<$pllX n>]().bits((pll_x_n - 1) as u16) });

                let pll_x = pll_setup.pll_p.as_ref().or(pll_setup.pll_q.as_ref().or(pll_setup.pll_r.as_ref())).unwrap();

                // Configure N divider. Returns the resulting VCO frequency
                let vco_ck = match pll.strategy {
                    PllConfigStrategy::Fractional => {
                        // Calculate FRACN
                        let pll_x_fracn = calc_fracn(pll_setup.ref_ck as f32, pll_x_n as f32, pll_x.div as f32, pll_x.ck as f32);
                        //RCC_PLL1FRACR
                        rcc.[<$pllX fracr>]().modify(|_, w| w.[<$pllX fracn>]().set(pll_x_fracn));
                        // Latch FRACN by resetting and setting it
                        rcc.[<$pllX cfgr>]().modify(|_, w| w.[< $pllX fracen>]().reset() );
                        rcc.[<$pllX cfgr>]().modify(|_, w| w.[< $pllX fracen>]().set_() );

                        calc_vco_ck(pll_setup.ref_ck, pll_x_n, pll_x_fracn)
                    },
                    // Iterative
                    _ => {
                        pll_setup.ref_ck * pll_x_n
                    },
                };

                pll_divider_setup! {($pllX, rcc, pll, vco_ck),
                                        p: &pll_setup.pll_p,
                                        q: &pll_setup.pll_q,
                                        r: &pll_setup.pll_r }
            }
        }
    };
}

/// Calcuate the Fractional-N part of the divider
///
/// ref_clk - Frequency at the PFD input
/// pll_n - Integer-N part of the divider
/// pll_p - P-divider
/// output - Wanted output frequency
fn calc_fracn(ref_clk: f32, pll_n: f32, pll_p: f32, output: f32) -> u16 {
    // VCO output frequency = Fref1_ck x (DIVN1 + (FRACN1 / 2^13)),
    let pll_fracn = FRACN_DIVISOR * (((output * pll_p) / ref_clk) - pll_n);
    assert!(pll_fracn >= 0.0);
    assert!(pll_fracn <= FRACN_MAX);
    // Rounding down by casting gives up the lowest without going over
    pll_fracn as u16
}

/// Calculates the VCO output frequency
///
/// ref_clk - Frequency at the PFD input
/// pll_n - Integer-N part of the divider
/// pll_fracn - Fractional-N part of the divider
fn calc_vco_ck(ref_ck: u32, pll_n: u32, pll_fracn: u16) -> u32 {
    (ref_ck as f32 * (pll_n as f32 + (pll_fracn as f32 / FRACN_DIVISOR))) as u32
}

impl Rcc {
    pll_setup! {pll1, false}
    pll_setup! {pll2, true}
    #[cfg(feature = "rm0481")]
    pll_setup! {pll3, true}
}

#[cfg(test)]
mod tests {
    use super::VCOSEL;
    use crate::rcc::{
        pll::{
            calc_fracn, calc_vco_ck, vco_output_divider_setup,
            PllConfigStrategy, PLL_OUT_DIV_MAX,
        },
        PllConfig,
    };

    macro_rules! dummy_method {
        ($($name:ident),+) => (
            $(
                fn $name(self) -> Self {
                    self
                }
            )+
        )
    }

    // Mock PLL CFGR
    struct WPllCfgr {}
    impl WPllCfgr {
        dummy_method! { pll1vcosel, medium_vco, wide_vco }
        dummy_method! { pll1rge, range1, range2, range4, range8 }
    }
    struct MockPllCfgr {}
    impl MockPllCfgr {
        // Modify mock registers
        fn modify<F>(&self, func: F)
        where
            F: FnOnce((), WPllCfgr) -> WPllCfgr,
        {
            func((), WPllCfgr {});
        }
    }

    // Mock RCC
    struct MockRcc {
        pll1cfgr: MockPllCfgr,
    }
    impl MockRcc {
        pub fn new() -> Self {
            MockRcc {
                pll1cfgr: MockPllCfgr {},
            }
        }

        fn pll1cfgr(&self) -> &MockPllCfgr {
            &self.pll1cfgr
        }
    }

    #[test]
    fn vco_setup_integer() {
        let rcc = MockRcc::new();

        let pllsrc = 25_000_000; // PLL source frequency eg. 25MHz crystal
        let pll_p_target = 242_000_000; // PLL output frequency (P_CK)
        let pll_q_target = 120_900_000; // PLL output frequency (Q_CK)
        let pll_r_target = 30_200_000; // PLL output frequency (R_CK)
        let pllcfg = PllConfig {
            strategy: PllConfigStrategy::Integer,
            p_ck: Some(pll_p_target),
            q_ck: Some(pll_q_target),
            r_ck: Some(pll_r_target),
        };
        println!(
            "PLL2/3 {} MHz -> {} MHz",
            pllsrc as f32 / 1e6,
            pll_p_target as f32 / 1e6
        );

        // ----------------------------------------

        // VCO Setup
        let pll_setup = vco_setup! {
           pllsrc, &pllcfg, rcc, pll1, true
        };

        println!("\nPLL setup: {pll_setup:?}\n");
        // Feedback divider. Integer only
        let pll_x_n = pll_setup.vco_out_target / pll_setup.ref_ck;
        // Resulting achieved vco_ck
        let vco_ck_achieved = calc_vco_ck(pll_setup.ref_ck, pll_x_n, 0);

        // ----------------------------------------

        // Input
        println!("M Divider {}", pll_setup.pll_m);
        let input = pllsrc as f32 / pll_setup.pll_m as f32;
        println!("==> Input {} MHz", input / 1e6);
        println!();
        assert!((input >= 1e6) && (input < 2e6), "input: {}", input);

        println!(
            "VCO CK Target {} MHz",
            pll_setup.vco_out_target as f32 / 1e6
        );
        println!("VCO CK Achieved {} MHz", vco_ck_achieved as f32 / 1e6);
        println!();

        // Output
        let pll_x_p = pll_setup.pll_p.unwrap().div;
        println!("P Divider {}", pll_x_p);
        let output_p = vco_ck_achieved as f32 / pll_x_p as f32;
        println!("==> Output {} MHz", output_p / 1e6);

        let error = output_p - pll_p_target as f32;
        println!(
            "Error {} {}",
            f32::abs(error),
            (pll_p_target as f32 / 100.0)
        );
        assert!(f32::abs(error) < (pll_p_target as f32 / 100.0)); // < ±1% error
        println!();

        let pll_x_q = pll_setup.pll_q.unwrap().div;
        let output_q = vco_ck_achieved as f32 / pll_x_q as f32;
        println!("Q Divider {}", pll_x_q);
        println!("==> Output Q {} MHz", output_q / 1e6);
        println!();
        let error = output_q - pll_q_target as f32;
        assert!(f32::abs(error) < (pll_q_target as f32 / 100.0)); // < ±1% error

        let pll_x_r = pll_setup.pll_r.unwrap().div;
        let output_r = vco_ck_achieved as f32 / pll_x_r as f32;
        println!("R Divider {}", pll_x_r);
        println!("==> Output Q {} MHz", output_r / 1e6);
        println!();
        let error = output_r - pll_r_target as f32;
        assert!(f32::abs(error) < (pll_r_target as f32 / 100.0)); // < ±1% error
    }

    #[test]
    fn vco_setup_fractional() {
        let rcc = MockRcc::new();

        let pllsrc = 16_000_000; // PLL source frequency eg. 16MHz crystal
        let pll_p_target = 48_000 * 256; // Target clock = 12.288MHz
        let pll_q_target = 48_000 * 128; // Target clock = 6.144MHz
        let pll_r_target = 48_000 * 63; // Target clock = 3.024MHz
        let pllcfg = PllConfig {
            strategy: PllConfigStrategy::Integer,
            p_ck: Some(pll_p_target),
            q_ck: Some(pll_q_target),
            r_ck: Some(pll_r_target),
        };
        let output = pll_p_target; // PLL output frequency (P_CK)
        println!(
            "PLL2/3 {} MHz -> {} MHz",
            pllsrc as f32 / 1e6,
            output as f32 / 1e6
        );

        // ----------------------------------------

        // VCO Setup
        let pll_setup = vco_setup! {
            pllsrc, &pllcfg, rcc, pll1, true
        };
        let input = pllsrc as f32 / pll_setup.pll_m as f32;

        println!("\nPLL setup: {pll_setup:?}\n");
        // Feedback divider. Integer only
        let pll_x_n = pll_setup.vco_out_target / pll_setup.ref_ck;
        let pll_x_p = pll_setup.pll_p.unwrap().div;
        let pll_x_fracn = calc_fracn(
            input as f32,
            pll_x_n as f32,
            pll_x_p as f32,
            output as f32,
        );
        println!("FRACN Divider {}", pll_x_fracn);
        // Resulting achieved vco_ck
        let vco_ck_achieved =
            calc_vco_ck(pll_setup.ref_ck, pll_x_n, pll_x_fracn);

        // Calulate additional output dividers
        let pll_x_q = pll_setup.pll_q.unwrap().div;
        let pll_x_r = pll_setup.pll_r.unwrap().div;
        assert!(pll_x_p <= PLL_OUT_DIV_MAX);
        assert!(pll_x_q <= PLL_OUT_DIV_MAX);
        assert!(pll_x_r <= PLL_OUT_DIV_MAX);

        // ----------------------------------------

        // Input
        println!("M Divider {}", pll_setup.pll_m);
        println!("==> Input {} MHz", input / 1e6);
        println!();

        println!(
            "VCO CK Target {} MHz",
            pll_setup.vco_out_target as f32 / 1e6
        );
        println!("VCO CK Achieved {} MHz", vco_ck_achieved as f32 / 1e6);
        println!();

        // Output
        let output_p = vco_ck_achieved as f32 / pll_x_p as f32;
        println!("P Divider {}", pll_x_p);
        println!("==> Output P {} MHz", output_p / 1e6);
        println!();

        // The P_CK should be very close to the target with a finely tuned FRACN
        //
        // The other clocks accuracy will vary depending on how close
        // they are to an integer fraction of the P_CK
        assert!(output_p <= pll_p_target as f32);
        let error_p = output_p - pll_p_target as f32;
        assert!(
            f32::abs(error_p) < (pll_p_target as f32 / 500_000.0),
            "P error too large: {error_p}"
        ); // < ±.0002% = 2ppm error

        let output_q = vco_ck_achieved as f32 / pll_x_q as f32;
        let error_q = output_q - pll_q_target as f32;
        assert!(
            f32::abs(error_q) < (pll_q_target as f32 / 100.0),
            "Q error too large: {error_q}"
        ); // < ±1% error
        println!("Q Divider {}", pll_x_q);
        println!("==> Output Q {} MHz", output_q / 1e6);
        println!();

        let output_r = vco_ck_achieved as f32 / pll_x_r as f32;
        let error_r = output_r - pll_r_target as f32;
        assert!(
            f32::abs(error_r) < (pll_r_target as f32 / 100.0),
            "R error too large: {error_r}"
        ); // < ±1% error
        println!("R Divider {}", pll_x_r);
        println!("==> Output R {} MHz", output_r / 1e6);
        println!();
    }
}
