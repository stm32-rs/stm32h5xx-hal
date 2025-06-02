// Note: Code taken from stm32h7xx-hal
//! Random Number Generator
//!
//! # Examples
//!
//! - [Random Blinky](https://github.com/stm32-rs/stm32h5xx-hal/blob/master/examples/blinky_random.rs)

use core::cmp;
use core::marker::PhantomData;
use core::mem;

use crate::rcc::{rec, rec::RngClkSel};
use crate::rcc::{CoreClocks, ResetEnable};
use crate::stm32::RNG;
use crate::time::Hertz;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SeedError;

/// Return the kernel clock for the Random Number Generator
///
/// # Panics
///
/// Panics if the kernel clock is not running
fn kernel_clk_unwrap(prec: rec::Rng, clocks: &CoreClocks) -> Hertz {
    match prec.get_kernel_clk_mux() {
        //RngClkSel::Hsi48 => {
        RngClkSel::Hsi48Ker => {
            clocks.hsi48_ck().expect("RNG: HSI48 must be enabled")
        }
        RngClkSel::Pll1Q => {
            clocks.pll1().q_ck().expect("RNG: PLL1_Q must be enabled")
        }
        RngClkSel::Lse => unimplemented!(),
        RngClkSel::Lsi => clocks.lsi_ck().expect("RNG: LSI must be enabled"),
    }
}

fn setup_clocks(prec: rec::Rng, clocks: &CoreClocks) -> Hertz {
    let prec = prec.enable().reset();

    let hclk = clocks.hclk();
    let rng_clk = kernel_clk_unwrap(prec, clocks);

    // Otherwise clock checker will always flag an error
    // See RM0481 Rev 2 Section 32.3.6
    assert!(rng_clk > (hclk / 32), "RNG: Clock too slow");

    rng_clk
}

#[cfg(any(
    feature = "stm32h562",
    feature = "stm32h563",
    feature = "stm32h573",
))]

/// Note:
/// This uses the register values specified in AN4230 but verification
/// using this HAL has not been performed. Users can/should do their
/// own verification or request documentation from ST directly.
/// Requires RNG to be disabled since some register values can only be written when RNGEN = 0
pub trait RngNist {
    fn rng_nist_st_an4230(
        self,
        prec: rec::Rng,
        clocks: &CoreClocks,
    ) -> Rng<NIST>;
}

#[cfg(any(
    feature = "stm32h562",
    feature = "stm32h563",
    feature = "stm32h573"
))]
impl RngNist for RNG {
    /// Note:
    /// This uses the register values specified in AN4230 but verification
    /// using this HAL has not been performed. Users can/should do their
    /// own verification or request documentation from ST directly.
    /// Requires RNG to be disabled since some register values can only be written when RNGEN = 0
    fn rng_nist_st_an4230(
        self,
        prec: rec::Rng,
        clocks: &CoreClocks,
    ) -> Rng<NIST> {
        let rng_clk = setup_clocks(prec, clocks);

        // ST has tested this configuration only with a RNG clock of 48MHz
        assert_eq!(rng_clk, Hertz::MHz(48), "RNG: Clock not 48 MHz");

        // Set control register values, also need to write 1 to CONDRST to be able to set the other values
        self.cr()
            .write(|w| unsafe { w.bits(0x00F00E00).condrst().set_bit() });

        // Set health test control register values
        self.htcr().write(|w| unsafe { w.bits(0x6A91) });

        // Set noise source control register
        self.nscr().write(|w| unsafe { w.bits(0x3AF66) });

        // Configuration done, reset CONDRST, its value goes to 0 when the reset process is
        // done. It takes about 2 AHB clock cycles + 2 RNG clock cycles.
        self.cr().write(|w| w.condrst().clear_bit());

        // It should take about 2 AHB clock cycles + 2 RNG clock cycles
        while self.cr().read().condrst().bit_is_set() {}

        // Enable RNG
        self.cr().modify(|_, w| w.rngen().set_bit());

        Rng::new(self)
    }
}

pub trait RngExt {
    fn rng(self, prec: rec::Rng, clocks: &CoreClocks) -> Rng<NORMAL>;
    fn rng_fast(self, prec: rec::Rng, clocks: &CoreClocks) -> Rng<FAST>;
}

impl RngExt for RNG {
    /// This uses the register values specified in RM0481 Rev 2 section 32.6.2 RNG configuration C
    fn rng(self, prec: rec::Rng, clocks: &CoreClocks) -> Rng<NORMAL> {
        setup_clocks(prec, clocks);

        // Set control register values, also need to write 1 to CONDRST to be able to set the other values
        self.cr().write(|w| unsafe {
            w.nistc()
                .clear_bit()
                .rng_config1()
                .bits(0x0F)
                .clkdiv()
                .bits(0x0)
                .rng_config2()
                .bits(0x0)
                .rng_config3()
                .bits(0xD)
                .ced()
                .clear_bit()
                .condrst()
                .set_bit()
        });

        // Set health test control register values
        self.htcr().write(|w| unsafe { w.bits(0xAAC7) });

        // Set noise source control register
        // Note:
        // This is currently not available in the PAC or SVD for H503 but is planned to be added
        #[cfg(not(feature = "stm32h503"))]
        self.nscr().write(|w| unsafe { w.bits(0x0003FFFF) });

        // Configuration done, reset CONDRST, its value goes to 0 when the reset process is
        // done. It takes about 2 AHB clock cycles + 2 RNG clock cycles.
        self.cr().write(|w| w.condrst().clear_bit());

        // It should take about 2 AHB clock cycles + 2 RNG clock cycles
        while self.cr().read().condrst().bit_is_set() {}

        // Enable RNG
        self.cr().modify(|_, w| w.rngen().set_bit());

        Rng::new(self)
    }

    /// This uses the register values specified in RM0481 Rev 2 section 32.6.2 RNG configuration B
    fn rng_fast(self, prec: rec::Rng, clocks: &CoreClocks) -> Rng<FAST> {
        setup_clocks(prec, clocks);

        // Set control register values, also need to write 1 to CONDRST to be able to set the other values
        self.cr().write(|w| unsafe {
            w.nistc()
                .set_bit()
                .rng_config1()
                .bits(0x18)
                .clkdiv()
                .bits(0x0)
                .rng_config2()
                .bits(0x0)
                .rng_config3()
                .bits(0x0)
                .ced()
                .clear_bit()
                .condrst()
                .set_bit()
        });

        // Set health test control register values
        self.htcr().write(|w| unsafe { w.bits(0xAAC7) });

        // Set noise source control register
        #[cfg(not(feature = "stm32h503"))] // Not available on H503
        self.nscr().write(|w| unsafe { w.bits(0x0003FFFF) });

        // Configuration done, reset CONDRST, its value goes to 0 when the reset process is
        // done. It takes about 2 AHB clock cycles + 2 RNG clock cycles.
        self.cr().write(|w| w.condrst().clear_bit());

        // It should take about 2 AHB clock cycles + 2 RNG clock cycles
        while self.cr().read().condrst().bit_is_set() {}

        // Enable RNG
        self.cr().modify(|_, w| w.rngen().set_bit());

        Rng::new(self)
    }
}

pub trait RngCore<W> {
    fn gen(&mut self) -> Result<W, SeedError>;
    fn fill(&mut self, dest: &mut [W]) -> Result<(), SeedError>;
}

#[cfg(any(
    feature = "stm32h562",
    feature = "stm32h563",
    feature = "stm32h573"
))]
/// NIST mode (type state)
/// Use [RngNist::rng_nist_st_an4230] to generate [Rng] in this mode
pub struct NIST;
/// FAST mode (type state)
/// Use [RngExt::rng_fast] to generate [Rng] in this mode
pub struct FAST;
/// NORMAL mode (type state)
/// Use [RngExt::rng] to generate [Rng] in this mode
pub struct NORMAL;

pub struct Rng<MODE> {
    rb: RNG,
    _mode: PhantomData<MODE>,
}

impl<MODE> Rng<MODE> {
    fn new(rb: RNG) -> Self {
        Self {
            rb,
            _mode: PhantomData,
        }
    }
    /// Returns 32 bits of randomness, or error
    /// Automatically resets the seed error flag upon SeedError but will still return SeedError
    /// Upon receiving SeedError the user is expected to keep polling this function until a valid value is returned
    pub fn value(&mut self) -> Result<u32, SeedError> {
        'outer: loop {
            let status = self.rb.sr().read();

            if status.cecs().bit() {
                #[cfg(feature = "log")]
                log::warn!("RNG Clock error detected, retrying");

                #[cfg(feature = "defmt")]
                defmt::warn!("RNG Clock error detected, retrying");

                let sr = self.rb.sr();
                // Give rng some time to recover from clock disturbance, this time seems to be about a handful of milliseconds
                for _ in 0..100_000 {
                    if sr.read().cecs().bit_is_clear() {
                        continue 'outer;
                    }
                }
                panic!("Failed to automatically recover from Rng Clock Error");
            } else if status.secs().bit() {
                // Reset seed error flag so as to leave the peripheral in a valid state ready for use
                self.rb.sr().modify(|_, w| w.seis().clear_bit());
                return Err(SeedError);
            } else if status.drdy().bit() {
                return Ok(self.rb.dr().read().rndata().bits());
            }
        }
    }

    pub fn release(self) -> RNG {
        self.rb
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &RNG {
        &self.rb
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut RNG {
        &mut self.rb
    }
}

impl<MODE> core::iter::Iterator for Rng<MODE> {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            match self.value() {
                Ok(x) => return Some(x),
                // We recover automatically from a seed error, so try again
                Err(SeedError) => (),
            }
        }
    }
}

macro_rules! rng_core {
    ($($type:ty),+) => {
        $(
            impl<MODE> RngCore<$type> for Rng<MODE> {
                /// Returns a single element with random value, or error
                fn gen(&mut self) -> Result<$type, SeedError> {
                    let val = self.value()?;
                    Ok(val as $type)
                }

                /// Fills buffer with random values, or return error
                fn fill(&mut self, buffer: &mut [$type]) -> Result<(), SeedError> {
                    const BATCH_SIZE: usize = mem::size_of::<u32>() / mem::size_of::<$type>();
                    let mut i = 0_usize;
                    while i < buffer.len() {
                        let random_word = self.value()?;

                        // using to_ne_bytes does not work for u8 and would make the macro
                        // implementation more complicated
                        #[allow(clippy::transmute_num_to_bytes)]
                        let bytes: [$type; BATCH_SIZE] = unsafe { mem::transmute(random_word) };
                        let n = cmp::min(BATCH_SIZE, buffer.len() - i);
                        buffer[i..i + n].copy_from_slice(&bytes[..n]);
                        i += n;
                    }
                    Ok(())
                }
            }
        )+
    };
}

// Only for types larger than 32 bits
macro_rules! rng_core_large {
    ($($type:ty),+) => {
        $(
            impl<MODE> RngCore<$type> for Rng<MODE> {
                fn gen(&mut self) -> Result<$type, SeedError> {
                    const WORDS: usize = mem::size_of::<$type>() / mem::size_of::<u32>();
                    let mut res: $type = 0;

                    for i in 0..WORDS {
                        res |= (self.value()? as $type) << (i * (u32::BITS as usize))
                    }

                    Ok(res)
                }

                fn fill(&mut self, dest: &mut [$type]) -> Result<(), SeedError> {
                    let len = dest.len() * (mem::size_of::<$type>() / mem::size_of::<u32>());
                    let ptr = dest.as_mut_ptr() as *mut u32;
                    let slice_u32 = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
                    self.fill(slice_u32)
                }
            }
        )+
    };
}

macro_rules! rng_core_transmute {
    ($($type:ty = $from:ty),+) => {
        $(
            impl<MODE> RngCore<$type> for Rng<MODE> {
                fn gen(&mut self) -> Result<$type, SeedError> {
                    let num = <Self as RngCore<$from>>::gen(self)?;
                    Ok(unsafe { mem::transmute::<$from, $type>(num) })
                }

                fn fill(&mut self, dest: &mut [$type]) -> Result<(), SeedError> {
                    let unsigned_slice = unsafe { mem::transmute::<&mut [$type], &mut [$from]>(dest) };
                    <Self as RngCore<$from>>::fill(self, unsigned_slice)
                }
            }
        )+
    };
}

rng_core!(u8, u16, u32);

// Alignment of these types must be a multiple of mem::align_of::<32>()
rng_core_large!(u64, u128);

// A and B must have the same alignment
// rng_core_transmute!(A = B)
// assert!(mem::align_of::<A>() == mem::align_of::<B>())
rng_core_transmute!(
    i8 = u8,
    i16 = u16,
    i32 = u32,
    i64 = u64,
    i128 = u128,
    isize = usize
);

// If usize is 32 bits, use the rng_core! impl
#[cfg(target_pointer_width = "32")]
rng_core!(usize);

// If usize is 64 bits, use the rng_core_large! impl
#[cfg(target_pointer_width = "64")]
rng_core_large!(usize);

// rand_core
#[cfg(feature = "rand")]
#[cfg_attr(docsrs, doc(cfg(feature = "rand")))]
impl<MODE> rand_core::RngCore for Rng<MODE> {
    /// Generate a random u32
    /// Panics if RNG fails.
    fn next_u32(&mut self) -> u32 {
        self.gen().unwrap()
    }

    /// Generate a random u64
    /// Panics if RNG fails.
    fn next_u64(&mut self) -> u64 {
        self.gen().unwrap()
    }

    /// Fill a slice with random data.
    /// Panics if RNG fails.
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.fill(dest).unwrap()
    }

    /// Try to fill a slice with random data. Return an error if RNG fails.
    fn try_fill_bytes(
        &mut self,
        dest: &mut [u8],
    ) -> Result<(), rand_core::Error> {
        self.fill(dest).map_err(|e| {
            core::num::NonZeroU32::new(
                rand_core::Error::CUSTOM_START + e as u32,
            )
            // This should never fail as long as no enum variant is equal to 0
            .expect("Internal hal error")
            .into()
        })
    }
}

#[cfg(all(
    feature = "rand",
    any(feature = "stm32h562", feature = "stm32h563", feature = "stm32h573")
))]
#[cfg_attr(docsrs, doc(cfg(feature = "rand")))]
impl rand_core::CryptoRng for Rng<NIST> {}
