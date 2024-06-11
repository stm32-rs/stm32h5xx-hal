//! Time units

pub use fugit::{
    HertzU32 as Hertz, KilohertzU32 as KiloHertz, MegahertzU32 as MegaHertz,
    MicrosDurationU32 as MicroSeconds, MillisDurationU32 as MilliSeconds,
    NanosDurationU32 as NanoSeconds,
};

//use core::time::Duration;
use cortex_m::peripheral::DWT;

/// Bits per second
pub type Bps = Hertz;

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps::from_raw(self)
    }
}

/// A measurement of a monotonically nondecreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(&self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.now)
    }
}
