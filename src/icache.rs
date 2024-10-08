//! Instruction Cache management
//!
//! To enable the the ICACHE:
//!
//! ```
//! let dp = ...;            // Device peripherals
//! let mut icache = dp.ICACHE.constrain();
//!
//! icache.enable();
//! ```
//!
//! To invalidate the cache, use the ICache::invalidate() method:
//!
//! ```
//! icache.invalidate();
//! ```
//!
//! Performance monitoring of the cache is possible using the cache hit and
//! miss counters:
//!
//! ```
//! icache.enable_hit_counter();
//! icache.enable_miss_counter();
//!
//! // do something
//!
//! let hit_count = icache.hit_count();
//! let miss_count = icache.miss_count();
//! ```

use crate::stm32::ICACHE;

pub trait ICacheExt {
    fn constrain(self) -> ICache;
}

impl ICacheExt for ICACHE {
    fn constrain(self) -> ICache {
        ICache::new(self)
    }
}

pub struct ICache {
    icache: ICACHE,
}

impl ICache {
    fn new(icache: ICACHE) -> Self {
        Self { icache }
    }

    pub fn is_enabled(&self) -> bool {
        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        self.icache.cr().read().en().bit_is_set()
    }

    fn wait_for_busy_complete(&self) {
        while self.icache.sr().read().busyf().bit_is_set() {}
    }

    /// Enable ICACHE in default operating mode (N-way associative)
    pub fn enable(&mut self) {
        self.enable_n_way()
    }

    /// Enable the ICACHE in N-way associative mode
    pub fn enable_n_way(&mut self) {
        if self.is_enabled() {
            return;
        }

        // Wait for any ongoing cache invalidation operation to complete
        self.wait_for_busy_complete();

        self.icache
            .cr()
            .write(|w| w.waysel().set_bit().en().set_bit());
    }

    /// Enable the ICACHE in 1-way associative mode (direct mapping)
    pub fn enable_direct_mapped(&mut self) {
        if self.is_enabled() {
            return;
        }

        // Wait for any ongoing cache invalidation operation to complete
        self.wait_for_busy_complete();

        self.icache
            .cr()
            .write(|w| w.waysel().clear_bit().en().set_bit());
    }

    /// Disable the ICACHE
    pub fn disable(&mut self) {
        if !self.is_enabled() {
            return;
        }

        // Restore cache mode to default (N-way associative)
        self.icache
            .cr()
            .write(|w| w.waysel().set_bit().en().clear_bit());

        self.invalidate();

        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }

    /// Invalidate the ICACHE. This will block while the cache is being
    /// invalidated
    pub fn invalidate(&mut self) {
        self.icache.cr().modify(|_, w| w.cacheinv().set_bit());

        self.wait_for_busy_complete();

        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }

    /// Enable the cache hit counter. The number of hits can be queried with ICache::hit_counter()
    pub fn enable_hit_counter(&mut self) {
        assert!(
            self.is_enabled(),
            "ICACHE must be enabled before enabling the hit counter"
        );

        // Enable and reset the cache hit counter
        self.icache
            .cr()
            .modify(|_, w| w.hitmrst().set_bit().hitmen().set_bit());
    }

    /// Get the current value of the hit counter
    pub fn hit_count(&self) -> u32 {
        self.icache.hmonr().read().hitmon().bits()
    }

    /// Reset the hit counter
    pub fn reset_hit_counter(&mut self) {
        if !self.is_enabled() {
            return;
        }
        self.icache.cr().modify(|_, w| w.hitmrst().set_bit());
    }

    /// Disable the hit counter. The hit counter is disabled when the peripheral
    /// is disabled
    pub fn disable_hit_counter(&mut self) {
        // Disabling the ICACHE disables the hitmem counter
        if !self.is_enabled() {
            return;
        }
        self.icache.cr().modify(|_, w| w.hitmen().clear_bit());
    }

    /// Enable the cache miss counter
    pub fn enable_miss_counter(&mut self) {
        assert!(
            self.is_enabled(),
            "ICACHE must be enabled before enabling the miss counter"
        );

        // Enable and reset the miss counter
        self.icache
            .cr()
            .modify(|_, w| w.missmrst().set_bit().missmen().set_bit());
    }

    /// Get the current value of the miss counter
    pub fn miss_count(&self) -> u16 {
        self.icache.mmonr().read().missmon().bits()
    }

    /// Reset the miss counter
    pub fn reset_miss_counter(&mut self) {
        if !self.is_enabled() {
            return;
        }
        self.icache.cr().modify(|_, w| w.missmrst().set_bit());
    }

    /// Disable the miss counter. The miss counter is disabled when the ICACHE
    /// is disabled
    pub fn disable_miss_counter(&mut self) {
        // Disabling the ICACHE disables the missmem counter
        if !self.is_enabled() {
            return;
        }
        self.icache.cr().modify(|_, w| w.missmen().clear_bit());
    }

    // Deconstruct and return ICACHE
    pub fn free(self) -> ICACHE {
        self.icache
    }
}
