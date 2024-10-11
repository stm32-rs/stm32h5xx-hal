//! Instruction Cache management
//!
//! To enable the ICACHE in the default N-way set associative mode:
//!
//! ```
//! let dp = ...;            // Device peripherals
//! let mut icache = dp.ICACHE.enable();
//! ```
//!
//! To enable the ICACHE in a specific mode (e.g. Direct-mapped):
//!
//! ```
//! let dp = ...;            // Device peripherals
//! let mut icache = dp.ICACHE.enable_direct_mapped();
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
//!
//! Using interrupts to trigger cache invalidation and determine when it is
//! complete:
//!
//! ```
//! icache.start_invalidation();
//!
//! // In interrupt context
//! if icache.check_event(Event::CacheInvalidationFinished) {
//!     icache.clear_irq(Event::CacheInvalidationFinished)
//! }
//! ```
//!

use crate::stm32::ICACHE;

pub enum Event {
    CacheError,
    CacheInvalidationFinished,
}

pub trait ICacheExt: Sized {
    /// Enable in N-way set associative mode (default)
    fn enable_n_way(self) -> ICache;

    /// Enable in direct mapped mode
    fn enable_direct_mapped(self) -> ICache;

    /// Enable in default mode (N-way set associative)
    fn enable(self) -> ICache {
        self.enable_n_way()
    }
}

impl ICacheExt for ICACHE {
    fn enable_n_way(self) -> ICache {
        ICache::new(self).enable_n_way()
    }

    fn enable_direct_mapped(self) -> ICache {
        ICache::new(self).enable_direct_mapped()
    }
}

pub struct ICache {
    icache: ICACHE,
}

impl ICache {
    fn new(icache: ICACHE) -> Self {
        Self { icache }
    }

    fn wait_for_busy_complete(&self) {
        while self.icache.sr().read().busyf().bit_is_set() {}
    }

    /// Enable the ICACHE in N-way associative mode
    fn enable_n_way(self) -> Self {
        // Wait for any ongoing cache invalidation operation to complete
        self.wait_for_busy_complete();

        self.icache
            .cr()
            .write(|w| w.waysel().set_bit().en().set_bit());

        self
    }

    /// Enable the ICACHE in 1-way associative mode (direct mapping)
    fn enable_direct_mapped(self) -> Self {
        // Wait for any ongoing cache invalidation operation to complete
        self.wait_for_busy_complete();

        self.icache
            .cr()
            .write(|w| w.waysel().clear_bit().en().set_bit());

        self
    }

    /// Disable the ICACHE
    pub fn disable(mut self) -> ICACHE {
        // Restore cache mode to default (N-way associative)
        self.icache
            .cr()
            .write(|w| w.waysel().set_bit().en().clear_bit());

        // Disable interrupts
        self.icache
            .ier()
            .modify(|_, w| w.errie().clear_bit().bsyendie().clear_bit());

        self.invalidate();

        cortex_m::asm::dsb();
        cortex_m::asm::isb();

        self.free()
    }

    /// Invalidate the ICACHE. This will block while the cache is being
    /// invalidated
    pub fn invalidate(&mut self) {
        self.start_cache_invalidation();

        self.wait_for_busy_complete();

        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }

    /// Start cache invalidation
    pub fn start_cache_invalidation(&mut self) {
        self.icache.cr().modify(|_, w| w.cacheinv().set_bit());
    }

    /// Enable interrupts for the given event
    pub fn listen(&mut self, event: Event) {
        self.icache.ier().modify(|_, w| match event {
            Event::CacheError => w.errie().set_bit(),
            Event::CacheInvalidationFinished => w.bsyendie().set_bit(),
        });
    }

    /// Disable interrupts for the given event
    pub fn unlisten(&mut self, event: Event) {
        self.icache.ier().modify(|_, w| match event {
            Event::CacheError => w.errie().clear_bit(),
            Event::CacheInvalidationFinished => w.bsyendie().clear_bit(),
        });
    }

    /// Clear the IRQ for the given event
    pub fn clear_irq(&mut self, event: Event) {
        self.icache.fcr().write(|w| match event {
            Event::CacheError => w.cerrf().set_bit(),
            Event::CacheInvalidationFinished => w.cbsyendf().set_bit(),
        });
    }

    /// Check whether an interrupt event has occurred. Returns true if it has.
    /// Clear the event IRQ by calling `clear_event`
    pub fn check_event(&mut self, event: Event) -> bool {
        let sr = self.icache.sr().read();
        match event {
            Event::CacheError => sr.errf().bit_is_set(),
            Event::CacheInvalidationFinished => sr.bsyendf().bit_is_set(),
        }
    }

    /// Enable the cache hit counter. The number of hits can be queried with ICache::hit_counter()
    pub fn start_hit_counter(&mut self) {
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
        self.icache.cr().modify(|_, w| w.hitmrst().set_bit());
    }

    /// Disable the hit counter. The hit counter is disabled when the peripheral
    /// is disabled
    pub fn stop_hit_counter(&mut self) {
        self.icache.cr().modify(|_, w| w.hitmen().clear_bit());
    }

    /// Enable the cache miss counter
    pub fn start_miss_counter(&mut self) {
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
        self.icache.cr().modify(|_, w| w.missmrst().set_bit());
    }

    /// Disable the miss counter. The miss counter is disabled when the ICACHE
    /// is disabled
    pub fn stop_miss_counter(&mut self) {
        self.icache.cr().modify(|_, w| w.missmen().clear_bit());
    }

    // Deconstruct and return ICACHE
    pub fn free(self) -> ICACHE {
        self.icache
    }
}
