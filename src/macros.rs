/// This macro is used to insert a double read of a peripheral register to give the  peripheral
/// enough time to process a write to a register to clear an interrupt. This prevents an interrupt
/// from firing immediately a second time after an ISR exits. It's necessary due to delayed
/// synchronization between a peripheral and the CPU due to the different clocks of the
/// peripheral and CPU. The register that is passed in should not have produce undesireable side
/// effects when read.
///
/// See ARM Application Note 321 Section 4.9
/// Also see discussion in the stm32h7xx-hal PRs [`#191`][191] and [`#195`][195].
///
/// [191]: https://github.com/stm32-rs/stm32h7xx-hal/pull/191
/// [195]: https://github.com/stm32-rs/stm32h7xx-hal/pull/195
macro_rules! interrupt_clear_clock_sync_delay {
    ($status_reg:expr) => {
        let _ = $status_reg.read();
        let _ = $status_reg.read();
    };
}
