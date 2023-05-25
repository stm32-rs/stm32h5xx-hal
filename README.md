# [Documentation](https://docs.rs/stm32h5xx-hal)

# stm32h5xx-hal

[![docs.rs](https://docs.rs/stm32h5xx-hal/badge.svg)](https://docs.rs/stm32h5xx-hal)
<!-- [![Bors enabled](https://bors.tech/images/badge_small.svg)](https://app.bors.tech/repositories/12691) -->
[![CI](https://github.com/stm32-rs/stm32h5xx-hal/workflows/Continuous%20integration/badge.svg)](https://github.com/stm32-rs/stm32h5xx-hal/actions)
[![Crates.io](https://img.shields.io/crates/v/stm32h5xx-hal.svg)](https://crates.io/crates/stm32h5xx-hal)
![Minimum rustc version](https://img.shields.io/badge/rustc-1.69.0+-yellow.svg)

[_stm32h5xx-hal_](https://github.com/stm32-rs/stm32h5xx-hal) contains
a hardware abstraction layer on top of the peripheral access API for
the STMicro STM32H5xx family of microcontrollers. The idea behind this
crate is to gloss over the slight differences in the various
peripherals available on those MCUs so a HAL can be written for all
chips in that same family without having to cut and paste crates for
every single model.

### Supported Configurations

| Part      | Supported | RM | Dev board |
| --------- | --------- | -- | --------- |
| stm32h503 | 🚧 (WIP)  | [RM0492](https://www.st.com/resource/en/reference_manual/rm0492-stm32h503-line-armbased-32bit-mcus-stmicroelectronics.pdf) | [Nucleo H503RB](https://www.st.com/en/evaluation-tools/nucleo-h503rb.html) |
| stm32h562 | ❌*       | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) |  |
| stm32h563 | ❌*       | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) | [Nucleo H563ZI](https://www.st.com/en/evaluation-tools/nucleo-h563zi.html) |
| stm32h573 | ❌*       | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) |  |

\* Support planned

### Minimum supported Rust version

The Minimum Supported Rust Version (MSRV) at the moment is **1.69.0**. Older
versions **may** compile, especially when some features are not used in your
application.

### Changelog

See [CHANGELOG.md](CHANGELOG.md).


### License

0-Clause BSD License, see [LICENSE-0BSD.txt](LICENSE-0BSD.txt) for more details.

[stm32-rs](https://github.com/stm32-rs)
[stm32h5](https://crates.io/crates/stm32h5)
[cortex-m](https://crates.io/crates/cortex-m)
[embedded-hal](https://github.com/rust-embedded/embedded-hal)
