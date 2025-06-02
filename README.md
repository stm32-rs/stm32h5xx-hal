# [Documentation](https://docs.rs/stm32h5xx-hal)

# stm32h5xx-hal

[![docs.rs](https://docs.rs/stm32h5xx-hal/badge.svg)](https://docs.rs/stm32h5xx-hal)
[![CI](https://github.com/stm32-rs/stm32h5xx-hal/workflows/Continuous%20integration/badge.svg)](https://github.com/stm32-rs/stm32h5xx-hal/actions)
[![Crates.io](https://img.shields.io/crates/v/stm32h5xx-hal.svg)](https://crates.io/crates/stm32h5xx-hal)
![Minimum rustc version](https://img.shields.io/badge/rustc-1.78.0+-yellow.svg)

[_stm32h5xx-hal_](https://github.com/stm32-rs/stm32h5xx-hal) contains
a hardware abstraction layer on top of the peripheral access API for
the STMicro STM32H5xx family of microcontrollers. The idea behind this
crate is to gloss over the slight differences in the various
peripherals available on those MCUs so a HAL can be written for all
chips in that same family without having to cut and paste crates for
every single model.

## Supported MCUs

The following STM32H5xx MCUs are supported by the HAL:

| Part      | RM | Dev board |
| --------- | -- | --------- |
| stm32h503 | [RM0492](https://www.st.com/resource/en/reference_manual/rm0492-stm32h503-line-armbased-32bit-mcus-stmicroelectronics.pdf) <sup>[(errata)](https://www.st.com/resource/en/errata_sheet/es0561-stm32h503cbebkbrb-device-errata-stmicroelectronics.pdf)</sup> | [Nucleo H503RB](https://www.st.com/en/evaluation-tools/nucleo-h503rb.html) |
| stm32h523 | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) <sup>[(errata)](https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf)</sup>| |
| stm32h533 | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) <sup>[(errata)](https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf)</sup>| [Nucleo H533RE](https://www.st.com/en/evaluation-tools/nucleo-h533re.html) |
| stm32h562 | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) <sup>[(errata)](https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf)</sup>|  |
| stm32h563 | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) <sup>[(errata)](https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf)</sup> | [Nucleo H563ZI](https://www.st.com/en/evaluation-tools/nucleo-h563zi.html) |
| stm32h573 | [RM0481](https://www.st.com/resource/en/reference_manual/rm0481-stm32h563h573-and-stm32h562-armbased-32bit-mcus-stmicroelectronics.pdf) <sup>[(errata)](https://www.st.com/resource/en/errata_sheet/es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf)</sup> |  |

## Peripheral Support
For all listed MCUs above, core clock & power configuration and startup is implemented. The status
of support for peripherals is shown in the table below.

| Peripheral/Feature | Supported? | Issue | Notes |
|------------|----|---|---|
| GPIO       | ✅ | - | |
| ICache     | ✅ | - | |
| I2C        | ✅ | - | Controller operation is done; Target is 🚧 |
| CAN        | 🚧 | - | |
| Rng        | 🚧 | [#34](https://github.com/stm32-rs/stm32h5xx-hal/issues/34)| |
| SPI        | 🚧 | [#36](https://github.com/stm32-rs/stm32h5xx-hal/issues/36) | |
| UART       | 🚧 | - | |
| DMA        | 🚧 | - | |
| ADC        | ❌ | [#35](https://github.com/stm32-rs/stm32h5xx-hal/issues/35) | |
| Timers     | ❌ | - | |
| PWM        | ❌ | - | |
| Rtc        | ❌ | - | |
| Flash      | ❌ | - | |

## Minimum supported Rust version

The Minimum Supported Rust Version (MSRV) at the moment is **1.78.0**. Older
versions **may** compile, especially when some features are not used in your
application.

## Getting Started

The [examples folder](examples/) contains several example programs. To compile
them, specify the target device in a cargo feature:

```
$ cargo build --features=stm32h523,rt --example <example>
```

If you are unfamiliar with embedded development using Rust, there are
a number of fantastic resources available to help.

- [Embedded Rust Documentation](https://docs.rust-embedded.org/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [Rust Embedded FAQ](https://docs.rust-embedded.org/faq.html)
- [rust-embedded/awesome-embedded-rust](https://github.com/rust-embedded/awesome-embedded-rust)

## Changelog

See [CHANGELOG.md](CHANGELOG.md). Note: this will be populated once the first crates have been
published to crates.io.

## License

0-Clause BSD License, see [LICENSE-0BSD.txt](LICENSE-0BSD.txt) for more details.

[stm32-rs](https://github.com/stm32-rs)
[stm32h5](https://crates.io/crates/stm32h5)
[cortex-m](https://crates.io/crates/cortex-m)
[embedded-hal](https://github.com/rust-embedded/embedded-hal)
