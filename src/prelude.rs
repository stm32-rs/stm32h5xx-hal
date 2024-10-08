//! Prelude

pub use crate::gpio::GpioExt as _stm32h5xx_hal_gpio_GpioExt;
pub use crate::icache::ICacheExt as _stm32h5xx_hal_delay_ICacheExt;
pub use crate::pwr::PwrExt as _stm32h5xx_hal_pwr_PwrExt;
pub use crate::rcc::RccExt as _stm32h5xx_hal_rcc_RccExt;

pub use fugit::{ExtU32 as _, RateExtU32 as _};
