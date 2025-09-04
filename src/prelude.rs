//! Prelude
pub use embedded_hal_02::prelude::*;

pub use crate::delay::DelayExt as _stm32h5xx_hal_delay_DelayExt;
pub use crate::dwt::DwtExt as _stm32h5xx_hal_delay_DwtExt;
pub use crate::gpdma::GpdmaExt as _stm32h5xx_hal_gpdma_GpdmaExt;
pub use crate::gpio::GpioExt as _stm32h5xx_hal_gpio_GpioExt;
pub use crate::i2c::I2cExt as _stm32h5xx_hal_i2c_I2cExt;
pub use crate::icache::ICacheExt as _stm32h5xx_hal_icache_ICacheExt;
pub use crate::pwr::PwrExt as _stm32h5xx_hal_pwr_PwrExt;
pub use crate::rcc::RccExt as _stm32h5xx_hal_rcc_RccExt;
pub use crate::spi::SpiExt as _stm32h5xx_hal_spi_SpiExt;
pub use crate::timer::TimerExt as _stm32h5xx_hal_timer_TimerExt;
pub use crate::usb::UsbExt as _stm32h5xx_hal_usb_UsbExt;

pub use crate::time::U32Ext as _;
pub use fugit::{ExtU32 as _, RateExtU32 as _};
