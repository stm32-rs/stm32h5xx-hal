#![cfg_attr(not(test), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![allow(non_camel_case_types)]

#[cfg(not(feature = "device-selected"))]
compile_error!(
    "This crate requires one of the following device features enabled:
        stm32h503
        stm32h562
        stm32h563
        stm32h573
"
);

#[cfg(all(feature = "rm0492", feature = "rm0481"))]
compile_error!("Cannot not select both rm0492 and rm0481");

#[cfg(feature = "stm32h503")]
pub use stm32h5::stm32h503 as stm32;

#[cfg(feature = "stm32h562")]
pub use stm32h5::stm32h562 as stm32;

#[cfg(feature = "stm32h563")]
pub use stm32h5::stm32h563 as stm32;

#[cfg(feature = "stm32h573")]
pub use stm32h5::stm32h573 as stm32;

#[cfg(feature = "device-selected")]
pub use crate::stm32 as pac;
#[cfg(feature = "device-selected")]
pub use crate::stm32 as device;

// Enable use of interrupt macro
#[cfg(feature = "rt")]
#[cfg_attr(docsrs, doc(cfg(feature = "rt")))]
pub use crate::stm32::interrupt;

#[cfg(feature = "device-selected")]
pub mod prelude;

#[cfg(feature = "device-selected")]
pub mod pwr;
