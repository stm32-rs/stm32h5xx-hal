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
