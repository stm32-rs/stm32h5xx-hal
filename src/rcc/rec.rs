//! Peripheral Reset and Enable Control (REC)
//!
//! This module contains safe accessors to the RCC functionality for each
//! peripheral.
//!
//! At a minimum each peripheral implements
//! [ResetEnable](trait.ResetEnable.html). Peripherals that have an
//! individual clock multiplexer in the PKSU also have methods
//! `kernel_clk_mux` and `get_kernel_clk_mux`. These set and get the state
//! of the kernel clock multiplexer respectively.
//!
//! Peripherals that share a clock multiplexer in the PKSU with other
//! peripherals implement a trait with a `get_kernel_clk_mux` method that
//! returns the current kernel clock state. Because the kernel_clk_mux is shared
//! between multiple peripherals, it cannot be set by any individual one of
//! them. Instead it can only be set by methods on the
//! [`PeripheralRec`](struct.PeripheralREC.html) itself. These methods are named
//! `kernel_xxxx_clk_mux()`.
//!
//! # Reset/Enable Example
//!
//! ```
//! // Constrain and Freeze power
//! ...
//! let rcc = dp.RCC.constrain();
//! let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);
//!
//! // Enable the clock to a peripheral and reset it
//! ccdr.peripheral.FDCAN.enable().reset();
//! ```
//!
//! # Individual Kernel Clock Example
//! ```
//! let ccdr = ...; // Returned by `freeze()`, see example above
//!
//! // Set individual kernel clock
//! let cec_prec = ccdr.peripheral.CEC.kernel_clk_mux(CecClkSel::LSI);
//!
//! assert_eq!(cec_prec.get_kernel_clk_mux(), CecClkSel::LSI);
//! ```
//!
//! # Group Kernel Clock Example
//! ```
//! let mut ccdr = ...; // Returned by `freeze()`, see example above
//!
//! // Set group kernel clock mux
//! ccdr.peripheral.kernel_i2c123_clk_mux(I2c123ClkSel::PLL3_R);
//!
//! // Enable and reset peripheral
//! let i2c3_prec = ccdr.peripheral.I2C3.enable().reset();
//!
//! assert_eq!(i2c3_prec.get_kernel_clk_mux(), I2c123ClkSel::PLL3_R);
//!
//! // Some method that consumes the i2c3 prec
//! init_i2c3(..., i2c3_prec);
//!
//! // Can't set group kernel clock (it would also affect I2C3)
//! // ccdr.peripheral.kernel_i2c123_clk_mux(I2c123ClkSel::HSI_KER);
//! ```
//!
//! # REC object
//!
//! There is a REC object for each peripheral. For example:
//!
//! ```
//! let rec_object = ccdr.peripheral.FDCAN;
//! ```
//!
//! If REC object is dropped by user code, then the Reset or Enable state of
//! this peripheral cannot be modified for the lifetime of the program.
#![deny(missing_docs)]

use core::marker::PhantomData;

use super::Rcc;
use crate::stm32::{rcc, RCC};
use cortex_m::interrupt;
use paste;

/// A trait for Resetting, Enabling and Disabling a single peripheral
pub trait ResetEnable {
    /// Enable this peripheral
    #[allow(clippy::return_self_not_must_use)]
    fn enable(self) -> Self;
    /// Disable this peripheral
    #[allow(clippy::return_self_not_must_use)]
    fn disable(self) -> Self;
    /// Reset this peripheral
    #[allow(clippy::return_self_not_must_use)]
    fn reset(self) -> Self;
}

/// The clock gating state of a peripheral in low-power mode
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum LowPowerMode {
    /// Kernel and bus interface clocks are not provided in low-power modes.
    Off,
    /// Kernel and bus interface clocks are provided in CSleep mode.
    #[default]
    Enabled,
}

/// A Token to represent a peripheral has been enabled
pub struct Token<P> {
    ph: PhantomData<P>,
}

/// A trait to represent that a peripheral can be turned into a token
pub trait RecTokenizer: ResetEnable + Sized {
    /// Turns the ResetEnable into a token
    fn tokenize(self) -> Token<Self> {
        self.enable().reset();

        Token { ph: PhantomData {} }
    }
}

impl Rcc {
    /// Returns all the peripherals resets / enables / kernel clocks.
    ///
    /// # Use case
    ///
    /// Allows peripherals to be reset / enabled before the calling
    /// freeze. For example, the internal watchdog could be enabled to
    /// issue a reset if the call to freeze hangs waiting for an external
    /// clock that is stopped.
    ///
    /// # Safety
    ///
    /// If this method is called multiple times, or is called before the
    /// [freeze](struct.Rcc.html#freeze), then multiple accesses to the
    /// same memory exist.
    #[inline]
    pub unsafe fn steal_peripheral_rec(&self) -> PeripheralREC {
        PeripheralREC::new_singleton()
    }
}

// This macro uses the paste::item! macro to create identifiers.
//
// https://crates.io/crates/paste
macro_rules! peripheral_reset_and_enable_control {
    ($( #[ $tmeta:meta ] $AXBn:ident, $axb_doc:expr => [
        $(
            $( #[ $pmeta:meta ] )*
                $(($NoReset:ident))? $p:ident
                $([ kernel $clk:ident: $pk:ident $(($Variant:ident))* $pk_alias:ident $ccip:ident $clk_doc:expr ])*
                $([ group clk: $pk_g:ident $( $(($Variant_g:ident))* $pk_g_alias:ident $ccip_g:ident $clk_doc_g:expr )* ])*
                $([ fixed clk: $clk_doc_f:expr ])*
        ),*
    ];)+) => {
        paste::item! {
            /// Peripheral Reset and Enable Control
            #[allow(non_snake_case)]
            #[non_exhaustive]
            pub struct PeripheralREC {
                $(
                    $(
                        #[allow(missing_docs)]
                        #[ $tmeta ]
                        $( #[ $pmeta ] )*
                        pub [< $p:upper >]: $p,
                    )*
                )+
            }
            impl PeripheralREC {
                /// Return a new instance of the peripheral resets /
                /// enables / kernel clocks
                ///
                /// # Safety
                ///
                /// If this method is called multiple times, then multiple
                /// accesses to the same memory exist.
                pub(super) unsafe fn new_singleton() -> PeripheralREC {
                    PeripheralREC {
                        $(
                            $(
                                #[ $tmeta ]
                                $( #[ $pmeta ] )*
                                [< $p:upper >]: $p {
                                    _marker: PhantomData,
                                },
                            )*
                        )+
                    }
                }
            }
            $(
                $(
                    #[ $tmeta ]
                    peripheral_reset_and_enable_control_generator! (
                        $AXBn, $(($NoReset))* $p, [< $p:upper >], [< $p:lower >],
                        $( $pmeta )*
                        $(
                            [kernel $clk: $pk $(($Variant))* $pk_alias $ccip $clk_doc]
                        )*
                        $(
                            [group clk: $pk_g [< $pk_g:lower >] $( $(($Variant_g))* $pk_g_alias $ccip_g $clk_doc_g )* ]
                        )*
                        $(
                            [fixed clk: $clk_doc_f]
                        )*
                    );
                )*
            )+
        }
    }
}

macro_rules! peripheral_reset_function_behavior {
    (
        $AXBn:ident,
        $p:ident
    ) => {
        paste::item! {
            // unsafe: Owned exclusive access to this bitfield
            interrupt::free(|_| {
                let rstr = unsafe {
                    &(*RCC::ptr()).[< $AXBn:lower rstr >]()
                };
                rstr.modify(|_, w| w.
                            [< $p:lower rst >]().set_bit());
                rstr.modify(|_, w| w.
                            [< $p:lower rst >]().clear_bit());
            });
        }
    };
    (
        $AXBn:ident,
        $NoReset:ident $p:ident
    ) => {};
}

// This macro uses the paste::item! macro to create identifiers.
//
// https://crates.io/crates/paste
//
// The macro is intended only to be called from within the
// peripheral_reset_and_enable_control macro
macro_rules! peripheral_reset_and_enable_control_generator {
    (
        $AXBn:ident,
        $(($NoReset:ident))? $p:ident,
        $p_upper:ident,         // Lower and upper case $p available for use in
        $p_lower:ident,         // comments, equivalent to with the paste macro.

        $( $pmeta:meta )*
        $([ kernel $clk:ident: $pk:ident $(($Variant:ident))* $pk_alias:ident $ccip:ident $clk_doc:expr ])*
        $([ group clk: $pk_g:ident $pk_g_lower:ident $( $(($Variant_g:ident))* $pk_g_alias:ident $ccip_g:ident $clk_doc_g:expr )* ])*
        $([ fixed clk: $clk_doc_f:expr ])*
    ) => {
        paste::item! {
            #[doc = " Reset, Enable and Clock functionality for " $p]
            ///
            /// # Reset/Enable Example
            ///
            /// ```
            /// let ccdr = ...; // From RCC
            ///
            /// // Enable the clock to the peripheral and reset it
            #[doc = "ccdr.peripheral." $p_upper ".enable().reset();"]
            /// ```
            ///
            $(                  // Individual kernel clocks
                /// # Individual Kernel Clock
                ///
                /// This peripheral has its own dedicated kernel clock.
                #[doc = "See [" $pk "ClkSel](crate::rcc::rec::" $pk "ClkSel) "
                  "for possible clock sources."]
                ///
                /// ```
                /// let ccdr = ...; // From RCC
                ///
                /// // Set individual kernel clock
                #[doc = "let " $p_lower "_prec = ccdr.peripheral." $p_upper
                  ".kernel_clk_mux(" $pk "ClkSel::XX_clock_soruce_XX);"]
                ///
                #[doc = "assert_eq!(" $p_lower "_prec.get_kernel_clk_mux(), "
                  $pk "ClkSel::XX_clock_source_XX);"]
                /// ```
            )*
            $(                  // Group kernel clocks
                /// # Group Kernel Clock
                ///
                /// This peripheral has a kernel clock that is shared with other
                /// peripherals.
                ///
                #[doc = "Since it is shared, it must be set using the "
                  "[kernel_" $pk_g_lower "_clk_mux](crate::rcc::rec::PeripheralREC#method"
                  ".kernel_" $pk_g_lower "_clk_mux) method."]
                ///
                /// ```
                /// let mut ccdr = ...; // From RCC
                ///
                /// // Set group kernel clock mux
                #[doc = " ccdr.peripheral."
                  "kernel_" $pk_g_lower "_clk_mux("
                  $pk_g "ClkSel::XX_clock_source_XX);"]
                ///
                #[doc = " assert_eq!(ccdr.peripheral." $p_upper
                  ".get_kernel_clk_mux(), " $pk_g "ClkSel::XX_clock_source_XX);"]
            )*
            $(                  // Fixed kernel clocks
                /// # Fixed Kernel Clock
                ///
                /// This peripheral has a kernel clock that is always equal to
                #[doc= $clk_doc_f "."]
            )*
            $( #[ $pmeta ] )*
            pub struct $p {
                pub(crate) _marker: PhantomData<*const ()>,
            }
            $( #[ $pmeta ] )*
            impl $p {
                /// Set Low Power Mode for peripheral
                #[allow(clippy::return_self_not_must_use)]
                pub fn low_power(self, lpm: LowPowerMode) -> Self {
                    // unsafe: Owned exclusive access to this bitfield
                    interrupt::free(|_| {
                        // LPEN
                        let lpenr = unsafe {
                            &(*RCC::ptr()).[< $AXBn:lower lpenr >]()
                        };
                        lpenr.modify(|_, w| w.[< $p:lower lpen >]()
                                     .bit(lpm != LowPowerMode::Off));
                    });
                    self
                }
            }
            $( #[ $pmeta ] )*
            unsafe impl Send for $p {}
            $( #[ $pmeta ] )*
            impl ResetEnable for $p {
                #[inline(always)]
                fn enable(self) -> Self {
                    // unsafe: Owned exclusive access to this bitfield
                    interrupt::free(|_| {
                        let enr = unsafe {
                            &(*RCC::ptr()).[< $AXBn:lower enr >]()
                        };
                        enr.modify(|_, w| w.
                                   [< $p:lower en >]().set_bit());
                    });
                    self
                }
                #[inline(always)]
                fn disable(self) -> Self {
                    // unsafe: Owned exclusive access to this bitfield
                    interrupt::free(|_| {
                        let enr = unsafe {
                            &(*RCC::ptr()).[< $AXBn:lower enr >]()
                        };
                        enr.modify(|_, w| w.
                                   [< $p:lower en >]().clear_bit());
                    });
                    self
                }
                #[inline(always)]
                fn reset(self) -> Self {
                    peripheral_reset_function_behavior!($AXBn, $($NoReset)? $p);
                    self
                }
            }
            $( #[ $pmeta ] )*
            impl $p {
                $(      // Individual kernel clocks
                    #[inline(always)]
                    #[allow(clippy::return_self_not_must_use)]
                    /// Modify the kernel clock for
                    #[doc=$clk_doc "."]
                    /// See RM0433 Rev 7 Section 8.5.8.
                    ///
                    /// It is possible to switch this clock dynamically without
                    /// generating spurs or timing violations. However, the user
                    /// must ensure that both clocks are running. See RM0433 Rev
                    /// 7 Section 8.5.10.
                    pub fn [< kernel_ $clk _mux >](self, sel: [< $pk ClkSel >]) -> Self {
                        // unsafe: Owned exclusive access to this bitfield
                        interrupt::free(|_| {
                            let ccip = unsafe {
                                &(*RCC::ptr()).[< $ccip >]()
                            };
                            ccip.modify(|_, w| w.
                                        [< $pk:lower sel >]().variant(sel));
                        });
                        self
                    }

                    #[inline(always)]
                    /// Return the current kernel clock selection
                    pub fn [< get_kernel_ $clk _mux>](&self) ->
                        variant_return_type!([< $pk ClkSel >] $(, $Variant)*)
                    {
                        // unsafe: We only read from this bitfield
                        let ccip = unsafe {
                            &(*RCC::ptr()).[< $ccip >]()
                        };
                        ccip.read().[< $pk:lower sel >]().variant()
                    }
                )*
            }
            $(          // Individual kernel clocks
                #[doc=$clk_doc]
                /// kernel clock source selection
                pub type [< $pk ClkSel >] =
                    rcc::[< $ccip >]::[< $pk_alias SEL >];
            )*
            $(          // Group kernel clocks
                impl [< $pk_g ClkSelGetter >] for $p {}
            )*
            $(          // Group kernel clocks
                $(
                    #[doc=$clk_doc_g]
                    /// kernel clock source selection.
                    pub type [< $pk_g ClkSel >] =
                        rcc::[< $ccip_g >]::[< $pk_g_alias SEL >];

                    /// Can return
                    #[doc=$clk_doc_g]
                    /// kernel clock source selection
                    pub trait [< $pk_g ClkSelGetter >] {
                        #[inline(always)]
                        #[allow(unused)]
                        /// Return the
                        #[doc=$clk_doc_g]
                        /// kernel clock selection
                        fn get_kernel_clk_mux(&self) ->
                            variant_return_type!([< $pk_g ClkSel >] $(, $Variant_g)*)
                        {
                            // unsafe: We only read from this bitfield
                            let ccip = unsafe {
                                &(*RCC::ptr()).[< $ccip_g >]()
                            };
                            ccip.read().[< $pk_g:lower sel >]().variant()
                        }
                    }
                )*
            )*
            impl PeripheralREC {
                $(          // Group kernel clocks
                    $(
                        /// Modify the kernel clock for
                        #[doc=$clk_doc_g "."]
                        /// See RM0492 Revision 2 Section 10.4.12.
                        ///
                        /// It is possible to switch this clock dynamically
                        /// without generating spurs or timing
                        /// violations. However, the user must ensure that both
                        /// clocks are running. See RM0433 Rev 7 Section 8.5.10.
                        pub fn [< kernel_ $pk_g:lower _clk_mux >](&mut self, sel: [< $pk_g ClkSel >]) -> &mut Self {
                            // unsafe: Owned exclusive access to this bitfield
                            interrupt::free(|_| {
                                let ccip = unsafe {
                                    &(*RCC::ptr()).[< $ccip_g >]()
                                };
                                ccip.modify(|_, w| w.
                                            [< $pk_g:lower sel >]().variant(sel));
                            });
                            self
                        }
                    )*
                )*
            }
        }
    }
}

// If the PAC does not fully specify a CCIP field (perhaps because one or
// more values are reserved), then we use a different return type
macro_rules! variant_return_type {
    ($t:ty) => { $t };
    ($t:ty, $Variant: ident) => {
        Option<$t>
    };
}

macro_rules! tokenable {
    ($($TK:ty),*) => {
        $(
            impl RecTokenizer for $TK {}
        )*
    };
}

// Enumerate all peripherals and optional clock multiplexers
//
// Peripherals are grouped by bus for convenience. Each bus is specified like:
// #[attribute] name, "description" => [..];
//
// The attribute is mandatory for the bus grouping, but can just be
// #[cfg(all())]. The description is not used. Each bus grouping can be repeated
// multiple times if needed.
//
// As well as busses, peripherals can optionally be preceeded by a conditional
// compilation attribute. However, this only works for peripherals without
// kernel clock multiplexers.
//
// Peripherals with an individual kernel clock must be marked "kernel clk". If a
// kernel clock multiplexer is shared between multiple peripherals, all those
// peripherals must instead be marked with a common "group clk".
peripheral_reset_and_enable_control! {

    #[cfg(all())]
    AHB1, "AMBA High-performance Bus (AHB1) peripherals" => [
        (NoReset) Sram1,
        (NoReset) BkpRam,
        RamCfg,
        Crc,
        (NoReset) Flitf,
        Gpdma1,
        Gpdma2
    ];
    #[cfg(feature = "rm0492")]
    AHB1, "" => [
        (NoReset) Gtzc1
    ];
    #[cfg(feature = "rm0481")]
    AHB1, "" => [
        (NoReset) Dcache,
        (NoReset) Tzsc1,
        Fmac,
        Cordic
    ];
    #[cfg(feature = "ethernet")]
    AHB1, "" => [
        (NoReset) Ethrx,
        (NoReset) Ethtx,
        Eth
    ];

    #[cfg(all())]
    AHB2, "AMBA High-performance Bus (AHB2) peripherals" => [
        (NoReset) Sram2,
        Hash,
        Gpioh,
        Gpiod,
        Gpioc,
        Gpiob,
        Gpioa,
        Rng [kernel clk: Rng RNG ccipr5 "RNG"],
        Adc [group clk: AdcDac(Variant) ADCDAC ccipr5 "ADC/DAC"]
    ];
    #[cfg(feature = "rm0492")]
    AHB2, "" => [
        Dac12 [group clk: AdcDac]
    ];
    #[cfg(feature = "rm0481")]
    AHB2, "" => [
        Gpiog,
        Gpiof,
        Gpioe,
        Dac1 [group clk: AdcDac]
    ];
    #[cfg(feature = "h56x_h573")]
    AHB2, "" => [
        Gpioi
    ];

    #[cfg(feature = "rm0481")]
    AHB4, "AMBA High-performance Bus (AHB4) peripherals" => [
        Octospi1  [kernel clk: Octospi1 OCTOSPI1 ccipr4 "OCTOSPI1"],
        Fmc,

        Sdmmc1 [kernel clk: Sdmmc1 SDMMC ccipr4 "SDMMC1"]
    ];
    #[cfg(feature = "sdmmc2")]
    AHB4, "" => [
        Sdmmc2 [kernel clk: Sdmmc2 SDMMC ccipr4 "SDMMC2"]
    ];
    #[cfg(feature = "otfdec")]
    AHB4, "" => [
        Otfdec1
    ];

    #[cfg(all())]
    APB1L, "Advanced Peripheral Bus 1L (APB1L) peripherals" => [
        Usart2 [kernel clk: Usart2(Variant) USART ccipr1 "USART2"],
        Usart3 [kernel clk: Usart3(Variant) USART ccipr1 "USART3"],
        Crs,
        I3c1 [kernel clk: I3c1(Variant) I3C ccipr4 "I3C1"],
        I2c2 [kernel clk: I2c2 I2C ccipr4 "I2C2"],
        I2c1 [kernel clk: I2c1 I2C ccipr4 "I2C1"],
        Spi3 [kernel clk: Spi3(Variant) SPI123 ccipr3 "SPI3"],
        Spi2 [kernel clk: Spi2(Variant) SPI123 ccipr3 "SPI2"],
        (NoReset) Wwdg,
        Tim2, Tim3, Tim6, Tim7
    ];
    #[cfg(feature = "rm0492")]
    APB1L, "" => [
        Opamp,
        Comp
    ];
    #[cfg(feature = "rm0481")]
    APB1L, "" => [
        Cec [kernel clk: Cec(Variant) CEC ccipr5 "CEC"],
        Usart6 [kernel clk: Usart6(Variant) USART ccipr1 "USART6"],
        Uart5 [kernel clk: Uart5(Variant) USART ccipr1 "USART5"],
        Uart4 [kernel clk: Uart4(Variant) USART ccipr1 "USART4"],
        Tim12, Tim5, Tim4
    ];
    #[cfg(feature = "h56x_h573")]
    APB1L, "" => [
        Uart8 [kernel clk: Uart8(Variant) USART ccipr1 "UART8"],
        Uart7 [kernel clk: Uart7(Variant) USART ccipr1 "UART7"],
        Usart11 [kernel clk: Usart11(Variant) USART ccipr2 "USART11"],
        Usart10 [kernel clk: Usart10(Variant) USART ccipr1 "USART10"],
        Tim14, Tim13
    ];

    #[cfg(all())]
    APB1H, "Advanced Peripheral Bus 1H (APB1H) peripherals" => [
        Fdcan [kernel clk: Fdcan(Variant) FDCAN ccipr5 "FDCAN"],
        Lptim2 [kernel clk: Lptim2(Variant) LPTIM ccipr2 "LPTIM2"],
        Dts
    ];
    #[cfg(feature = "rm0481")]
    APB1H, "" => [
        Ucpd1
    ];
    #[cfg(feature = "h56x_h573")]
    APB1H, "" => [
        Uart12 [kernel clk: Uart12(Variant) USART ccipr2 "USART12"],
        Uart9 [kernel clk: Uart9(Variant) USART ccipr1 "UART9"]
    ];

    #[cfg(all())]
    APB2, "Advanced Peripheral Bus 2 (APB2) peripherals" => [
        Usb [kernel clk: Usb USB ccipr4 "USB"],
        Usart1 [kernel clk: Usart1(Variant) USART ccipr1 "USART1"],
        Spi1 [kernel clk: Spi1(Variant) SPI123 ccipr3 "SPI1"],
        Tim1
    ];
    #[cfg(feature = "rm0481")]
    APB2, "" => [
        Sai2 [kernel clk: Sai2(Variant) SAI ccipr5 "SAI2"],
        Sai1 [kernel clk: Sai1(Variant) SAI ccipr5 "SAI1"],
        Spi4 [kernel clk: Spi4(Variant) SPI456 ccipr3 "SPI4"],
        Tim15, Tim8
    ];
    #[cfg(feature = "h56x_h573")]
    APB2, "" => [
        Spi6 [kernel clk: Spi6(Variant) SPI456 ccipr3 "SPI6"],
        Tim17, Tim16
    ];

    #[cfg(all())]
    APB3, "Advanced Peripheral Bus 3 (APB3) peripherals" => [
        (NoReset) RtcApb,
        LpTim1 [kernel clk: LpTim1(Variant) LPTIM ccipr2 "LPTIM1"],
        LpUart1 [kernel clk: LpUart1(Variant) USART ccipr3 "LPUART1"],
        (NoReset) Sbs
    ];
    #[cfg(feature = "rm0481")]
    APB3, "" => [
        Vref,
        I2c3 [kernel clk: I2c3 I2C ccipr4 "I2C3"]
    ];
    #[cfg(feature = "h56x_h573")]
    APB3, "" => [
        LpTim6 [kernel clk: LpTim6(Variant) LPTIM ccipr2 "LPTIM6"],
        LpTim5 [kernel clk: LpTim5(Variant) LPTIM ccipr2 "LPTIM5"],
        LpTim4 [kernel clk: LpTim4(Variant) LPTIM ccipr2 "LPTIM4"],
        LpTim3 [kernel clk: LpTim3(Variant) LPTIM ccipr2 "LPTIM3"],
        I2c4 [kernel clk: I2c4 I2C ccipr4 "I2C4"],
        Spi5 [kernel clk: Spi5(Variant) SPI456 ccipr3 "SPI5"]
    ];
    #[cfg(any(feature = "rm0492", feature = "h523_h533"))]
    APB3, "" => [
        I3c2 [kernel clk: I3c2(Variant) I3C ccipr4 "I3C2"]
    ];
}

tokenable!(Fdcan);
