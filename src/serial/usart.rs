//! USART implementation for Serial
//!
//! This provides an implementation of the Serial functionality via the USART peripheral. See the
//! documentation for the `serial` module for more information.

use super::config::WordSize;
use super::*;

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::ptr;

use embedded_io as io;
#[cfg(feature = "log")]
use log::debug;

use crate::rcc::{CoreClocks, ResetEnable};
use crate::stm32::{
    usart1,
    usart1::cr1::{M0, M1, PCE, PS},
    usart1::cr3::{RXFTCFG, TXFTCFG},
    usart1::presc::PRESCALER,
};
use crate::time::Hertz;

mod usart_def;

pub trait Pins<USART> {
    const SYNCHRONOUS: bool = false;
}
pub trait PinTx<USART> {}
pub trait PinRx<USART> {}
pub trait PinCk<USART> {}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
{
}

impl<USART, TX, RX, CK> Pins<USART> for (TX, RX, CK)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
    CK: PinCk<USART>,
{
    const SYNCHRONOUS: bool = true;
}

/// A filler type for when the Tx pin is unnecessary
pub struct NoTx;
/// A filler type for when the Rx pin is unnecessary
pub struct NoRx;
/// A filler type for when the Ck pin is unnecessary
pub struct NoCk;

pub trait InstanceClock {
    fn clock(clocks: &CoreClocks) -> Hertz;
}

pub trait Instance:
    InstanceClock + crate::Sealed + Deref<Target = usart1::RegisterBlock>
{
    type Rec: ResetEnable;

    #[doc(hidden)]
    fn ptr() -> *const usart1::RegisterBlock;

    #[doc(hidden)]
    fn rec() -> Self::Rec;
}

/// Serial abstraction
pub struct Serial<USART, W: WordBits = u8> {
    inner: Inner<USART, W>,
}

pub struct Inner<USART, W: WordBits = u8> {
    usart: USART,
    _word: PhantomData<W>,
}

impl<USART, W: WordBits> Deref for Serial<USART, W> {
    type Target = Inner<USART, W>;

    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<USART, W: WordBits> DerefMut for Serial<USART, W> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

pub trait SerialExt<USART: Instance, W: WordBits = u8>: Sized {
    fn serial<P: Pins<USART>>(
        self,
        _pins: P,
        config: impl Into<config::Config>,
        rec: USART::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<USART, W>, config::InvalidConfig>;

    fn serial_unchecked(
        self,
        config: impl Into<config::Config>,
        rec: USART::Rec,
        clocks: &CoreClocks,
        synchronous: bool,
    ) -> Result<Serial<USART, W>, config::InvalidConfig>;
}

impl<USART: Instance, W: WordBits> SerialExt<USART, W> for USART {
    fn serial<P: Pins<USART>>(
        self,
        _pins: P,
        config: impl Into<config::Config>,
        rec: USART::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<USART, W>, config::InvalidConfig> {
        Serial::new(self, config, rec, clocks, P::SYNCHRONOUS)
    }

    fn serial_unchecked(
        self,
        config: impl Into<config::Config>,
        rec: <USART as Instance>::Rec,
        clocks: &CoreClocks,
        synchronous: bool,
    ) -> Result<Serial<USART, W>, config::InvalidConfig> {
        Serial::new(self, config, rec, clocks, synchronous)
    }
}

impl<USART: Instance, W: WordBits> Serial<USART, W> {
    fn new(
        usart: USART,
        config: impl Into<config::Config>,
        rec: USART::Rec,
        clocks: &CoreClocks,
        synchronous: bool,
    ) -> Result<Self, config::InvalidConfig> {
        // Enable clock for USART and reset
        rec.enable().reset();

        let mut serial = Serial {
            inner: Inner::new(usart),
        };
        let config = config.into();
        serial.usart.cr1().reset();

        // Ensure that word size in config matches the word size type, W
        if config.word_size == WordSize::DataBits9 && W::BITS < 9 {
            return Err(config::InvalidConfig);
        }
        if config.word_size != WordSize::DataBits9 && W::BITS > 8 {
            return Err(config::InvalidConfig);
        }

        // If synchronous mode is supported, check that it is not
        // enabled alongside half duplex mode
        if config.half_duplex & synchronous {
            return Err(config::InvalidConfig);
        }

        let ker_ck = USART::clock(clocks);
        serial.configure(&config, ker_ck, synchronous)?;

        Ok(serial)
    }

    /// Runs the serial port configuration process
    ///
    /// The serial port must be disabled when called.
    fn configure(
        &mut self,
        config: &config::Config,
        ker_ck: Hertz,
        synchronous: bool,
    ) -> Result<(), config::InvalidConfig> {
        use self::config::*;

        // If the baudrate is low enough that BRR would be greater than 65535, use a prescalar to
        // divide down the kernel clock frequency by a power of 2.
        let mut div = ker_ck / config.baudrate;
        div >>= u16::BITS;
        let (div, presc) = match div {
            0 => (1, PRESCALER::Div1),
            1 => (2, PRESCALER::Div2),
            2..=3 => (4, PRESCALER::Div4),
            4..=5 => (6, PRESCALER::Div6),
            6..=7 => (8, PRESCALER::Div8),
            8..=9 => (10, PRESCALER::Div10),
            10..=11 => (12, PRESCALER::Div12),
            12..=15 => (16, PRESCALER::Div16),
            16..=31 => (32, PRESCALER::Div32),
            32..=63 => (64, PRESCALER::Div64),
            64..=127 => (128, PRESCALER::Div128),
            _ => (256, PRESCALER::Div256),
        };

        self.usart.presc().write(|w| w.prescaler().variant(presc));

        let usart_ker_ck_presc = ker_ck / div;

        // The frequency to calculate USARTDIV is this:
        //
        // (See RM0492 Rev 2 Section 36.5.8)
        //
        // 16 bit oversample: OVER8 = 0
        // 8 bit oversample:  OVER8 = 1
        //
        // USARTDIV =        (ker_ck)
        //            ------------------------
        //            8 x (2 - OVER8) x (baud)
        //
        // BUT, the USARTDIV has 4 "fractional" bits, which effectively
        // means that we need to "correct" the equation as follows:
        //
        // USARTDIV =      (ker_ck) * 16
        //            ------------------------
        //            8 x (2 - OVER8) x (baud)
        //
        // Calculate correct baudrate divisor on the fly
        let (over8, usartdiv) = if (usart_ker_ck_presc / 16) >= config.baudrate
        {
            // We have the ability to oversample to 16 bits, take
            // advantage of it.

            let div =
                (usart_ker_ck_presc + (config.baudrate / 2)) / config.baudrate;
            (false, div)
        } else if (usart_ker_ck_presc / 8) >= config.baudrate {
            // We are close enough to pclk where we can only
            // oversample 8.
            let div = ((usart_ker_ck_presc * 2) + (config.baudrate / 2))
                / config.baudrate;

            // Shift USARTDIV[3:0] right by 1 when oversampling by 8
            let frac = div & 0xF;
            let div = (div & !0xF) | (frac >> 1);
            (true, div)
        } else {
            return Err(config::InvalidConfig);
        };

        #[cfg(feature = "log")]
        {
            let baudrate = usart_ker_ck_presc / usartdiv;
            debug!("USART: Kernel clock: {ker_ck}; Prescalar: {div}; Over8: {over8}; BRR: {usartdiv:#X}; Baudrate: {baudrate}");
        }

        // Calculate baudrate divisor

        // 16 times oversampling, OVER8 = 0
        let brr = usartdiv as u16;
        self.usart.brr().write(|w| w.brr().set(brr));

        // Reset registers to disable advanced USART features
        self.usart.cr2().reset();
        self.usart.cr3().reset();

        // RXFIFO threshold
        let fifo_threshold = match config.rx_fifo_threshold {
            FifoThreshold::Eighth => RXFTCFG::Depth1_8,
            FifoThreshold::Quarter => RXFTCFG::Depth1_4,
            FifoThreshold::Half => RXFTCFG::Depth1_2,
            FifoThreshold::ThreeQuarter => RXFTCFG::Depth3_4,
            FifoThreshold::SevenEighth => RXFTCFG::Depth7_8,
            FifoThreshold::Full => RXFTCFG::Full,
            // Empty is not a valid configuration
            _ => return Err(config::InvalidConfig),
        };
        self.usart
            .cr3()
            .modify(|_, w| w.rxftcfg().variant(fifo_threshold));

        // TXFIFO threashold
        let fifo_threshold = match config.tx_fifo_threshold {
            FifoThreshold::Empty => TXFTCFG::Empty,
            FifoThreshold::Eighth => TXFTCFG::Depth1_8,
            FifoThreshold::Quarter => TXFTCFG::Depth1_4,
            FifoThreshold::Half => TXFTCFG::Depth1_2,
            FifoThreshold::ThreeQuarter => TXFTCFG::Depth3_4,
            FifoThreshold::SevenEighth => TXFTCFG::Depth7_8,
            // Full is not a valid configuration
            _ => return Err(config::InvalidConfig),
        };
        self.usart
            .cr3()
            .modify(|_, w| w.txftcfg().variant(fifo_threshold));

        // Configure half-duplex mode
        self.usart.cr3().modify(|_, w| {
            if config.half_duplex {
                w.hdsel().selected()
            } else {
                w.hdsel().not_selected()
            }
        });

        // Configure serial mode
        self.usart.cr2().write(|w| {
            match config.stop_bits {
                StopBits::Stop0p5 => w.stop().stop0p5(),
                StopBits::Stop1 => w.stop().stop1(),
                StopBits::Stop1p5 => w.stop().stop1p5(),
                StopBits::Stop2 => w.stop().stop2(),
            };

            match config.bit_order {
                BitOrder::LsbFirst => w.msbfirst().lsb(),
                BitOrder::MsbFirst => w.msbfirst().msb(),
            };

            if config.swap_txrx {
                w.swap().swapped()
            } else {
                w.swap().standard()
            };

            if config.invert_rx {
                w.rxinv().inverted()
            } else {
                w.rxinv().standard()
            };

            if config.invert_tx {
                w.txinv().inverted()
            } else {
                w.txinv().standard()
            };

            if synchronous {
                w.clken().enabled();

                if config.last_bit_clock_pulse {
                    w.lbcl().output()
                } else {
                    w.lbcl().not_output()
                };

                match config.clock_polarity {
                    ClockPolarity::IdleHigh => w.cpol().high(),
                    ClockPolarity::IdleLow => w.cpol().low(),
                };

                match config.clock_phase {
                    ClockPhase::First => w.cpha().first(),
                    ClockPhase::Second => w.cpha().second(),
                };
            } else {
                w.clken().disabled();
            }

            w
        });

        // Enable transmission and receiving and configure frame
        // Retain enabled events
        self.usart.cr1().modify(|_, w| {
            w.fifoen()
                .enabled() // FIFO mode enabled
                .over8()
                .bit(over8)
                .ue()
                .enabled()
                .te()
                .enabled()
                .re()
                .enabled()
                .m1()
                .variant(match config.word_size {
                    WordSize::DataBits7 => M1::Bit7,
                    _ => M1::M0,
                })
                .m0()
                .variant(match config.word_size {
                    WordSize::DataBits9 => M0::Bit9,
                    _ => M0::Bit8,
                })
                .pce()
                .variant(match config.parity {
                    Parity::ParityNone => PCE::Disabled,
                    _ => PCE::Enabled,
                })
                .ps()
                .variant(match config.parity {
                    Parity::ParityOdd => PS::Odd,
                    _ => PS::Even,
                })
        });

        Ok(())
    }

    /// Releases the USART peripheral
    pub fn free(self) -> USART {
        // Wait until both TXFIFO and shift register are empty
        while self.usart.isr().read().tc().bit_is_clear() {}

        self.inner.usart
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &USART {
        &self.usart
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut USART {
        &mut self.usart
    }
}

impl<USART, W: WordBits> Inner<USART, W> {
    fn new(usart: USART) -> Self {
        Inner {
            usart,
            _word: PhantomData,
        }
    }
}

macro_rules! check_status_error {
    ($isr:expr) => {
        if $isr.pe().bit_is_set() {
            Err(Error::Parity)
        } else if $isr.fe().bit_is_set() {
            Err(Error::Framing)
        } else if $isr.ne().bit_is_set() {
            Err(Error::Noise)
        } else if $isr.ore().bit_is_set() {
            Err(Error::Overrun)
        } else {
            Ok(())
        }
    };
}

impl<USART: Instance, W: WordBits> Inner<USART, W> {
    /// Starts listening for an interrupt event
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::RxNotEmpty => {
                self.usart.cr1().modify(|_, w| w.rxneie().enabled());
            }
            Event::TxNotFull => {
                self.usart.cr1().modify(|_, w| w.txeie().enabled());
            }
            Event::Idle => {
                self.usart.cr1().modify(|_, w| w.idleie().enabled());
            }
            Event::TxFifoThreshold => {
                self.usart.cr3().modify(|_, w| w.txftie().set_bit());
            }
            Event::RxFifoThreshold => {
                self.usart.cr3().modify(|_, w| w.rxftie().set_bit());
            }
        }
    }

    /// Stop listening for an interrupt event
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::RxNotEmpty => {
                self.usart.cr1().modify(|_, w| w.rxneie().disabled());
            }
            Event::TxNotFull => {
                self.usart.cr1().modify(|_, w| w.txeie().disabled());
            }
            Event::Idle => {
                self.usart.cr1().modify(|_, w| w.idleie().disabled());
            }
            Event::TxFifoThreshold => {
                self.usart.cr3().modify(|_, w| w.txftie().clear_bit());
            }
            Event::RxFifoThreshold => {
                self.usart.cr3().modify(|_, w| w.rxftie().clear_bit());
            }
        }
        let _ = self.usart.cr1().read();
        let _ = self.usart.cr1().read(); // Delay 2 peripheral clocks
    }

    /// Return true if the line idle status is set
    ///
    /// The line idle status bit is set when the peripheral detects the receive line is idle.
    /// The bit is cleared by software, by calling `clear_idle()`.
    pub fn is_idle(&self) -> bool {
        self.usart.isr().read().idle().bit_is_set()
    }

    /// Clear the line idle status bit
    pub fn clear_idle(&mut self) {
        self.usart.icr().write(|w| w.idlecf().clear());
        let _ = self.usart.isr().read();
        let _ = self.usart.isr().read(); // Delay 2 peripheral clocks
    }

    /// Return true if the line busy status is set
    ///
    /// The busy status bit is set when there is communication active on the receive line,
    /// and reset at the end of reception.
    pub fn is_busy(&self) -> bool {
        self.usart.isr().read().busy().bit_is_set()
    }

    /// Return true if the tx register is empty (and can accept data)
    fn is_tx_empty(&self) -> bool {
        self.usart.isr().read().txfe().is_empty()
    }

    /// Return true if the rx register is not empty (and can be read)
    fn is_data_ready(&mut self) -> Result<bool, Error> {
        let isr = self.usart.isr().read();

        match check_status_error!(isr) {
            Ok(()) => {}
            Err(error) => {
                self.clear_error_flag(error);
                return Err(error);
            }
        };

        Ok(isr.rxfne().is_data_ready())
    }

    fn read_data(&mut self) -> W {
        // NOTE(read_volatile) see `write_volatile` below
        unsafe { ptr::read_volatile(self.usart.rdr() as *const _ as *const W) }
    }

    fn write_data(&mut self, word: W) {
        // NOTE(unsafe) atomic write to stateless register
        // NOTE(write_volatile) 8- or 16-bit write that's not possible through the svd2rust API
        unsafe {
            let tdr = self.usart.tdr() as *const _ as *const UnsafeCell<W>;
            ptr::write_volatile(UnsafeCell::raw_get(tdr), word);
        }
    }

    fn clear_error_flag(&mut self, error: Error) {
        match error {
            Error::Framing => self.usart.icr().write(|w| w.fecf().clear()),
            Error::Noise => self.usart.icr().write(|w| w.necf().clear()),
            Error::Overrun => self.usart.icr().write(|w| w.orecf().clear()),
            Error::Parity => self.usart.icr().write(|w| w.pecf().clear()),
        };
    }

    fn read_if_ready(&mut self, word: &mut W) -> Result<bool, Error> {
        if self.is_data_ready()? {
            *word = self.read_data();
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn write_if_ready(&mut self, word: W) -> Result<bool, Error> {
        if self.is_tx_empty() {
            self.write_data(word);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn is_transmit_complete(&mut self) -> bool {
        self.usart.isr().read().tc().bit_is_set()
    }

    fn read_words(&mut self, words: &mut [W]) -> Result<usize, Error> {
        for (i, w) in words.iter_mut().enumerate() {
            if !self.read_if_ready(w)? {
                return Ok(i);
            }
        }
        Ok(words.len())
    }

    fn write_words(&mut self, words: &[W]) -> Result<usize, Error> {
        for (i, w) in words.iter().enumerate() {
            if !self.write_if_ready(*w)? {
                return Ok(i);
            }
        }
        Ok(words.len())
    }
}

/*
 *  HAL Implementations
 */

impl<USART, W: WordBits> io::ErrorType for Serial<USART, W> {
    type Error = Error;
}

impl<USART: Instance> io::Read for Serial<USART, u8> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let count = loop {
            let count = self.read_words(buf)?;
            if count > 0 {
                break count;
            }
        };
        Ok(count)
    }
}

impl<USART: Instance> io::ReadReady for Serial<USART, u8> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        self.is_data_ready()
    }
}

impl<USART: Instance> io::Write for Serial<USART, u8> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let count = loop {
            let count = self.write_words(buf)?;
            if count > 0 {
                break count;
            }
        };
        Ok(count)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.is_transmit_complete() {}
        Ok(())
    }
}

impl<USART: Instance> io::WriteReady for Serial<USART, u8> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_tx_empty())
    }
}
