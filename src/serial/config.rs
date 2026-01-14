use crate::time::Hertz;

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoThreshold {
    /// Only valid for Tx Threshold
    Empty,
    Eighth,
    Quarter,
    Half,
    ThreeQuarter,
    SevenEighth,
    /// Only valid for Rx Threshold
    Full,
}

/// The parity bits appended to each serial data word
///
/// When enabled parity bits will be automatically added by hardware on transmit, and automatically checked by
/// hardware on receive. For example, `read()` would return [`Error::Parity`](super::Error::Parity).
///
/// Note that parity bits are included in the serial word length, so if parity is used word length will be set to 9.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[doc = "1 stop bit"]
    Stop1,
    #[doc = "0.5 stop bits"]
    Stop0p5,
    #[doc = "2 stop bits"]
    Stop2,
    #[doc = "1.5 stop bits"]
    Stop1p5,
}
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitOrder {
    LsbFirst,
    MsbFirst,
}
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockPhase {
    First,
    Second,
}
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockPolarity {
    IdleHigh,
    IdleLow,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WordSize {
    DataBits7,
    DataBits8,
    DataBits9,
}

/// A structure for specifying the USART configuration
///
/// This structure uses the builder pattern to generate the configuration:
///
/// ```
/// let config = Config::new().partity_odd();
/// ```
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub baudrate: Hertz,
    pub parity: Parity,
    pub stop_bits: StopBits,
    pub bit_order: BitOrder,
    pub clock_phase: ClockPhase,
    pub clock_polarity: ClockPolarity,
    pub last_bit_clock_pulse: bool,
    pub swap_txrx: bool,
    pub invert_rx: bool,
    pub invert_tx: bool,
    pub rx_fifo_threshold: FifoThreshold,
    pub tx_fifo_threshold: FifoThreshold,
    pub half_duplex: bool,
    pub word_size: WordSize,
}

impl Config {
    /// Create a default configuration for the USART or UART interface
    ///
    /// * 8 bits, 1 stop bit, no parity (8N1)
    /// * LSB first
    pub fn new(frequency: Hertz) -> Self {
        Config {
            baudrate: frequency,
            parity: Parity::ParityNone,
            stop_bits: StopBits::Stop1,
            bit_order: BitOrder::LsbFirst,
            clock_phase: ClockPhase::First,
            clock_polarity: ClockPolarity::IdleLow,
            last_bit_clock_pulse: false,
            swap_txrx: false,
            invert_rx: false,
            invert_tx: false,
            rx_fifo_threshold: FifoThreshold::Eighth,
            tx_fifo_threshold: FifoThreshold::SevenEighth,
            half_duplex: false,
            word_size: WordSize::DataBits8,
        }
    }

    pub fn baudrate(mut self, baudrate: Hertz) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    /// Enables Even Parity
    ///
    /// Note that parity bits are included in the serial word length, so if parity is used word length will be set
    /// to 9.
    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    /// Enables Odd Parity
    ///
    /// Note that parity bits are included in the serial word length, so if parity is used word length will be set
    /// to 9.
    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    /// Specify the number of stop bits
    pub fn stop_bits(mut self, stopbits: StopBits) -> Self {
        self.stop_bits = stopbits;
        self
    }
    /// Specify the bit order
    pub fn bit_order(mut self, bitorder: BitOrder) -> Self {
        self.bit_order = bitorder;
        self
    }
    /// Specify the clock phase. Only applies to USART peripherals
    pub fn clock_phase(mut self, clockphase: ClockPhase) -> Self {
        self.clock_phase = clockphase;
        self
    }
    /// Specify the clock polarity. Only applies to USART peripherals
    pub fn clock_polarity(mut self, clockpolarity: ClockPolarity) -> Self {
        self.clock_polarity = clockpolarity;
        self
    }
    /// Specify if the last bit transmitted in each word has a corresponding
    /// clock pulse in the SCLK pin. Only applies to USART peripherals
    pub fn last_bit_clock_pulse(mut self, lastbitclockpulse: bool) -> Self {
        self.last_bit_clock_pulse = lastbitclockpulse;
        self
    }

    /// If `true`, swap the Tx and Rx pins
    pub fn swap_txrx(mut self, swaptxrx: bool) -> Self {
        self.swap_txrx = swaptxrx;
        self
    }

    /// If `true`, RX pin signal levels are inverted
    pub fn invert_rx(mut self, invertrx: bool) -> Self {
        self.invert_rx = invertrx;
        self
    }

    /// If `true`, TX pin signal levels are inverted
    pub fn invert_tx(mut self, inverttx: bool) -> Self {
        self.invert_tx = inverttx;
        self
    }

    pub fn rx_fifo_threshold(mut self, rxfifothreshold: FifoThreshold) -> Self {
        self.rx_fifo_threshold = rxfifothreshold;
        self
    }

    pub fn tx_fifo_threshold(mut self, txfifothreshold: FifoThreshold) -> Self {
        self.tx_fifo_threshold = txfifothreshold;
        self
    }

    /// If `true`, sets to half-duplex mode
    pub fn half_duplex(mut self, halfduplex: bool) -> Self {
        self.half_duplex = halfduplex;
        self
    }

    pub fn data_width(mut self, word_size: WordSize) -> Self {
        self.word_size = word_size;
        self
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidConfig;

impl Default for Config {
    fn default() -> Config {
        Self::new(Hertz::from_raw(115_200)) // 115k2 baud
    }
}

impl From<Hertz> for Config {
    fn from(frequency: Hertz) -> Config {
        Self::new(frequency)
    }
}
