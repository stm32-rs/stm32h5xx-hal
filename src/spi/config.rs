use crate::stm32::spi1::cfg2::COMM;

use super::{Mode, Polarity};

/// Specifies the communication mode of the SPI interface.
#[derive(Copy, Clone, PartialEq)]
pub enum CommunicationMode {
    /// Both RX and TX are used on separate wires. This is the default communications mode
    FullDuplex,

    /// RX and TX are used on a single wire.
    HalfDuplex,

    /// Only the SPI TX functionality is used on a single wire.
    SimplexTransmitter,

    /// Only the SPI RX functionality is used on a single wire.
    SimplexReceiver,
}

impl From<COMM> for CommunicationMode {
    fn from(value: COMM) -> Self {
        match value {
            COMM::Transmitter => CommunicationMode::SimplexTransmitter,
            COMM::Receiver => CommunicationMode::SimplexReceiver,
            COMM::FullDuplex => CommunicationMode::FullDuplex,
            COMM::HalfDuplex => CommunicationMode::HalfDuplex,
        }
    }
}

impl From<CommunicationMode> for COMM {
    fn from(value: CommunicationMode) -> Self {
        match value {
            CommunicationMode::SimplexTransmitter => COMM::Transmitter,
            CommunicationMode::SimplexReceiver => COMM::Receiver,
            CommunicationMode::FullDuplex => COMM::FullDuplex,
            CommunicationMode::HalfDuplex => COMM::HalfDuplex,
        }
    }
}

/// The endianness with which to send the data
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endianness {
    /// Least significant bit first
    LsbFirst,
    /// Most significant bit first. MSB first is the default SPI behavior.
    MsbFirst,
}

/// A structure for specifying SPI configuration.
///
/// This structure uses builder semantics to generate the configuration.
///
/// `Example`
/// ```
/// use stm32h5xx::spi::Mode;
///
/// let config = Config::new(Mode::MODE_0)
///     .manage_cs()
/// ```
#[derive(Copy, Clone)]
pub struct Config {
    pub(super) mode: Mode,
    pub(super) swap_miso_mosi: bool,
    pub(super) hardware_cs: HardwareCS,
    pub(super) inter_word_delay: f32,
    pub(super) communication_mode: CommunicationMode,
    pub(super) endianness: Endianness,
}

impl Config {
    /// Create a default configuration for the SPI interface.
    ///
    /// Arguments:
    /// * `mode` - The SPI mode to configure.
    pub fn new(mode: Mode) -> Self {
        Config {
            mode,
            swap_miso_mosi: false,
            hardware_cs: HardwareCS {
                mode: HardwareCSMode::Disabled,
                assertion_delay: 0.0,
                polarity: Polarity::IdleHigh,
            },
            inter_word_delay: 0.0,
            communication_mode: CommunicationMode::FullDuplex,
            endianness: Endianness::MsbFirst,
        }
    }

    /// Specify that the SPI MISO/MOSI lines are swapped.
    ///
    /// Note:
    /// * This function updates the HAL peripheral to treat the pin provided in the MISO parameter
    ///   as the MOSI pin and the pin provided in the MOSI parameter as the MISO pin.
    #[must_use]
    pub fn swap_mosi_miso(mut self) -> Self {
        self.swap_miso_mosi = true;
        self
    }

    /// Specify the behaviour of the hardware chip select.
    ///
    /// This also affects the way data is sent using [HardwareCSMode].
    /// By default the hardware cs is disabled.
    #[must_use]
    pub fn hardware_cs<CS>(mut self, hardware_cs: CS) -> Self
    where
        CS: Into<HardwareCS>,
    {
        self.hardware_cs = hardware_cs.into();
        self
    }

    /// Specify the time in seconds that should be idled between every data word being sent.
    ///
    /// Note:
    /// * This value is converted to a number of spi peripheral clock ticks and at most 15 of those.
    #[must_use]
    pub fn inter_word_delay(mut self, inter_word_delay: f32) -> Self {
        self.inter_word_delay = inter_word_delay;
        self
    }

    /// Select the communication mode of the SPI bus.
    #[must_use]
    pub fn communication_mode(mut self, mode: CommunicationMode) -> Self {
        self.communication_mode = mode;
        self
    }

    // /// Select endianness is used for data transfers
    pub fn endianness(mut self, endianness: Endianness) -> Self {
        self.endianness = endianness;
        self
    }
}

impl From<Mode> for Config {
    fn from(mode: Mode) -> Self {
        Self::new(mode)
    }
}

/// Object containing the settings for the hardware chip select pin
#[derive(Clone, Copy)]
pub struct HardwareCS {
    /// The value that determines the behaviour of the hardware chip select pin.
    pub mode: HardwareCSMode,
    /// The delay between CS assertion and the beginning of the SPI transaction in seconds.
    ///
    /// Note:
    /// * This value introduces a delay on SCK from the initiation of the transaction. The delay
    ///   is specified as a number of SCK cycles, so the actual delay may vary.
    pub assertion_delay: f32,
    /// The polarity of the CS pin.
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HardwareCSMode {
    /// Handling the CS is left for the user to do in software
    Disabled,
    /// The CS will assert and de-assert for each word being sent
    WordTransaction,
    /// The CS will assert and only de-assert after the whole frame is sent.
    ///
    /// When this mode is active, the blocking embedded hal interface automatically
    /// sets up the frames so it will be one frame per call.
    ///
    /// Note:
    /// * If using the non-blocking APIs, this mode does require some maintenance. Before sending,
    ///   you must setup the frame with [Spi::setup_transaction]. After everything has been sent,
    ///   you must also clean it up with [Spi::end_transaction].
    FrameTransaction,
}

impl HardwareCS {
    pub fn new(mode: HardwareCSMode) -> Self {
        HardwareCS {
            mode,
            assertion_delay: 0.0,
            polarity: Polarity::IdleHigh,
        }
    }

    pub fn assertion_delay(mut self, delay: f32) -> Self {
        self.assertion_delay = delay;
        self
    }

    pub fn polarity(mut self, polarity: Polarity) -> Self {
        self.polarity = polarity;
        self
    }

    pub(super) fn enabled(&self) -> bool {
        !matches!(self.mode, HardwareCSMode::Disabled)
    }

    pub(super) fn interleaved_cs(&self) -> bool {
        matches!(self.mode, HardwareCSMode::WordTransaction)
    }
}

impl From<HardwareCSMode> for HardwareCS {
    fn from(value: HardwareCSMode) -> Self {
        HardwareCS::new(value)
    }
}
