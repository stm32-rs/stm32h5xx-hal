//! The transform module provides a configuration builder to set up the data transformations
//! supported by the GPDMA peripheral.
//!
//! # Usage
//!    use stm32h5xx_hal::gpdma::DmaConfig;
//!    use stm32h5xx_hal::gpdma::config::transform::*;  // This ensures relevant traits are in scope
//!
//!    let config: DmaConfig<u32,u16> = DmaConfig::new().with_data_transform(
//!        DataTransform::builder()
//!            .swap_source_middle_bytes()
//!            .right_align_left_truncate()
//!            .swap_destination_half_word_byte_order()
//!    );
use core::marker::PhantomData;

use super::Word;

/// Represents the options available for the padding and alignment step in the data transformation
/// pipeline
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum PaddingAlignmentMode {
    #[default]
    None,

    // PAM1 - Source data width < Destination data width
    ZeroPadded,
    SignExtended,
    Packed,

    // PAM2 - Source data width > Destination data width
    RightAlignedLeftTruncated,
    LeftAlignedRightTruncated,
    Unpacked,
}

impl PaddingAlignmentMode {
    pub fn bits(&self) -> u8 {
        match self {
            PaddingAlignmentMode::None => {
                panic!("Do not set PAM bits if no PAM mode was chosen")
            }
            PaddingAlignmentMode::ZeroPadded => 0,
            PaddingAlignmentMode::SignExtended => 1,
            PaddingAlignmentMode::Packed => 2,
            PaddingAlignmentMode::RightAlignedLeftTruncated => 0,
            PaddingAlignmentMode::LeftAlignedRightTruncated => 1,
            PaddingAlignmentMode::Unpacked => 2,
        }
    }
}

pub trait SourceByteExchange {
    fn swap_source_middle_bytes(self) -> Self;
}

pub trait PaddingAlignment {
    fn right_align_zero_pad(self) -> Self;
    fn right_align_sign_extend(self) -> Self;
    fn pack(self) -> Self;
}

pub trait TruncationAlignment {
    fn right_align_left_truncate(self) -> Self;
    fn left_align_right_truncate(self) -> Self;
    fn unpack(self) -> Self;
}

pub trait DestinationHalfWordExchange {
    fn swap_destination_half_words(self) -> Self;
}

pub trait DestinationByteExchange {
    fn swap_destination_half_word_byte_order(self) -> Self;
}

/// The DataTransformBuilder is used to configure the data transformation pipeline that the GPDMA
/// peripheral implements.
///
/// Depending upon what word sizes are used for transfers, different pipeline steps are applicable:
///
/// - The first possible step in the pipeline, the source byte exchange step is applicable to 32-bit
///   sources only and swaps the middle 2 bytes of the 32-bit word
/// - The next step is applicable when the source data width is not equal to the destination data
///   width:
///   - If the destination width is less than the source width, the data can be truncated (left or
///     right aligned) or unpacked into a FIFO to output all the data to subsequent destination
///     words (destination buffer size must be large enough to accomodate the size in bytes of the
///     unpacked source data)
///   - If the destination width is greater than the source width, the data can be zero- or
///     sign-extended, or it can be packed into the destination words.
/// - After the padding/alignment step, the order of the destination 16-bit half-words in a 32-bit
///   destination word can be swapped (only applicable if the destination word is 32-bit)
/// - Finally, the order of the bytes in each 16-bit destination (half-) word can be swapped (only
///   applicable for 32- and 16-bit destination word sizes)
///
/// This builder allows each step to be specified, only when relevant to the source and destination
/// data-widths.
///
/// To get a builder use [`DataTransform::builder()`]. Type inference is used to determine the
/// source and destination word sizes, so the builder can be created without specifying the types
/// explicitly.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataTransformBuilder<S, D> {
    _source_type: PhantomData<S>,
    _destination_type: PhantomData<D>,
    pub(super) transform: DataTransform,
}

impl<S: Word, D: Word> DataTransformBuilder<S, D> {
    pub fn new() -> Self {
        Self::default()
    }
}

impl<D: Word> SourceByteExchange for DataTransformBuilder<u32, D> {
    /// The order of the unaligned middle bytes of a 32-bit source word is exchanged
    /// ie. B3B2B1B0 -> B3B1B2B0
    fn swap_source_middle_bytes(mut self) -> Self {
        self.transform.source_byte_exchange = true;
        self
    }
}

impl PaddingAlignment for DataTransformBuilder<u16, u32> {
    /// Pad out the upper 16 bits of the 32-bit destination word with zeroes (default)
    fn right_align_zero_pad(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::ZeroPadded;
        self
    }

    /// Sign extend the upper 16 bits of the 32-bit destination word
    fn right_align_sign_extend(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::SignExtended;
        self
    }

    /// Pack subsequent 16-bit words into the 32-bit destination words
    /// ie: B3B2,B1B0 -> B3B2B1B0 (see RM0492, Table 92)
    fn pack(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::Packed;
        self
    }
}

impl PaddingAlignment for DataTransformBuilder<u8, u32> {
    /// Pad out the upper 24 bits of the 32-bit destination word with zeroes (default)
    fn right_align_zero_pad(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::ZeroPadded;
        self
    }

    /// Sign extend the upper 24 bits of the 32-bit destination word
    fn right_align_sign_extend(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::SignExtended;
        self
    }

    /// Pack subsequent 8-bit words into the 32-bit destination words
    /// ie: B3,B2,B1,B0 -> B3B2B1B0 (see RM0492, Table 92)
    fn pack(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::Packed;
        self
    }
}

impl PaddingAlignment for DataTransformBuilder<u8, u16> {
    /// Pad out the upper 8 bits of the 16-bit destination word with zeroes (default)
    fn right_align_zero_pad(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::ZeroPadded;
        self
    }

    /// Sign extend the upper 8 bits of the 32-bit destination word
    fn right_align_sign_extend(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::SignExtended;
        self
    }

    /// Pack subsequent 8-bit words into the 16-bit destination words
    /// ie: B1,B0 -> B1B0 (see RM0492, Table 92)
    fn pack(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::Packed;
        self
    }
}

impl TruncationAlignment for DataTransformBuilder<u32, u16> {
    /// Keep the least significant 16-bits and truncate the rest (default)
    ///
    /// ie: B7B6B5B4,B3B2B1B0 -> B5B4,B1B0 (see RM0492, Table 92)
    fn right_align_left_truncate(mut self) -> Self {
        self.transform.padding_alignment =
            PaddingAlignmentMode::RightAlignedLeftTruncated;
        self
    }

    /// Keep the most significant 16-bits and truncate the rest
    ///
    /// ie: B7B6B5B4,B3B2B1B0 -> B7B6,B3B2 (see RM0492, Table 92)
    fn left_align_right_truncate(mut self) -> Self {
        self.transform.padding_alignment =
            PaddingAlignmentMode::LeftAlignedRightTruncated;
        self
    }

    /// Unpack each 32-bit word into separate 16-bit half-words.
    /// Note that the destination buffer must have sufficient room for n*2 16-bit values where n is
    /// the number of 32-bit words in the source buffer.
    ///
    /// ie: B7B6B5B4,B3B2B1B0 -> B7B6,B5B4,B3B2,B1B0 (see RM0492, Table 92)
    fn unpack(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::Unpacked;
        self
    }
}

impl TruncationAlignment for DataTransformBuilder<u32, u8> {
    /// Keep the least significant 8-bits and truncate the rest (default)
    ///
    /// ie: B7B6B5B4,B3B2B1B0 -> B4,B0
    fn right_align_left_truncate(mut self) -> Self {
        self.transform.padding_alignment =
            PaddingAlignmentMode::RightAlignedLeftTruncated;
        self
    }

    /// Keep the most significant 8-bits and truncate the rest
    ///
    /// i.e: B7B6B5B4,B3B2B1B0 -> B7,B3
    fn left_align_right_truncate(mut self) -> Self {
        self.transform.padding_alignment =
            PaddingAlignmentMode::LeftAlignedRightTruncated;
        self
    }

    /// Unpack each word or half-word into separate 8-bit bytes.
    /// Note that the destination buffer must have sufficient room for n*2 8-bit values where n is
    /// the number of word or half-words in the source buffer.
    ///
    /// ie: B7B6B5B4,B3B2B1B0 -> B7,B6,B5,B4,B3,B2,B1,B0
    fn unpack(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::Unpacked;
        self
    }
}

impl TruncationAlignment for DataTransformBuilder<u16, u8> {
    /// Keep the least significant 8 bits and truncate the rest (default)
    ///
    /// ie: B3B2,B1B0 -> B2,B0 (see RM0492, Table 92)
    fn right_align_left_truncate(mut self) -> Self {
        self.transform.padding_alignment =
            PaddingAlignmentMode::RightAlignedLeftTruncated;
        self
    }

    /// Keep the most significant 8 bits and truncate the rest
    ///
    /// ie: B3B2,B1B0 -> B3,B1 (see RM0492, Table 92)
    fn left_align_right_truncate(mut self) -> Self {
        self.transform.padding_alignment =
            PaddingAlignmentMode::LeftAlignedRightTruncated;
        self
    }

    /// Unpack each 16-bit word into separate 8-bit half-words.
    /// Note that the destination buffer must have sufficient room for n*2 16-bit values where n is
    /// the number of 32-bit words in the source buffer.
    ///
    /// ie: B3B2,B1B0 -> B3,B2,B1,B0 (see RM0492, Table 92)
    fn unpack(mut self) -> Self {
        self.transform.padding_alignment = PaddingAlignmentMode::Unpacked;
        self
    }
}

impl<S: Word> DestinationHalfWordExchange for DataTransformBuilder<S, u32> {
    /// Swap the order of the 16-bit half-words in the 32-bit destination word
    fn swap_destination_half_words(mut self) -> Self {
        self.transform.dest_half_word_exchange = true;
        self
    }
}

impl<S: Word> DestinationByteExchange for DataTransformBuilder<S, u16> {
    /// Swap the order of bytes in each 16-bit destination word
    fn swap_destination_half_word_byte_order(mut self) -> Self {
        self.transform.dest_byte_exchange = true;
        self
    }
}

impl<S: Word> DestinationByteExchange for DataTransformBuilder<S, u32> {
    /// Swap the order of bytes in each 16-bit destination half-word
    fn swap_destination_half_word_byte_order(mut self) -> Self {
        self.transform.dest_byte_exchange = true;
        self
    }
}

/// DataTransform represents the configuration of the data transformation pipeline as produced
/// by the above builder structs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataTransform {
    pub(crate) source_byte_exchange: bool,
    pub(crate) padding_alignment: PaddingAlignmentMode,
    pub(crate) dest_half_word_exchange: bool,
    pub(crate) dest_byte_exchange: bool,
}

impl DataTransform {
    pub fn builder<S: Word, D: Word>() -> DataTransformBuilder<S, D> {
        DataTransformBuilder {
            _source_type: PhantomData,
            _destination_type: PhantomData,
            transform: DataTransform::default(),
        }
    }
}
