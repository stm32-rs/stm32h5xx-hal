use core::num::NonZeroUsize;

use embedded_hal::spi::{
    Error as HalError, ErrorKind, ErrorType, Operation, SpiBus, SpiDevice,
};
use embedded_hal_nb::spi::FullDuplex;

use super::{Error, FrameSize, Instance, NonBlockingTransfer, Spi};

impl HalError for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::Overrun => ErrorKind::Overrun,
            Error::Underrun => ErrorKind::Other,
            Error::ModeFault => ErrorKind::ModeFault,
            Error::Crc => ErrorKind::Other,
            Error::TransferAlreadyComplete => ErrorKind::Other,
            Error::TransactionAlreadyStarted => ErrorKind::Other,
            Error::BufferTooBig { max_size: _ } => ErrorKind::Other,
            Error::InvalidOperation => ErrorKind::Other,
        }
    }
}

impl<SPI, W: FrameSize> ErrorType for Spi<SPI, W> {
    type Error = Error;
}

impl<SPI: Instance, W: FrameSize> FullDuplex<W> for Spi<SPI, W> {
    fn read(&mut self) -> nb::Result<W, Error> {
        self.check_read()
    }

    fn write(&mut self, word: W) -> nb::Result<(), Error> {
        self.check_write(word)
    }
}

impl<SPI: Instance, W: FrameSize> SpiBus<W> for Spi<SPI, W> {
    #[inline]
    fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.read(words)
    }

    #[inline]
    fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.write(words)
    }

    #[inline]
    fn transfer(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Self::Error> {
        self.transfer(read, write)
    }

    #[inline]
    fn transfer_in_place(
        &mut self,
        words: &mut [W],
    ) -> Result<(), Self::Error> {
        self.transfer_inplace(words)
    }

    #[inline]
    fn flush(&mut self) -> Result<(), Self::Error> {
        // todo!();
        Ok(())
    }
}

trait OperationExt {
    fn len(&self) -> usize;
}

impl<W: FrameSize> OperationExt for Operation<'_, W> {
    fn len(&self) -> usize {
        match self {
            Operation::Read(words) => words.len(),
            Operation::Write(words) => words.len(),
            Operation::Transfer(read, write) => {
                core::cmp::max(read.len(), write.len())
            }
            Operation::TransferInPlace(words) => words.len(),
            Operation::DelayNs(_) => 0,
        }
    }
}

impl<SPI: Instance, W: FrameSize> SpiDevice<W> for Spi<SPI, W> {
    fn transaction(
        &mut self,
        operations: &mut [Operation<'_, W>],
    ) -> Result<(), Self::Error> {
        let len = operations.iter().fold(0, |acc, op| acc + op.len());
        if len == 0 {
            return Ok(());
        }
        self.setup_transaction(NonZeroUsize::new(len).unwrap())?;

        for operation in operations {
            match operation {
                Operation::Read(words) => {
                    self.read_words(words)?;
                }
                Operation::Write(words) => {
                    self.write_words(words)?;
                }
                Operation::Transfer(read, write) => {
                    let mut transfer = NonBlockingTransfer::new(write, read);
                    nb::block!(transfer.exchange_nb(self))?;
                }
                Operation::TransferInPlace(words) => {
                    self.transfer_words_inplace(words)?;
                }
                Operation::DelayNs(_) => {
                    unimplemented!()
                }
            }
        }

        self.end_transaction();
        Ok(())
    }
}
