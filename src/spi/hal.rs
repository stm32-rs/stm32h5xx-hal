use embedded_hal::spi::{
    Error as HalError, ErrorKind, ErrorType, Operation, SpiBus, SpiDevice,
};

use super::{Error, Instance, Spi, Word};

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

impl<SPI, W: Word> ErrorType for Spi<SPI, W> {
    type Error = Error;
}

impl<SPI: Instance, W: Word> SpiBus<W> for Spi<SPI, W> {
    fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.read(words)
    }

    fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.write(words)
    }

    fn transfer(
        &mut self,
        read: &mut [W],
        write: &[W],
    ) -> Result<(), Self::Error> {
        self.transfer(read, write)
    }

    fn transfer_in_place(
        &mut self,
        words: &mut [W],
    ) -> Result<(), Self::Error> {
        self.transfer_inplace(words)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // This is handled within each of the above functions
        Ok(())
    }
}

trait OperationExt {
    fn len(&self) -> usize;
}

impl<W> OperationExt for Operation<'_, W> {
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

impl<SPI: Instance, W: Word> Spi<SPI, W> {
    #[inline(always)]
    fn perform_operation<'a>(
        &mut self,
        operation: &mut Operation<'a, W>,
    ) -> Result<(), Error> {
        match operation {
            Operation::Read(words) => self.read_words(words),
            Operation::Write(words) => self.write_words(words),
            Operation::Transfer(read, write) => {
                self.transfer_words(read, write)
            }
            Operation::TransferInPlace(words) => {
                self.transfer_words_inplace(words)
            }
            Operation::DelayNs(_) => {
                unimplemented!()
            }
        }
    }
}

impl<SPI: Instance, W: Word> SpiDevice<W> for Spi<SPI, W> {
    fn transaction(
        &mut self,
        operations: &mut [Operation<'_, W>],
    ) -> Result<(), Self::Error> {
        let len = operations.iter().fold(0, |acc, op| acc + op.len());
        if len == 0 {
            return Ok(());
        }
        self.setup_transaction(len);

        for operation in operations {
            match self.perform_operation(operation) {
                Ok(()) => {}
                Err(error) => {
                    self.abort_transaction();
                    return Err(error);
                }
            }
        }

        self.end_transaction();
        Ok(())
    }
}
