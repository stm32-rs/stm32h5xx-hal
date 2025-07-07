use super::{
    Error, Instance, Op, Read, Spi, Transaction, Transfer, TransferInplace,
    Word, Write,
};

/// Trait that provides non-blocking SPI operations.
pub trait NonBlocking<W: Word> {
    /// Start a non-blocking read operation. This will return a [`Transaction`] that can be processed
    /// by calling [`NonBlocking::transfer_nonblocking`] until it completes.
    fn start_nonblocking_read<'a>(
        &mut self,
        buf: &'a mut [W],
    ) -> Result<Transaction<Read<'a, W>, W>, Error>;

    /// Start a non-blocking write operation. This will return a [`Transaction`] that can be processed
    /// by calling [`NonBlocking::transfer_nonblocking`] until it completes.
    fn start_nonblocking_write<'a>(
        &mut self,
        words: &'a [W],
    ) -> Result<Transaction<Write<'a, W>, W>, Error>;

    /// Start a non-blocking full duplex transfer operation. This will return a [`Transaction`] that
    /// can be processed by calling [`NonBlocking::transfer_nonblocking`] until it completes.
    fn start_nonblocking_duplex_transfer<'a>(
        &mut self,
        read: &'a mut [W],
        write: &'a [W],
    ) -> Result<Transaction<Transfer<'a, W>, W>, Error>;

    /// Start a non-blocking full duplex transfer operation that reuses the same buffer for transmit
    /// and receive. This will return a [`Transaction`] that can be processed by calling
    /// [`NonBlocking::transfer_nonblocking`] until it completes.
    fn start_nonblocking_duplex_transfer_inplace<'a>(
        &mut self,
        words: &'a mut [W],
    ) -> Result<Transaction<TransferInplace<'a, W>, W>, Error>;

    /// Process a transaction in a non-blocking manner. If the transaction completes during this
    /// call, the method will return `Ok(None)`. If the transaction is still in progress, the method
    /// will return `Ok(Some(transaction))`. If an error occurs during the transaction,
    /// the method will return `Err(Error)`.
    /// Upon completion of the transaction, or upon error the method will ensure that the SPI
    /// peripheral is properly disabled and ready for the next operation.
    #[allow(private_bounds)]
    fn transfer_nonblocking<OP: Op<W>>(
        &mut self,
        transaction: Transaction<OP, W>,
    ) -> Result<Option<Transaction<OP, W>>, Error>;
}

impl<SPI: Instance, W: Word> NonBlocking<W> for Spi<SPI, W> {
    fn start_nonblocking_read<'a>(
        &mut self,
        buf: &'a mut [W],
    ) -> Result<Transaction<Read<'a, W>, W>, Error> {
        self.start_read(buf)
    }

    fn start_nonblocking_write<'a>(
        &mut self,
        words: &'a [W],
    ) -> Result<Transaction<Write<'a, W>, W>, Error> {
        self.start_write(words)
    }

    fn start_nonblocking_duplex_transfer<'a>(
        &mut self,
        read: &'a mut [W],
        write: &'a [W],
    ) -> Result<Transaction<Transfer<'a, W>, W>, Error> {
        self.start_transfer(read, write)
    }

    fn start_nonblocking_duplex_transfer_inplace<'a>(
        &mut self,
        words: &'a mut [W],
    ) -> Result<Transaction<TransferInplace<'a, W>, W>, Error> {
        self.start_transfer_inplace(words)
    }

    #[allow(private_bounds)]
    fn transfer_nonblocking<OP: Op<W>>(
        &mut self,
        transaction: Transaction<OP, W>,
    ) -> Result<Option<Transaction<OP, W>>, Error> {
        self.transfer_nonblocking_internal(transaction)
    }
}
