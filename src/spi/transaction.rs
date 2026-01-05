//! This module abstracts the handling of SPI transactions with different buffer access patterns.
//! The Transaction struct encapsulates the buffer access operations for reading and writing data
//! that can be passed repeatedly to the driver for performing non-blocking operations.
use core::marker::PhantomData;

/// Trait for SPI transaction operations. This defines the buffer accesses for each type of SPI
/// operation (read, write, transfer, etc.).
pub(super) trait Op<W = u8> {
    fn read_buf(&mut self) -> &mut [W];
    fn read_idx(&mut self) -> &mut usize;
    fn read_len(&self) -> usize;
    fn write_buf(&self) -> &[W];
    fn write_idx(&mut self) -> &mut usize;
    fn write_len(&self) -> usize;
}

/// Read operation for a SPI transaction
pub struct Read<'a, W = u8> {
    read_idx: usize,
    write_idx: usize, // Number of bytes flushed out the TX buffer to generate clocks for read operations on full duplex bus
    buf: &'a mut [W],
}

impl<W> Op<W> for Read<'_, W> {
    #[inline(always)]
    fn read_buf(&mut self) -> &mut [W] {
        self.buf
    }

    #[inline(always)]
    fn read_idx(&mut self) -> &mut usize {
        &mut self.read_idx
    }

    #[inline(always)]
    fn read_len(&self) -> usize {
        self.buf.len()
    }

    #[inline(always)]
    fn write_buf(&self) -> &[W] {
        panic!("Read operation!") // The write buffer is not used in a read operation
    }

    #[inline(always)]
    fn write_idx(&mut self) -> &mut usize {
        &mut self.write_idx
    }

    #[inline(always)]
    fn write_len(&self) -> usize {
        0
    }
}

/// Write operation for a SPI transaction
pub struct Write<'a, W = u8> {
    read_idx: usize, // Number of bytes discarded from RX buffer for write operations on full duplex bus
    write_idx: usize,
    out: &'a [W],
}

impl<W> Op<W> for Write<'_, W> {
    #[inline(always)]
    fn read_buf(&mut self) -> &mut [W] {
        panic!("Write operation!") // The read buffer is not used in a write operation
    }

    #[inline(always)]
    fn read_idx(&mut self) -> &mut usize {
        &mut self.read_idx
    }

    #[inline(always)]
    fn read_len(&self) -> usize {
        0
    }

    #[inline(always)]
    fn write_buf(&self) -> &[W] {
        self.out
    }

    #[inline(always)]
    fn write_idx(&mut self) -> &mut usize {
        &mut self.write_idx
    }

    #[inline(always)]
    fn write_len(&self) -> usize {
        self.out.len()
    }
}

pub struct Transfer<'a, W = u8> {
    read_idx: usize,
    write_idx: usize,
    read: &'a mut [W],
    write: &'a [W],
}

/// Bidirectional transfer operation for a SPI transaction
impl<W> Op<W> for Transfer<'_, W> {
    #[inline(always)]
    fn read_buf(&mut self) -> &mut [W] {
        self.read
    }

    #[inline(always)]
    fn read_idx(&mut self) -> &mut usize {
        &mut self.read_idx
    }

    #[inline(always)]
    fn read_len(&self) -> usize {
        self.read.len()
    }

    #[inline(always)]
    fn write_buf(&self) -> &[W] {
        self.write
    }

    #[inline(always)]
    fn write_idx(&mut self) -> &mut usize {
        &mut self.write_idx
    }

    #[inline(always)]
    fn write_len(&self) -> usize {
        self.write.len()
    }
}

/// Bidirectional transfer operation for a SPI transaction that modifies the same buffer in place
pub struct TransferInplace<'a, W = u8> {
    read_idx: usize,
    write_idx: usize,
    buf: &'a mut [W],
}

impl<W> Op<W> for TransferInplace<'_, W> {
    #[inline(always)]
    fn read_buf(&mut self) -> &mut [W] {
        self.buf
    }

    #[inline(always)]
    fn read_idx(&mut self) -> &mut usize {
        &mut self.read_idx
    }

    #[inline(always)]
    fn read_len(&self) -> usize {
        self.buf.len()
    }

    #[inline(always)]
    fn write_buf(&self) -> &[W] {
        self.buf
    }

    #[inline(always)]
    fn write_idx(&mut self) -> &mut usize {
        &mut self.write_idx
    }

    #[inline(always)]
    fn write_len(&self) -> usize {
        self.buf.len()
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Transaction<OP, W> {
    op: OP,
    _word_type: PhantomData<W>,
}

impl<'a, OP, W> Transaction<OP, W> {
    pub(super) fn read(buf: &'a mut [W]) -> Transaction<Read<'a, W>, W> {
        Transaction::<Read<'a, W>, W> {
            op: Read {
                read_idx: 0,
                write_idx: 0,
                buf,
            },
            _word_type: PhantomData,
        }
    }

    pub(super) fn write(out: &'a [W]) -> Transaction<Write<'a, W>, W> {
        Transaction::<Write<'a, W>, W> {
            op: Write {
                read_idx: 0,
                write_idx: 0,
                out,
            },
            _word_type: PhantomData,
        }
    }

    pub(super) fn transfer(
        write: &'a [W],
        read: &'a mut [W],
    ) -> Transaction<Transfer<'a, W>, W> {
        Transaction::<Transfer<'a, W>, W> {
            op: Transfer {
                read_idx: 0,
                write_idx: 0,
                read,
                write,
            },
            _word_type: PhantomData,
        }
    }

    pub(super) fn transfer_inplace(
        buf: &'a mut [W],
    ) -> Transaction<TransferInplace<'a, W>, W> {
        Transaction::<TransferInplace<'a, W>, W> {
            op: TransferInplace {
                read_idx: 0,
                write_idx: 0,
                buf,
            },
            _word_type: PhantomData,
        }
    }
}

#[allow(private_bounds)]
impl<OP: Op<W>, W> Transaction<OP, W> {
    #[inline(always)]
    pub(super) fn is_complete(&mut self) -> bool {
        self.is_read_complete()
            && self.rx_remainder_to_discard() == 0
            && self.is_write_complete()
            && self.tx_flush_remainder() == 0
    }

    #[inline(always)]
    pub(super) fn is_read_complete(&mut self) -> bool {
        *self.op.read_idx() >= self.op.read_len()
    }

    #[inline(always)]
    pub(super) fn is_write_complete(&mut self) -> bool {
        *self.op.write_idx() >= self.op.write_len()
    }

    #[inline(always)]
    pub(super) fn read_buf(&mut self) -> &mut [W] {
        let read_idx = *self.op.read_idx();
        &mut self.op.read_buf()[read_idx..]
    }

    #[inline(always)]
    pub(super) fn write_buf(&mut self) -> &[W] {
        let write_idx = *self.op.write_idx();
        &self.op.write_buf()[write_idx..]
    }

    #[inline(always)]
    pub(super) fn advance_write_idx(&mut self, count: usize) {
        *self.op.write_idx() += count;
    }

    #[inline(always)]
    pub(super) fn advance_read_idx(&mut self, count: usize) {
        *self.op.read_idx() += count;
    }

    #[inline(always)]
    pub(super) fn rx_remainder_to_discard(&mut self) -> usize {
        self.op.write_len().saturating_sub(*self.op.read_idx())
    }

    #[inline(always)]
    pub(super) fn tx_flush_remainder(&mut self) -> usize {
        self.op.read_len().saturating_sub(*self.op.write_idx())
    }
}
