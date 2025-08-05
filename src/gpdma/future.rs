use core::{
    future::{Future, IntoFuture},
    ops::{Deref, DerefMut},
    pin::Pin,
    task::{Context, Poll},
};

use embedded_dma::{ReadBuffer, WriteBuffer};
use futures_util::task::AtomicWaker;

use crate::interrupt;
use crate::stm32::{GPDMA1, GPDMA2};

use super::{
    ch::{
        Channel, ChannelRegs, DmaChannel0, DmaChannel1, DmaChannel2,
        DmaChannel3, DmaChannel4, DmaChannel5, DmaChannel6, DmaChannel7,
        DmaChannelRef,
    },
    DmaTransfer, Error, Instance, Word,
};

#[allow(private_bounds)]
pub trait DmaChannel: Channel + ChannelWaker {}

impl<DMA, CH, const N: usize> DmaChannel for DmaChannelRef<DMA, CH, N>
where
    DMA: Instance + InstanceWaker,
    CH: ChannelRegs,
    Self: Deref<Target = CH>,
{
}

#[allow(private_bounds)]
impl<'a, CH, S, D> IntoFuture for DmaTransfer<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    type Output = Result<(), Error>;
    type IntoFuture = DmaTransferFuture<'a, CH, S, D>;

    fn into_future(mut self) -> DmaTransferFuture<'a, CH, S, D> {
        self.enable_interrupts();
        DmaTransferFuture { transfer: self }
    }
}

pub struct DmaTransferFuture<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    transfer: DmaTransfer<'a, CH, S, D>,
}

impl<'a, CH, S, D> Deref for DmaTransferFuture<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    type Target = DmaTransfer<'a, CH, S, D>;

    fn deref(&self) -> &Self::Target {
        &self.transfer
    }
}

impl<'a, CH, S, D> DerefMut for DmaTransferFuture<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.transfer
    }
}

impl<'a, CH, S, D> Unpin for DmaTransferFuture<'a, CH, S, D>
where
    CH: DmaChannel,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
}

impl<'a, CH, S, D> Future for DmaTransferFuture<'a, CH, S, D>
where
    CH: DmaChannel + ChannelWaker,
    S: ReadBuffer<Word: Word>,
    D: WriteBuffer<Word: Word>,
{
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.channel.waker().register(cx.waker());
        if self.channel.check_transfer_complete()? {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

#[allow(private_bounds)]
impl<DMA, CH, const N: usize> DmaChannelRef<DMA, CH, N>
where
    DMA: Instance + InstanceWaker,
    CH: ChannelRegs,
    Self: Deref<Target = CH>,
{
    #[inline(always)]
    fn handle_interrupt() {
        let mut ch = Self::new();
        ch.disable_transfer_interrupts();
        ch.waker().wake();
    }
}

impl<DMA, CH, const N: usize> ChannelWaker for DmaChannelRef<DMA, CH, N>
where
    DMA: Instance + InstanceWaker,
    CH: ChannelRegs,
    Self: Deref<Target = CH>,
{
    #[inline(always)]
    fn waker(&self) -> &'static AtomicWaker {
        DMA::waker(N)
    }
}

macro_rules! gpdma_irq {
    ($GPDMA:ident, $CH:literal) => {
        paste::item! {
            #[interrupt]
            fn [<$GPDMA _CH $CH>]() {
                [< DmaChannel $CH>]::<$GPDMA>::handle_interrupt();
            }
        }
    };
}

pub(super) trait InstanceWaker {
    fn waker(idx: usize) -> &'static AtomicWaker;
}

pub(super) trait ChannelWaker {
    /// Returns a reference to the AtomicWaker for the channel.
    fn waker(&self) -> &'static AtomicWaker;
}

mod gpdma1 {
    use super::*;

    static WAKERS_GPDMA1: [AtomicWaker; 8] = [const { AtomicWaker::new() }; 8];

    #[allow(private_bounds)]
    impl InstanceWaker for GPDMA1 {
        #[inline(always)]
        fn waker(idx: usize) -> &'static AtomicWaker {
            &WAKERS_GPDMA1[idx]
        }
    }

    gpdma_irq!(GPDMA1, 0);
    gpdma_irq!(GPDMA1, 1);
    gpdma_irq!(GPDMA1, 2);
    gpdma_irq!(GPDMA1, 3);
    gpdma_irq!(GPDMA1, 4);
    gpdma_irq!(GPDMA1, 5);
    gpdma_irq!(GPDMA1, 6);
    gpdma_irq!(GPDMA1, 7);
}

mod gpdma2 {
    use super::*;

    static WAKERS_GPDMA2: [AtomicWaker; 8] = [const { AtomicWaker::new() }; 8];

    #[allow(private_bounds)]
    impl InstanceWaker for GPDMA2 {
        #[inline(always)]
        fn waker(idx: usize) -> &'static AtomicWaker {
            &WAKERS_GPDMA2[idx]
        }
    }

    gpdma_irq!(GPDMA2, 0);
    gpdma_irq!(GPDMA2, 1);
    gpdma_irq!(GPDMA2, 2);
    gpdma_irq!(GPDMA2, 3);
    gpdma_irq!(GPDMA2, 4);
    gpdma_irq!(GPDMA2, 5);
    gpdma_irq!(GPDMA2, 6);
    gpdma_irq!(GPDMA2, 7);
}
