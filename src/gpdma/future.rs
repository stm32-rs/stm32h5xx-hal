use core::{
    future::{Future, IntoFuture},
    ops::{Deref, DerefMut},
    pin::Pin,
    task::{Context, Poll},
};

use futures_util::task::AtomicWaker;

use crate::interrupt;
use crate::stm32::{GPDMA1, GPDMA2};

use super::{
    ch::{
        ChannelRegs, DmaChannel, DmaChannel0, DmaChannel1, DmaChannel2,
        DmaChannel3, DmaChannel4, DmaChannel5, DmaChannel6, DmaChannel7,
        DmaChannelImpl, DmaChannelRef,
    },
    DmaTransfer, Error, Instance,
};

#[allow(private_bounds)]
impl<'a, CH: DmaChannel + ChannelWaker> IntoFuture for DmaTransfer<'a, CH> {
    type Output = Result<(), Error>;
    type IntoFuture = DmaTransferFuture<'a, CH>;
    
    fn into_future(self) -> DmaTransferFuture<'a, CH> {
        DmaTransferFuture { transfer: self }
    }
}

pub struct DmaTransferFuture<'a, CH: DmaChannel> {
    transfer: DmaTransfer<'a, CH>,
}

impl<'a, CH> Deref for DmaTransferFuture<'a, CH>
where
    CH: DmaChannel,
{
    type Target = DmaTransfer<'a, CH>;

    fn deref(&self) -> &Self::Target {
        &self.transfer
    }
}

impl<'a, CH> DerefMut for DmaTransferFuture<'a, CH>
where
    CH: DmaChannel,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.transfer
    }
}

impl<'a, CH> Drop for DmaTransferFuture<'a, CH>
where
    CH: DmaChannel,
{
    fn drop(&mut self) {
        match self.abort() {
            Ok(()) => {}
            Err(_error) => {
                #[cfg(feature = "log")]
                log::error!("Error aborting DMA transfer: {_error:?}");
            }
        }
        self.disable_interrupts();
    }
}

impl<'a, CH: DmaChannel> Unpin for DmaTransferFuture<'a, CH> {}

impl<'a, CH> Future for DmaTransferFuture<'a, CH>
where
    CH: DmaChannel + ChannelWaker,
{
    type Output = Result<(), Error>;

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        self.channel.waker().register(cx.waker());
        if self.is_transfer_complete()? {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }
}

#[allow(private_bounds)]
impl<DMA, CH, const N: usize> DmaChannelImpl<DmaChannelRef<DMA, CH, N>>
where
    DMA: Instance,
    CH: ChannelRegs,
    Self: ChannelWaker,
    DmaChannelRef<DMA, CH, N>: ChannelRegs,
{
    #[inline(always)]
    fn handle_interrupt() {
        let ch = Self::new();
        ch.waker().wake();
        ch.disable_transfer_interrupts();
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

trait ChannelWaker {
    fn waker(&self) -> &'static AtomicWaker;
}

mod gpdma1 {
    use super::*;

    static WAKERS_GPDMA1: [AtomicWaker; 8] = [const { AtomicWaker::new() }; 8];

    #[allow(private_bounds)]
    impl<CH: ChannelRegs, const N: usize> ChannelWaker
        for DmaChannelImpl<DmaChannelRef<GPDMA1, CH, N>>
    {
        fn waker(&self) -> &'static AtomicWaker {
            &WAKERS_GPDMA1[N]
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
    impl<CH: ChannelRegs, const N: usize> ChannelWaker
        for DmaChannelImpl<DmaChannelRef<GPDMA2, CH, N>>
    {
        fn waker(&self) -> &'static AtomicWaker {
            &WAKERS_GPDMA2[N]
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
