//! This module provides adds a Future implementation for GPDMA transfers, allowing DMA transfers
//! to be awaited asynchronously. GPDMA futures are enabled with the `gpdma-futures`. This future
//! implentation uses GPDMA channel interrupts to drive the future.
//!
//! Note that when the `gpdma-futures` feature is enabled, a set of `AtomicWaker`s are created for
//! each GPDMA channel and defined statically. They are used to wake up the task that is waiting
//! for the transfer to complete.
//!
//! It is necessary to unmask each required GPDMA channel interrupts in the NVIC to use this
//! feature. This is NOT done automatically so as to allow fine grained control by the user over
//! which interrupts are enabled in a system. To do so:
//!```
//!    use stm32h5xx_hal::pac::{NVIC, interrupt};
//!
//!    // Un-mask the interrupt for GPDMA 1 channel 0
//!    unsafe {
//!        NVIC::unmask(interrupt::GPDMA1_CH0);
//!    };
//! ```
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

// This trait is defined to be bound by ChannelWaker when futures are enabled via the
// `gpdma-futures` feature. This is to ensure that the waker associated with a channel can
// be accessed from the interrupt handler and the DmaTransfer `poll` function.
// Specifically, this alternate definition is needed to ensure that the DmaChannel implementations
// that are exposed to user code are bound to the ChannelWaker trait, which is also defined and
// and implemented for DmaChannelRef in this module. Without defining this trait, the futures
// implementation would be leaked into the channel definitions when futures are not enabled.
// Given that not everyone will use futures, and enabling them requires statically allocating RAM,
// it's better to redefine the trait here.
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

/// The `DmaTransferFuture` struct is a wrapper around the `DmaTransfer` struct that implements
/// the `Future` trait. It allows the DMA transfer to be awaited asynchronously, enabling the
/// use of DMA transfers in an asynchronous context, such as with the RTIC framework. It is created
/// when the `DmaTransfer` is awaited via the `IntoFuture` trait implementation. It's main function
/// is to ensure that interrupts are enabled when the transfer is awaited so that the future can
/// be driven by the channel interrupt.
#[doc(hidden)]
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
        // This creates a DmaChannelRef instance for channel N, which is be a duplicate to the
        // DmaChannelRef instance that is held as a mutable reference in the DmaTransfer struct
        // while a transfer is in progress. However, it is only used to disable the transfer
        // interrupts for the channel and wake up the task that is waiting for the transfer to
        // complete, which can be done safely.
        //
        // When the DmaTransfer struct is dropped, interrupts are disabled for the channel,
        // preventing this interrupt from being triggered. Interrupts are only enabled when the
        // transfer is awaited (calling IntoFuture::into_future).
        let mut ch = Self::new();

        // This is a single volatile write to the channel's interrupt status register to disable
        // interrupts for the channel so this interrupt doesn't trigger again while the transfer
        // struct is being polled.
        ch.disable_transfer_interrupts();

        // This is an atomic operation to wake up the task that is waiting for the transfer to
        // complete.
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

/// Private trait that provides access to the [`AtomicWaker`]s for a particular GPDMA . It is only
/// implemented when the `gpdma-futures` feature is enabled.
pub(super) trait InstanceWaker {
    fn waker(idx: usize) -> &'static AtomicWaker;
}

/// Private trait that provides access to the [`AtomicWaker`] for a specific channel. It is only
/// implemented when the `gpdma-futures` feature is enabled.
pub(super) trait ChannelWaker {
    /// Returns a reference to the AtomicWaker for the channel.
    fn waker(&self) -> &'static AtomicWaker;
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
