use core::{convert::Infallible, future::Future, task::Poll};

use crate::{
    gpio::{exti::Exti, Edge, Input, Pin},
    interrupt,
};
use futures_util::task::AtomicWaker;

use crate::pac::EXTI;

static WAKERS: [AtomicWaker; 16] = [const { AtomicWaker::new() }; 16];

macro_rules! impl_irq {
    ($($INT:ident => $rpifX:ident, $fpifX:ident, $imX:ident, $i:literal),*) => {$(
        #[interrupt]
        fn $INT() {
            let exti = unsafe { EXTI::steal() };

            // Check pending
            let is_set = exti.rpr1().read().$rpifX().is_pending()
                | exti.fpr1().read().$fpifX().is_pending();

            cortex_m::interrupt::free(|_| {
                // Disable the triggered interrupt to prevent further
                // triggers and to signal the future that it is done
                exti.imr1().modify(|_, w| w.$imX().clear_bit());
            });

            if is_set {
                WAKERS[$i].wake();
            }

            // Clear pending bits
            exti.rpr1().write(|w| w.$rpifX().clear());
            exti.fpr1().write(|w| w.$fpifX().clear());
        }
    )*};
}

impl_irq! {
    EXTI0 => rpif0, fpif0, im0, 0,
    EXTI1 => rpif1, fpif1, im1, 1,
    EXTI2 => rpif2, fpif2, im2, 2,
    EXTI3 => rpif3, fpif3, im3, 3,
    EXTI4 => rpif4, fpif4, im4, 4,
    EXTI5 => rpif5, fpif5, im5, 5,
    EXTI6 => rpif6, fpif6, im6, 6,
    EXTI7 => rpif7, fpif7, im7, 7,
    EXTI8 => rpif8, fpif8, im8, 8,
    EXTI9 => rpif9, fpif9, im9, 9,
    EXTI10 => rpif10, fpif10, im10, 10,
    EXTI11 => rpif11, fpif11, im11, 11,
    EXTI12 => rpif12, fpif12, im12, 12,
    EXTI13 => rpif13, fpif13, im13, 13,
    EXTI14 => rpif14, fpif14, im14, 14,
    EXTI15 => rpif15, fpif15, im15, 15
}

#[non_exhaustive]
#[must_use = "Futures no nothing unless polled or awaited"]
struct ExtiFuture<const N: u8>;

impl<const N: u8> Future for ExtiFuture<N> {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        WAKERS[N as usize].register(cx.waker());

        let exti = unsafe { EXTI::steal() };

        if exti.imr1().read().bits() & N as u32 == 0 {
            // EXTIx interrupt disabled which the irq does once an edge is detected
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl<const N: u8> ExtiFuture<N> {
    fn new(edge: Edge) -> Self {
        use crate::gpio::exti::ExtiedPin;

        // SAFETY:
        // * Conjure a pin with mathing number to be able to setup exti
        //   - This isa safe since all pins with the same number share the
        //     same exti registers and only one pin per number can be setup for
        //     exti since they consume that exti channel.
        unsafe {
            let mut pin_handle = Pin::<'A', N, Input, true>::new();
            let mut exti = Exti(EXTI::steal());

            // Only modifies Exti channel N's entries of EXTI
            cortex_m::interrupt::free(|_| {
                pin_handle.trigger_on_edge(&mut exti, edge);

                pin_handle.clear_interrupt_pending_bit(Edge::Rising);
                pin_handle.clear_interrupt_pending_bit(Edge::Falling);

                pin_handle.enable_interrupt(&mut exti);
            });
        };

        ExtiFuture
    }
}

impl<const P: char, const N: u8, M> embedded_hal::digital::ErrorType
    for Pin<P, N, M, true>
{
    type Error = Infallible;
}

impl<const P: char, const N: u8, M> embedded_hal_async::digital::Wait
    for Pin<P, N, M, true>
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        if !self._is_low() {
            // Pin is already high, done.
            return Ok(());
        }
        ExtiFuture::<N>::new(Edge::Rising).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        if self._is_low() {
            // Pin is already low, done.
            return Ok(());
        }
        ExtiFuture::<N>::new(Edge::Rising).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        ExtiFuture::<N>::new(Edge::Rising).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        ExtiFuture::<N>::new(Edge::Falling).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        ExtiFuture::<N>::new(Edge::RisingFalling).await;
        Ok(())
    }
}
