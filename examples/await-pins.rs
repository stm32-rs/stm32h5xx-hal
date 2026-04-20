#![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;
use embedded_hal_async::digital::Wait;
use stm32h5xx_hal::{
    gpio::{ExtiExt, ExtiPin},
    pac,
    prelude::*,
};
use utilities::logger::info;

#[rtic::app(device = pac, dispatchers = [USART1, USART2], peripherals = true)]
mod app {
    use stm32h5xx_hal::gpio;

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        pin: gpio::PA5<gpio::Input, true>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;

        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.vos0().freeze();

        // Constrain and Freeze clock
        let rcc = dp.RCC.constrain();
        let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

        let (mut exti, exti_channels) = dp.EXTI.split();

        let pin = gpioa
            .pa5
            .into_input()
            .make_interrupt_source(exti_channels.ch5, &mut exti);

        (Shared {}, Local { pin })
    }

    #[task(local = [pin])]
    async fn foo(ctx: foo::Context) {
        loop {
            ctx.local.pin.wait_for_high().await.unwrap();
            info!("On");
            ctx.local.pin.wait_for_low().await.unwrap();
            info!("Off");
        }
    }
}
