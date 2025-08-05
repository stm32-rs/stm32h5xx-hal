#![no_main]
#![no_std]

mod utilities;

use embedded_dma::{ReadBuffer, WriteBuffer};
use rtic::app;
use stm32h5xx_hal::{
    gpdma::{periph::DmaDuplex, DmaChannel0, DmaChannel1, Word},
    gpio::{Output, PA5},
    pac,
    prelude::*,
    spi::{self, dma::SpiDma, Config as SpiConfig},
};

use rtic_monotonics::systick::prelude::*;
systick_monotonic!(Mono, 1000);

// Buffer is used to manage a reference to a static buffer returned by the cortex_m::singleton!
// macro and which can be with the DmaTransfer API (which requires passing ReadBuffer and
// WriteBuffer implementations by value) and then used to access the buffer after the transfer has
// completed.
struct Buffer<T: Word + 'static, const N: usize> {
    data: &'static mut [T; N],
}

impl<T, const N: usize> Buffer<T, N>
where
    T: Word + 'static,
{
    fn new(data: &'static mut [T; N]) -> Self {
        Self { data }
    }
}

unsafe impl<T, const N: usize> ReadBuffer for &mut Buffer<T, N>
where
    T: Word + 'static,
{
    type Word = T;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.data.as_ptr(), N)
    }
}

unsafe impl<T, const N: usize> WriteBuffer for &mut Buffer<T, N>
where
    T: Word + 'static,
{
    type Word = T;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (self.data.as_mut_ptr(), N)
    }
}

#[app(device = pac, dispatchers = [USART1, USART2], peripherals = true)]
mod app {

    use core::cell::Cell;

    use cortex_m::singleton;
    use stm32h5::stm32h503::{GPDMA1, NVIC};

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PA5<Output>,
        spi: SpiDma<
            pac::SPI2,
            DmaDuplex<pac::SPI2, u8, DmaChannel0<GPDMA1>, DmaChannel1<GPDMA1>>,
        >,
        source: Cell<Buffer<u8, 40>>,
        dest: Cell<Buffer<u8, 40>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        utilities::logger::init();

        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = pwr.vos0().freeze();

        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(192.MHz())
            .pll1_q_ck(64.MHz())
            .freeze(pwrcfg, &ctx.device.SBS);

        log::info!("Starting RTIC SPI example...");

        // Uncomment if use SysTick as monotonic timer
        Mono::start(ctx.core.SYST, ccdr.clocks.sysclk().raw());

        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);

        let led = gpioa.pa5.into_push_pull_output();

        let sck = gpiob.pb13.into_alternate();
        let miso = gpiob.pb14.into_alternate();
        let mosi = gpiob.pb15.into_alternate();

        let channels = ctx.device.GPDMA1.channels(ccdr.peripheral.GPDMA1);
        let tx_ch = channels.0;
        let rx_ch = channels.1;

        let spi = ctx
            .device
            .SPI2
            .spi(
                (sck, miso, mosi),
                SpiConfig::new(spi::MODE_0),
                1.MHz(),
                ccdr.peripheral.SPI2,
                &ccdr.clocks,
            )
            .use_dma_duplex(tx_ch, rx_ch);

        unsafe {
            NVIC::unmask(pac::interrupt::GPDMA1_CH0);
            NVIC::unmask(pac::interrupt::GPDMA1_CH1);
        };

        let src = singleton!(: [u8; 40] = [0; 40]).unwrap();
        let dest = singleton!(: [u8; 40] = [0; 40]).unwrap();

        tick::spawn().unwrap();
        spi_transfer::spawn().unwrap();
        (
            Shared {},
            Local {
                led,
                spi,
                source: Cell::new(Buffer::new(src)),
                dest: Cell::new(Buffer::new(dest)),
            },
        )
    }

    #[task(local = [led, count: u32 = 0], priority = 1)]
    async fn tick(ctx: tick::Context) {
        loop {
            ctx.local.led.toggle();
            *ctx.local.count += 1;
            log::info!("Tick {}", *ctx.local.count);
            Mono::delay(1000.millis()).await;
        }
    }

    #[task(local = [spi, count: u32 = 0, source, dest], priority = 2)]
    async fn spi_transfer(ctx: spi_transfer::Context) {
        loop {
            log::info!("Starting SPI transfer");

            let src = ctx.local.source.get_mut();
            let dest = ctx.local.dest.get_mut();
            src.data.fill(*ctx.local.count as u8);
            dest.data.fill(0);

            *ctx.local.count += 1;
            ctx.local.spi.transfer_dma(src, dest).await.unwrap();

            assert_eq!(
                *ctx.local.source.get_mut().data,
                *ctx.local.dest.get_mut().data
            );
            log::info!("Success!");
            Mono::delay(1000.millis()).await;
        }
    }
}
