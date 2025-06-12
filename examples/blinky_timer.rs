#![deny(warnings)]
#![no_main]
#![no_std]

mod utilities;

use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, Ordering},
};

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;

use stm32h5xx_hal::gpio::gpioa::PA5; // LED pin
use stm32h5xx_hal::gpio::{Output, PushPull};
use stm32h5xx_hal::{pac, pac::interrupt, prelude::*, timer};
use utilities::logger::info;

static LED_IS_ON: AtomicBool = AtomicBool::new(false);
static LED: Mutex<RefCell<Option<PA5<Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<timer::Timer<pac::TIM2>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0().freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(250.MHz()).freeze(pwrcfg, &dp.SBS);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let mut led = gpioa.pa5.into_push_pull_output();
    led.set_low();

    let mut timer = dp.TIM2.timer(2.Hz(), ccdr.peripheral.TIM2, &ccdr.clocks);
    timer.listen(timer::Event::TimeOut);

    cortex_m::interrupt::free(|cs| {
        LED.borrow(cs).replace(Some(led));
        TIMER.borrow(cs).replace(Some(timer));
    });

    info!("Start blinking with timer...");
    // Enable TIM2 interrupt
    unsafe {
        cp.NVIC.set_priority(interrupt::TIM2, 1);
        NVIC::unmask::<interrupt>(interrupt::TIM2);
    }

    loop {
        // do_nothing
    }
}

/// Handle timer overflow
///
/// The interrupt should be configured at maximum priority, it won't take very long.
#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            timer.clear_irq();
        }
        // Signal that the interrupt fired
        let led_is_on = LED_IS_ON.fetch_not(Ordering::Relaxed);
        if let Some(led) = LED.borrow(cs).borrow_mut().as_mut() {
            if led_is_on {
                led.set_low();
            } else {
                led.set_high();
            }
        }
    })
}
