use crate::f103;
use crate::f103::Peripherals;
use crate::f103::{interrupt, SPI1, USART3};
use crate::f103_hal::delay::Delay;
use crate::f103_hal::gpio::{
    gpioa::*, gpiob::*, gpioc::*, Alternate, Floating, Input, Output, PushPull,
};
use crate::f103_hal::prelude::*;
use crate::f103_hal::serial::{Rx, Serial, Tx};
use crate::f103_hal::spi::*;
use crate::f103_hal::timer::{Event, Timer};
use crate::CortexPeripherals;
use cortex_m::asm;
use stm32f1xx_hal::device::Interrupt::TIM3;

type SpiPins = (
    PA5<Alternate<PushPull>>,
    PA6<Input<Floating>>,
    PA7<Alternate<PushPull>>,
);

static mut LED_BLINK: Option<PC13<Output<PushPull>>> = None;

pub struct Robot<T, K, P> {
    pub spi_eth: Spi<K, P>,
    pub servo_tx: Tx<T>,
    pub servo_rx: Rx<T>,
    pub delay: Delay,
    pub cs: PB13<Output<PushPull>>,
    pub led_hardfault: PB7<Output<PushPull>>,
    pub led_feedback: PC14<Output<PushPull>>,
}

pub fn init_peripherals(
    chip: Peripherals,
    mut cortex: CortexPeripherals,
) -> Robot<USART3, SPI1, SpiPins> {
    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    let mut rcc = chip.RCC.constrain();
    let mut flash = chip.FLASH.constrain();
    let mut afio = chip.AFIO.constrain(&mut rcc.apb2);
    let mut nvic = cortex.NVIC;
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);
    //let _channels = chip.DMA1.split(&mut rcc.ahb);

    cortex.DCB.enable_trace();
    cortex.DWT.enable_cycle_counter();

    //  Configuration des GPIOs
    let mut gpioa = chip.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = chip.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = chip.GPIOC.split(&mut rcc.apb2);

    // Configuration des PINS

    // Slave select, on le fixe à un état bas (on n'en a pas besoin, une seule communication)
    let mut cs = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    cs.set_low();

    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let pb10 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let pb11 = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    let mut led_hardfault = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
    let mut led_feedback = gpioc.pc14.into_push_pull_output(&mut gpioc.crh);
    let led_black_pill = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    unsafe {
        LED_BLINK = Some(led_black_pill);
    }
    led_hardfault.set_low();
    led_feedback.set_low();

    // Configuration des USART

    let spi = Spi::spi1(
        chip.SPI1,
        (sclk, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let servo = Serial::usart3(
        chip.USART3,
        (pb10, pb11),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb1,
    );

    let (servo_tx, servo_rx) = servo.split();

    nvic.enable(TIM3);

    let mut t = Timer::tim3(chip.TIM3, 5.hz(), clocks, &mut rcc.apb1);
    t.listen(Event::Update);

    //  Create a delay timer from the RCC clocks.
    let delay = Delay::new(cortex.SYST, clocks);

    Robot {
        spi_eth: spi,
        servo_tx,
        servo_rx,
        delay,
        cs,
        led_hardfault,
        led_feedback,
    }
}

#[interrupt]
fn TIM3() {
    unsafe {
        if let Some(led) = &mut LED_BLINK {
            if led.is_set_high() {
                led.set_low()
            } else {
                led.set_high();
            }
        }
        (*f103::TIM3::ptr()).sr.write(|w| w.uif().clear_bit());
    }
}
