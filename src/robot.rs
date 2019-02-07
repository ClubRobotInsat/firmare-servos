use crate::f103::Peripherals;
use crate::CortexPeripherals;

use crate::f103_hal::delay::Delay;
use crate::f103_hal::gpio::{gpiob::*, gpioc::*, Alternate, Floating, Input, Output, PushPull};
use crate::f103_hal::prelude::*;
use crate::f103_hal::serial::{Rx, Serial, Tx};
use crate::f103_hal::spi::*;

use crate::f103::{SPI1, USART3};

type SpiPins = (
    PB3<Alternate<PushPull>>,
    PB4<Input<Floating>>,
    PB5<Alternate<PushPull>>,
);

pub struct Robot<T, K, P> {
    pub spi_eth: Spi<K, P>,
    pub servo_tx: Tx<T>,
    pub servo_rx: Rx<T>,
    pub delay: Delay,
    pub cs: PB13<Output<PushPull>>,
    pub led_hardfault: PB7<Alternate<PushPull>>,
    pub led_feedback: PC14<Alternate<PushPull>>,
}

pub fn init_peripherals(
    chip: Peripherals,
    cortex: CortexPeripherals,
) -> Robot<USART3, SPI1, SpiPins> {
    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    let mut rcc = chip.RCC.constrain();
    let mut flash = chip.FLASH.constrain();
    let mut afio = chip.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    //let _channels = chip.DMA1.split(&mut rcc.ahb);

    //  Configuration des GPIOs
    let mut gpiob = chip.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = chip.GPIOC.split(&mut rcc.apb2);

    // Configuration des PINS

    // Slave select, on le fixe à un état bas (on n'en a pas besoin, une seule communication)
    let mut cs = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
    cs.set_low();

    let sclk = gpiob.pb3.into_alternate_push_pull(&mut gpiob.crl);
    let miso = gpiob.pb4.into_floating_input(&mut gpiob.crl);
    let mosi = gpiob.pb5.into_alternate_push_pull(&mut gpiob.crl);

    let pb10 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let pb11 = gpiob.pb11.into_floating_input(&mut gpiob.crh);

    let mut led_hardfault = gpiob.pb7.into_alternate_push_pull(&mut gpiob.crl);
    let mut led_feedback = gpioc.pc14.into_alternate_push_pull(&mut gpioc.crh);
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
