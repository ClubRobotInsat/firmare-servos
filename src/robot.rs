use f103::Peripherals;
use CortexPeripherals;

use f103_hal::delay::Delay;
use f103_hal::gpio::{gpioa::*, gpiob::*, Alternate, Floating, Input, Output, PushPull};
use f103_hal::prelude::*;
use f103_hal::serial::{Rx, Serial, Tx};
use f103_hal::spi::*;

use f103::{SPI1, USART3};

type SpiPins = (
    PA5<Alternate<PushPull>>,
    PA6<Input<Floating>>,
    PA7<Alternate<PushPull>>,
);

pub struct Robot<T, K, P> {
    pub spi_eth: Spi<K, P>,
    pub servo_tx: Tx<T>,
    pub servo_rx: Rx<T>,
    pub delay: Delay,
    pub pb8: PB8<Output<PushPull>>,
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
    let _channels = chip.DMA1.split(&mut rcc.ahb);

    //  Configuration des GPIOs
    let mut gpiob = chip.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = chip.GPIOA.split(&mut rcc.apb2);

    // Configuration des PINS

    let mut pb8 = gpiob.pb8.into_push_pull_output(&mut gpiob.crh);
    pb8.set_low();

    let sclk = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);

    let pb10 = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let pb11 = gpiob.pb11.into_floating_input(&mut gpiob.crh);

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
        pb8,
    }
}
