//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![deny(unsafe_code)] //  Don't allow unsafe code in this file.
#![allow(unused_imports)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
#[macro_use] //  Import macros from the following crates,
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
extern crate stm32f103xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.

use bluepill_hal::delay::Delay; //  Delay timer.
use bluepill_hal::prelude::*;   //  Define HAL traits.
use bluepill_hal::stm32f103xx::Peripherals;
use bluepill_hal::time::Hertz;
use core::fmt::Write; //  Provides writeln() function for debug console output.
use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.
use cortex_m_semihosting::hio; //  For displaying messages on the debug console. //  Clocks, flash memory, GPIO for the STM32 Blue Pill.

//  Black Pill starts execution at function main().
entry!(main);

//  Black Pill starts execution here. "-> !" means this function will never return (because of the loop).
fn main() -> ! {
    //  Show "Hello, world!" on the debug console, which is shown in OpenOCD. "mut" means that this object is mutable, i.e. it can change.
    //let mut debug_out = hio::hstdout().unwrap();
    //writeln!(debug_out, "Hello, world!").unwrap();

    //  Get peripherals (clocks, flash memory, GPIO) for the STM32 Black Pill microcontroller.
    let bluepill = Peripherals::take().unwrap();

    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let tim3 = bluepill.TIM3;
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    //  Get GPIO Port C, which also enables the Advanced Peripheral Bus 2 (APB2) clock for Port C.
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = bluepill.GPIOA.split(&mut rcc.apb2);

    //  Use Pin PC 13 of the Black Pill for GPIO Port C. Select Output Push/Pull mode for the pin, which is connected to our LED.
    let mut led = gpiob.pb9.into_push_pull_output(&mut gpiob.crh);
    //let mut pa6 = gpioa.pa6.into_alternate_push_pull(&mut gpioa.crl);
    //let mut pa7 = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let pb0 = gpiob.pb0.into_alternate_push_pull(&mut gpiob.crl);
    let pb1 = gpiob.pb1.into_alternate_push_pull(&mut gpiob.crl);

    //let pa2 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
    //let pa3 = gpioa.pa3.into_floating_input(&mut gpioa.crl);

    //  Create a delay timer from the RCC clocks.
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, clocks);
    let (mut pwm_pb0, _) = tim3.pwm(
        (pb0, pb1),
        &mut afio.mapr,
        Hertz(10000),
        clocks,
        &mut rcc.apb1,
    );
    pwm_pb0.enable();
    let mut cpt: u16 = 0;
    let max_pwm = pwm_pb0.get_max_duty();
    //  Loop forever.
    loop {
        pwm_pb0.set_duty(max_pwm / (1 + (cpt % 5)));
        //  Output 3.3V on the LED Pin and show a message in OpenOCD console.
        //led.set_high();
        //writeln!(debug_out, "LED is ON!").unwrap();

        //  Wait 1,000 millisec (1 sec).
        //delay.delay_ms(1_000_u16);

        //  Output 0V on the LED Pin and show a message in OpenOCD console.
        led.set_low();
        //writeln!(debug_out, "LED is OFF!").unwrap();
        cpt = (cpt + 1) % 65;

        //  Wait 1,000 millisec (1 sec).
        delay.delay_ms(1000_u16);
    }
}

//  For any hard faults, show a message on the debug console and stop.
exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.
exception!(*, default_handler);

fn default_handler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
