//! Blink the LED (connected to Pin PC 13) on and off with 1 second interval.

#![deny(unsafe_code)] //  Don't allow unsafe code in this file.
#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]
#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

#[macro_use]
extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
#[macro_use] //  Import macros from the following crates,
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate drs_0x01;
extern crate embedded_hal;
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
extern crate stm32f103xx_hal as bluepill_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
#[macro_use]
extern crate nb;
extern crate librobot;

use cortex_m::asm;

use bluepill_hal::delay::Delay; //  Delay timer.
use bluepill_hal::prelude::*;   //  Define HAL traits.
use bluepill_hal::serial::Serial;
use bluepill_hal::stm32f103xx::Peripherals;
use bluepill_hal::time::Hertz;
use core::fmt::Write; //  Provides writeln() function for debug console output.
use cortex_m_rt::ExceptionFrame; //  Stack frame for exception handling.
use cortex_m_semihosting::hio;
use embedded_hal::serial::Write as EWrite; //  For displaying messages on the debug console. //  Clocks, flash memory, GPIO for the STM32 Blue Pill.

use drs_0x01::prelude::Servo as HServo;

use librobot::transmission::{Control, Frame, FrameReader, Message, Servo, ServoGroup};

//  Black Pill starts execution at function main().
entry!(main);

fn init_servos(connection: &mut impl EWrite<u8>, delay: &mut Delay) {
    let servo = HServo::new(0xFE);
    let message = servo.reboot();
    for b in message {
        block!(connection.write(b));
    }
    delay.delay_ms(250u8);
    let message = servo.enable_torque();
    for b in message {
        block!(connection.write(b));
    }
}

//  Black Pill starts execution here. "-> !" means this function will never return (because of the loop).
fn main() -> ! {
    //  Get peripherals (clocks, flash memory, GPIO) for the STM32 Black Pill microcontroller.
    let bluepill = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    //  Get the clocks from the STM32 Reset and Clock Control (RCC) and freeze the Flash Access Control Register (ACR).
    let mut rcc = bluepill.RCC.constrain();
    let mut flash = bluepill.FLASH.constrain();
    let mut afio = bluepill.AFIO.constrain(&mut rcc.apb2);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let channels = bluepill.DMA1.split(&mut rcc.ahb);

    //  Configuration des GPIOs
    let mut gpiob = bluepill.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = bluepill.GPIOA.split(&mut rcc.apb2);

    // Configuration des PINS
    let pb6 = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let pb7 = gpiob.pb7.into_floating_input(&mut gpiob.crl);
    let pa2 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let pa3 = gpioa.pa3.into_floating_input(&mut gpioa.crl);

    // Configuration des USART

    let pc = Serial::usart1(
        bluepill.USART1,
        (pb6, pb7),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb2,
    );
    let servo = Serial::usart2(
        bluepill.USART2,
        (pa2, pa3),
        &mut afio.mapr,
        115_200.bps(),
        clocks,
        &mut rcc.apb1,
    );

    let (mut pc_tx, mut pc_rx) = pc.split();
    let (mut servo_tx, mut servo_rx) = servo.split();

    //  Create a delay timer from the RCC clocks.
    let mut delay = Delay::new(cp.SYST, clocks);

    init_servos(&mut servo_tx, &mut delay);

    delay.delay_ms(50u32);
    let mut buf1 = singleton!(: [u8; 16] = [0; 16]).unwrap();
    let mut reader = FrameReader::new();

    let c = channels.5;
    let mut transfer = pc_rx.read_exact(c, buf1);
    loop {
        // Transfert DMA
        {
            let (buf, c5, serial_handle) = transfer.wait();
            let working_buffer = buf.clone();
            transfer = serial_handle.read_exact(c5, buf);
            for b in &working_buffer {
                reader.step(*b);
            }
        }

        if reader.get_buffer_size() > 0 {
            let frame = reader.pop_frame().unwrap();
            if let Ok(servos) = ServoGroup::new(frame.data) {
                for servo in servos.servos {
                    asm::bkpt();
                    let s = HServo::new(servo.id);
                    let msg = match servo.control {
                        Control::Position(pos) => s.set_position(pos),
                        Control::Speed(speed) => s.set_speed(speed),
                    };
                    for b in msg {
                        block!(servo_tx.write(b)).unwrap();
                    }
                }
            }
        }

        /*let h1 = block!(pc_rx.read()).unwrap();
        if h1 == 0xAC {
            let h2 = block!(pc_rx.read()).unwrap();
            if h2 == 0xDC {
                let h3 = block!(pc_rx.read()).unwrap();
                if h3 == 0xAB {
                    let h4 = block!(pc_rx.read()).unwrap();
                    if h4 == 0xBA {
                        let size_low = block!(pc_rx.read()).unwrap();
                        let size_high = block!(pc_rx.read()).unwrap();
                        let id = block!(pc_rx.read()).unwrap();
                        let mut mess = Message::new();
                        for _ in 0..(size_low - 1) {
                            mess.push(block!(pc_rx.read()).unwrap());
                        }
                        asm::bkpt();
                        if let Ok(servos) = ServoGroup::new(mess) {
                            for servo in servos.servos {
                                asm::bkpt();
                                let s = HServo::new(servo.id);
                                let msg = match servo.control {
                                    Control::Position(pos) => s.set_position(pos),
                                    Control::Speed(speed) => s.set_speed(speed),
                                };
                                for b in msg {
                                    block!(servo_tx.write(b)).unwrap();
                                }
                            }
                        } else {
                            asm::bkpt();
                        }
                    }
                }
            }
        }*/
    }
}

//  For any hard faults, show a message on the debug console and stop.
exception!(HardFault, hard_fault);

fn hard_fault(ef: &ExceptionFrame) -> ! {
    asm::bkpt();
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.
exception!(*, default_handler);

fn default_handler(irqn: i16) {
    asm::bkpt();
    panic!("Unhandled exception (IRQn = {})", irqn);
}
