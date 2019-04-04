#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

use stm32f1xx_hal as f103_hal;

mod robot;

use crate::f103::Peripherals;
use crate::f103_hal::delay::Delay;
use crate::f103_hal::prelude::*;
use crate::f103_hal::stm32 as f103;
use crate::robot::init_peripherals;
use cortex_m::asm;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::ExceptionFrame;
use cortex_m_rt::{entry, exception};
use drs_0x01::addr::WritableRamAddr;
use drs_0x01::Servo as HServo;
use embedded_hal::serial::Write as EWrite;
use librobot::transmission::eth::{init_eth, SOCKET_UDP};
use librobot::transmission::servo::{Control, Servo};
use librobot::transmission::Jsonizable;
use nb::block;
#[allow(unused_imports)]
use panic_abort;
use w5500::*;

fn init_servos(connection: &mut impl EWrite<u8>, delay: &mut Delay) {
    let servo = HServo::new(0xFE);
    let message = servo.reboot();
    for b in message {
        let _ = block!(connection.write(b));
    }
    delay.delay_ms(250u8);
    let message = servo.enable_torque();
    for b in message {
        let _ = block!(connection.write(b));
    }
    delay.delay_ms(250u8);
    /*
    let message = servo.ram_write(WritableRamAddr::AckPolicy(2));
    for b in message {
        let _ = block!(connection.write(b));
    }*/
}

#[entry]
fn main() -> ! {
    let chip = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let mut robot = init_peripherals(chip, cortex);

    let mut eth = W5500::new(&mut robot.spi_eth, &mut robot.cs);
    init_eth(
        &mut eth,
        &mut robot.spi_eth,
        &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, 0x05),
        &IpAddress::new(192, 168, 1, 2),
    );
    init_servos(&mut robot.servo_tx, &mut robot.delay);

    robot.delay.delay_ms(50u32);

    let mut buffer = [0; 2048];

    loop {
        if let Some((_, _, size)) = eth
            .try_receive_udp(&mut robot.spi_eth, SOCKET_UDP, &mut buffer)
            .unwrap()
        {
            let _id = buffer[0];
            match Servo::from_json_slice(&buffer[1..size]) {
                Ok(servo) => {
                    let s = HServo::new(servo.id);
                    let msg = match servo.control {
                        Control::Position => s.set_position(servo.data),
                        Control::Speed => s.set_speed(
                            servo.data,
                            match servo.rotation {
                                librobot::servo::Rotation::Clockwise => {
                                    drs_0x01::Rotation::Clockwise
                                }
                                librobot::servo::Rotation::CounterClockwise => {
                                    drs_0x01::Rotation::CounterClockwise
                                }
                            },
                        ),
                    };
                    for b in msg {
                        block!(robot.servo_tx.write(b)).expect(
                            "Fail to communicate with \
                             servo",
                        );
                    }
                }
                Err(e) => panic!("{:#?}", e),
            }
        }
    }
}

//  For any hard faults, show a message on the debug console and stop.
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    asm::bkpt();
    panic!("Hard fault: {:#?}", ef);
}

//  For any unhandled interrupts, show a message on the debug console and stop.
#[exception]
fn DefaultHandler(irqn: i16) {
    asm::bkpt();
    panic!("Unhandled exception (IRQn = {})", irqn);
}
