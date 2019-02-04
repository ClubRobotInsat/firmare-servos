#![no_main] //  Don't use the Rust standard bootstrap. We will provide our own.
#![no_std] //  Don't use the Rust standard library. We are building a binary that can run on its own.

extern crate cortex_m; //  Low-level functions for ARM Cortex-M3 processor in STM32 Blue Pill.
#[macro_use(entry, exception)] //  Import macros from the following crates,
extern crate cortex_m_rt; //  Startup and runtime functions for ARM Cortex-M3.
extern crate arrayvec;
extern crate cortex_m_semihosting; //  Debug console functions for ARM Cortex-M3.
extern crate drs_0x01;
extern crate embedded_hal;
extern crate librobot;
#[macro_use]
extern crate nb;
extern crate heapless;
extern crate numtoa;
extern crate panic_semihosting; //  Panic reporting functions, which transmit to the debug console.
extern crate stm32f103xx_hal as f103_hal; //  Hardware Abstraction Layer (HAL) for STM32 Blue Pill.
extern crate w5500;

mod robot;

// ------ Cortex | F103 imports
use cortex_m::asm;
use cortex_m::Peripherals as CortexPeripherals;
use cortex_m_rt::ExceptionFrame;
use cortex_m_semihosting::hio;
use f103::Peripherals;
use f103_hal::delay::Delay;
use f103_hal::prelude::*;
use f103_hal::stm32f103xx as f103;

// ------ Embedded HAL imports
use embedded_hal::serial::Write as EWrite;
use embedded_hal::spi::FullDuplex;

// ------ Library imports
use heapless::consts::U256;
use w5500::*;

use drs_0x01::addr::WritableRamAddr;
use drs_0x01::Servo as HServo;

use drs_0x01::*;

use librobot::transmission::servo::{Control, Servo};
use librobot::transmission::{Frame, Message, MessageKind};

// ------ Local imports
use robot::init_peripherals;

const SOCKET_UDP: Socket = Socket::Socket0;

entry!(main);

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
    let message = servo.ram_write(WritableRamAddr::AckPolicy(2));
    for b in message {
        let _ = block!(connection.write(b));
    }
}

fn init_eth<E: core::fmt::Debug>(eth: &mut W5500, spi: &mut FullDuplex<u8, Error = E>) {
    //eth.set_mode(spi,false, false, false, true).unwrap();
    // using a 'locally administered' MAC address
    eth.init(spi).expect("Failed to initialize w5500");
    eth.set_mode(spi, false, false, false, true).unwrap();
    eth.set_mac(spi, &MacAddress::new(0x02, 0x01, 0x02, 0x03, 0x04, 0x05))
        .unwrap();
    eth.set_ip(spi, &IpAddress::new(192, 168, 0, 222)).unwrap();
    eth.set_subnet(spi, &IpAddress::new(255, 255, 255, 0))
        .unwrap();
    eth.set_gateway(spi, &IpAddress::new(192, 168, 0, 254))
        .unwrap();
    eth.reset_interrupt(spi, SOCKET_UDP, Interrupt::Received)
        .expect("Failed ot reset interrupts for W5500");
    eth.listen_udp(spi, SOCKET_UDP, 51)
        .expect("Faild to listen to port 51");
}

fn main() -> ! {
    let chip = Peripherals::take().unwrap();
    let cortex = CortexPeripherals::take().unwrap();
    let mut _debug_out = hio::hstdout().unwrap();
    let mut robot = init_peripherals(chip, cortex);
    let mut eth = W5500::new(&mut robot.spi_eth, &mut robot.pb8);
    init_eth(&mut eth, &mut robot.spi_eth);
    //init_servos(&mut robot.servo_tx, &mut robot.delay);

    robot.delay.delay_ms(50u32);

    let mut buffer = [0; 2048];

    loop {
        robot.delay.delay_ms(50u32);
        robot.led.set_high();
        robot.delay.delay_ms(50u32);
        robot.led.set_low();

        /*
        if let Some((_, _, size)) = eth
            .try_receive_udp(&mut robot.spi_eth, SOCKET_UDP, &mut buffer)
            .unwrap()
        {
            match Servo::from_json_slice(&buffer[0..size]) {
                Ok(servo) => {
                    //write!(debug_out, "{:?}", servo.to_string::<U256>().unwrap()).unwrap();
                    let s = HServo::new(servo.id);
                    let msg = match servo.control {
                        Control::Position => s.set_position(servo.data),
                        Control::Speed => s.set_speed(servo.data, Rotation::Clockwise),
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
        */
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
