#![no_std]
#![no_main]
// required to resolve linker error
extern crate cortex_m;

use cortex_m_rt::entry;
use adafruit_nrf52840_express::{Board, prelude::*, hal::{pwm::Channel, gpio}};
use panic_halt as _;
use rtt_target::{debug_rtt_init_print as rtt_init_print, debug_rprintln as rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::new().unwrap();

    let mut delay = board.delay;

    let mut led = board.blue_led;
    let servo_pin = board.d5.into_push_pull_output(gpio::Level::Low);
    let servo_pwm = board.pwm0;
    servo_pwm.set_period(50u32.hz());
    servo_pwm.set_output_pin(Channel::C0, servo_pin.degrade());
    loop {
        let new_state = led.is_set_low().unwrap();
        led.set_state(new_state.into()).unwrap();
        if new_state {
            // for duty in 0..servo_pwm.get_max_duty() {
            servo_pwm.set_duty_on_common(servo_pwm.get_max_duty() / 2);
            //     delay.delay_ms(50u16);
            // }
        } else {
            servo_pwm.set_duty_on_common(servo_pwm.get_max_duty() / 8);
        }
        delay.delay_ms(500u16);
    }
}
