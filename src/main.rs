#![no_std]
#![no_main]
// required to resolve linker error
extern crate cortex_m;

use cortex_m_rt::entry;
use adafruit_nrf52840_express::{Board, prelude::*};
use panic_halt as _;
use rtt_target::{debug_rtt_init_print as rtt_init_print, debug_rprintln as rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::new().unwrap();

    let mut delay = board.delay;

    let mut led = board.blue_led;
    loop {
        // can technically be accomplished with //.toggle().unwrap() but nice to see other methods
        let new_state = led.is_set_low().unwrap();
        led.set_state(new_state.into()).unwrap();
        delay.delay_ms(500u16);
    }
}
