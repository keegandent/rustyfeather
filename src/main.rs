#![no_std]
#![no_main]
// required to resolve linker error
extern crate cortex_m;

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use embedded_hal::digital::{StatefulOutputPin, OutputPin};
use hal::pac::Peripherals;
use nrf52840_hal::{self as hal, gpio::Level};
use panic_halt as _;
use rtt_target::{debug_rtt_init_print as rtt_init_print, debug_rprintln as rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let p = Peripherals::take().unwrap();
    let port1 = hal::gpio::p1::Parts::new(p.P1);
    let mut led = port1.p1_10.into_push_pull_output(Level::Low);
    loop {
        // can technically be accomplished with //.toggle().unwrap() but nice to see other methods
        let new_state = led.is_set_low().unwrap();
        led.set_state(new_state.into()).unwrap();
        for _ in 0..(1e6 as i32) { nop(); }
    }
}
