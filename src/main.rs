#![no_std]
#![no_main]

use cortex_m as _;
use cortex_m::asm::nop;
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use hal::pac::Peripherals;
use nrf52840_hal::{self as hal, gpio::Level};
use panic_halt as _;

#[entry]
fn main() -> ! {
    let p = Peripherals::take().unwrap();
    let port1 = hal::gpio::p1::Parts::new(p.P1);
    let mut led = port1.p1_10.into_push_pull_output(Level::Low);
    loop {
        let new_state = led.is_set_low().unwrap();
        info!("Turning {}...", new_state.then_some("on").unwrap_or("off"));
        led.set_state(new_state.into()).unwrap();
        for _ in 0..(5e6 as i32) {
            nop();
        }
    }
}
