#![no_std]
#![no_main]
// required to resolve linker error
extern crate cortex_m;

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use nrf52840_pac::Peripherals;
use panic_halt as _;
use rtt_target::{debug_rtt_init_print as rtt_init_print, debug_rprintln as rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let p = Peripherals::take().unwrap();
    p.P1.pin_cnf[10].write(|w| w.dir().output());
    let mut led_on: bool = false;
    loop {
        p.P1.out.write(|w| w.pin10().bit(led_on));
        for _ in 0..(1e6 as i32) { nop(); }
        led_on = !led_on;
    }
}
