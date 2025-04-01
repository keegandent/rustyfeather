#![no_std]
#![no_main]
// required to resolve linker error
extern crate cortex_m;

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{debug_rtt_init_print as rtt_init_print, debug_rprintln as rprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    loop {
        rprintln!("Hello, world!");
        for _ in 0..(1e6 as i32) { nop(); }
    }
}
