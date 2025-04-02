#![no_std]
#![no_main]
// required to resolve linker error
extern crate cortex_m;

use core::{ops::{BitOr, Shl, Sub}, ptr::{read_volatile, write_volatile}};

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use panic_halt as _;
use rtt_target::{debug_rtt_init_print as rtt_init_print, debug_rprintln as rprintln};

fn reset_at_pos<T, U>(val: T, pos: U) -> T where
T: BitOr<T, Output = T> + Sub<T, Output = T> + From<u32> + Shl<U, Output = T> + Copy
{
    let resetter: T = <u32 as Into<T>>::into(1u32) << pos;
    (val | resetter) - resetter
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    // page 324
    // gpio port 1: 0x50000300
    // cnf[10]: offset 0x728
    const GPIO1_P10_CNF_ADDR: * mut u32 = 0x5000_0A28 as * mut u32;
    const PINCNF_POS_DIR: u32 = 0;
    const PINCNF_DIR_VAL_OUT: u32 = 1;
    const GPIO1_P10_CNF_VAL: u32 = PINCNF_DIR_VAL_OUT << PINCNF_POS_DIR;
    rprintln!(
        "Configuring {} @ {:#x} = {:#b}",
        stringify!(GPIO1_P10_CNF_VAL),
        GPIO1_P10_CNF_ADDR as u32,
        GPIO1_P10_CNF_VAL,
    );
    unsafe {
        write_volatile(GPIO1_P10_CNF_ADDR, GPIO1_P10_CNF_VAL);
    }
    const GPIO1_OUT_ADDR: * mut u32 = 0x5000_0804 as * mut u32;
    const GPIO1_OUT_LED_POS: u32 = 10;
    let mut led_on: bool = false;
    loop {
        unsafe {
            let old = read_volatile(GPIO1_OUT_ADDR);
            let new = reset_at_pos(old, GPIO1_OUT_LED_POS) + ((led_on as u32) << GPIO1_OUT_LED_POS);
            write_volatile(GPIO1_OUT_ADDR, new);
        }
        for _ in 0..(1e6 as i32) { nop(); }
        led_on = !led_on;
    }
}
