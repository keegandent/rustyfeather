#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::{main, Spawner};
use embassy_nrf::gpio::{Input, Pull};
use panic_probe as _;

#[main]
async fn main(_spawner: Spawner) -> ! {
    info!("Starting...");
    let p = embassy_nrf::init(Default::default());
    let mut button = Input::new(p.P1_02, Pull::Up);
    loop {
        button.wait_for_low().await;
        info!("Button pressed!");
        button.wait_for_high().await;
        info!("Button released.");
    }
}