#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::{main, Spawner};
use embassy_nrf::gpio::{Input, Pull};
use embassy_time::Timer;
use panic_probe as _;

#[main]
async fn main(_spawner: Spawner) -> ! {
    info!("Starting...");
    let p = embassy_nrf::init(Default::default());
    let mut button = Input::new(p.P1_02, Pull::Up);
    let debounce = async || Timer::after_millis(50).await;
    loop {
        button.wait_for_low().await;
        info!("Button pressed!");
        debounce().await;
        button.wait_for_high().await;
        info!("Button released.");
        debounce().await;
    }
}