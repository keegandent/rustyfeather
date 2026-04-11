#![no_std]
#![no_main]

use core::mem;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::{main, Spawner};
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::interrupt::Priority;
use embassy_time::Timer;
use nrf_softdevice::{raw, Softdevice};
use panic_probe as _;

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[main]
async fn main(spawner: Spawner) -> ! {
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.gpiote_interrupt_priority = Priority::P2;
    nrf_config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(nrf_config);

    let config = nrf_softdevice::Config {
        ..Default::default()
    };
    let sd = Softdevice::enable(&config);
    spawner.spawn(softdevice_task(sd)).unwrap();

    info!("Starting...");
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
