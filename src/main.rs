#![no_std]
#![no_main]

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::gpio::{Input, Pull};
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{Peri, bind_interrupts, rng};
use embassy_time::{Duration, Timer};
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use nrf_sdc::{self as sdc, mpsl};
use panic_probe as _;
use static_cell::StaticCell;
use trouble_host::prelude::*;

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

type BleController = nrf_sdc::SoftdeviceController<'static>;

fn build_advertisement() -> (AdvertisementParameters, Advertisement<'static>) {
    let mut params = AdvertisementParameters::default();
    params.interval_min = Duration::from_millis(250);
    params.interval_max = Duration::from_millis(250);

    static ADV_BUF: StaticCell<[u8; 31]> = StaticCell::new();
    let buf = ADV_BUF.init([0u8; 31]);
    let len = AdStructure::encode_slice(
        &[
            AdStructure::CompleteLocalName(b"RustyFeather"),
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
        ],
        buf,
    )
    .unwrap();

    (
        params,
        Advertisement::NonconnectableScannableUndirected {
            adv_data: &buf[..len],
            scan_data: &[],
        },
    )
}

fn init_ble_stack(
    spawner: &Spawner,
    mpsl_p: mpsl::Peripherals<'static>,
    sdc_p: sdc::Peripherals<'static>,
    rng_p: Peri<'static, RNG>,
) -> (
    Runner<'static, BleController, DefaultPacketPool>,
    Peripheral<'static, BleController, DefaultPacketPool>,
) {
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
    let mpsl_layer = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::new(
        mpsl_p, Irqs, lfclk_cfg
    )));
    spawner.spawn(unwrap!(mpsl_task(mpsl_layer)));

    static RNG: StaticCell<rng::Rng<'static, Async>> = StaticCell::new();
    let rng_driver = RNG.init(rng::Rng::new(rng_p, Irqs));

    // Minimum SDC memory for a single advertiser role; increase if adding connections.
    const SDC_MEM_SIZE: usize = 792;
    static SDC_MEM: StaticCell<sdc::Mem<SDC_MEM_SIZE>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::new());

    let sdc_controller = unwrap!(
        sdc::Builder::new()
            .unwrap()
            .support_adv()
            .build(sdc_p, rng_driver, mpsl_layer, sdc_mem)
    );

    // Derive a unique random static address from the chip's factory-programmed FICR device ID.
    // Top 2 bits of the last byte must be set per the BLE random static address spec (Vol 6, Part B, §1.3.2.1).
    let ficr_lo = unsafe { (0x1000_0060 as *const u32).read_volatile() };
    let ficr_hi = unsafe { (0x1000_0064 as *const u32).read_volatile() };
    let ble_address: [u8; 6] = [
        ficr_lo as u8,
        (ficr_lo >> 8) as u8,
        (ficr_lo >> 16) as u8,
        (ficr_lo >> 24) as u8,
        ficr_hi as u8,
        (ficr_hi >> 8) as u8 | 0xC0,
    ];
    static RESOURCES: StaticCell<HostResources<BleController, DefaultPacketPool, 0, 0>> =
        StaticCell::new();
    let resources = RESOURCES.init(HostResources::new());

    static STACK: StaticCell<Stack<'static, BleController, DefaultPacketPool>> = StaticCell::new();
    let stack = STACK.init(
        trouble_host::new(sdc_controller, resources)
            .set_random_address(Address::random(ble_address))
            .build(),
    );

    (stack.runner(), stack.peripheral())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p: embassy_nrf::Peripherals = embassy_nrf::init(Default::default());

    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24,
        p.PPI_CH25, p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );
    let (mut ble_runner, mut ble_peripheral) = init_ble_stack(&spawner, mpsl_p, sdc_p, p.RNG);

    info!("Starting...");

    let (adv_params, advertisement) = build_advertisement();

    let mut button = Input::new(p.P1_02, Pull::Up);
    let debounce = async || Timer::after_millis(50).await;

    let _ = join(ble_runner.run(), async {
        let _adv_handle = ble_peripheral
            .advertise(&adv_params, advertisement)
            .await
            .unwrap();
        loop {
            button.wait_for_low().await;
            info!("Button pressed!");
            debounce().await;
            button.wait_for_high().await;
            info!("Button released.");
            debounce().await;
        }
    })
    .await;
}
