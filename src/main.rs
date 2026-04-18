#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{debug, info, unwrap};
extern crate libm;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_nrf::mode::Async;
use embassy_nrf::peripherals::RNG;
use embassy_nrf::{Peri, bind_interrupts, i2s, rng};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::signal::Signal;
use embassy_time::Duration;
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
    I2S => i2s::InterruptHandler<embassy_nrf::peripherals::I2S>;
});

static IS_PLAYING: AtomicBool = AtomicBool::new(false);
// Kicked when IS_PLAYING transitions false→true so the i2s_task can wake from
// deep sleep. The i2s_task is genuinely suspended here (no DMA running), so the
// waker registration is always valid. The false→true transition uses AtomicBool
// directly in the DMA hot loop to avoid any waker dependency there.
static PLAY_NOTIFY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

type BleController = nrf_sdc::SoftdeviceController<'static>;

// Adafruit NUS UUID family: 6E40????-B5A3-F393-E0A9-E50E24DCCA9E
// B5A3 (not Nordic's B5A4) — Bluefruit Connect checks this to enable UART/Controller modules.
// Service = 0x0001, RX char = 0x0002, TX char = 0x0003.
const fn nus_uuid_le(svc_index: u16) -> [u8; 16] {
    let i = svc_index.to_le_bytes();
    [
        0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
        0x93, 0xF3, 0xA3, 0xB5, i[0], i[1], 0x40, 0x6E,
    ]
}

#[gatt_service(uuid = nus_uuid_le(0x0001))]
struct NordicUartService {
    #[characteristic(uuid = nus_uuid_le(0x0002), write, write_without_response)]
    rx: [u8; 20],
    #[characteristic(uuid = nus_uuid_le(0x0003), notify)]
    tx: [u8; 20],
}

#[gatt_server(connections_max = 1, mutex_type = NoopRawMutex)]
struct NusServer {
    nus: NordicUartService,
}

// Bluefruit Controller packet: !B<button><state><CRC>
// button: ASCII '1'-'8', state: '1' = pressed, '0' = released
fn handle_controller_packet(data: &[u8]) {
    if data.len() < 4 {
        debug!("BLE: short packet ({} bytes), ignoring", data.len());
        return;
    }
    if data[0] == b'!' && data[1] == b'B' && data[3] == b'1' {
        match data[2] {
            b'1' => { info!("BLE: play"); IS_PLAYING.store(true, Ordering::Relaxed); PLAY_NOTIFY.signal(()); }
            b'2' => { info!("BLE: pause"); IS_PLAYING.store(false, Ordering::Relaxed); }
            b'3' => info!("BLE: next"),
            b'4' => info!("BLE: previous"),
            _ => debug!("BLE: unknown button {:#x}", data[2]),
        }
    }
}

const TONE_HZ: f32 = 8000.0;
const TONE_DBFS: f32 = -3.0;
// 32 MHz / 21 / 32 = 47,619 Hz; round to nearest whole period.
const SAMPLE_RATE: f32 = 32_000_000.0 / 21.0 / 32.0;
const TONE_SAMPLES: usize = (SAMPLE_RATE / TONE_HZ + 0.5) as usize;

// Stereo i32: L in bits[31:16], R in bits[15:0]; both halves carry the same sample.
#[embassy_executor::task]
async fn i2s_task(
    i2s_periph: Peri<'static, embassy_nrf::peripherals::I2S>,
    mck: Peri<'static, embassy_nrf::peripherals::P0_12>,   // MCK output (free GPIO; change if occupied)
    sck: Peri<'static, embassy_nrf::peripherals::P0_14>,   // SCK / bit clock
    lrck: Peri<'static, embassy_nrf::peripherals::P0_13>,  // LRCK / WS
    sdout: Peri<'static, embassy_nrf::peripherals::P0_15>, // SD / serial data
) {
    let master_clock = i2s::MasterClock::new(i2s::MckFreq::_32MDiv21, i2s::Ratio::_32x);
    let config = i2s::Config::default(); // Stereo, Align::Left, Format::I2S (Philips)

    let driver = i2s::I2S::new_master(i2s_periph, Irqs, mck, sck, lrck, master_clock, config);
    let mut stream = driver.output(sdout, i2s::DoubleBuffering::<i32, TONE_SAMPLES>::new());

    let amplitude = libm::powf(10.0_f32, TONE_DBFS / 20.0) * i16::MAX as f32;
    let fill_sine = |buf: &mut [i32]| {
        for (i, v) in buf.iter_mut().enumerate() {
            let s = libm::sinf(2.0 * core::f32::consts::PI * i as f32 / TONE_SAMPLES as f32);
            let w = (s * amplitude) as i16 as u16 as u32;
            *v = (w << 16 | w) as i32;
        }
    };

    loop {
        // DMA is stopped here — MCU can sleep between BLE events.
        // PLAY_NOTIFY is only signaled on false→true, and the task is genuinely
        // suspended (Poll::Pending), so the waker is always registered correctly.
        info!("I2S: waiting (DMA stopped)");
        loop {
            PLAY_NOTIFY.wait().await;
            if IS_PLAYING.load(Ordering::Relaxed) { break; }
        }

        info!("I2S: starting");
        fill_sine(stream.buffer());
        unwrap!(stream.start().await);
        fill_sine(stream.buffer()); // both buffers identical; TXPTRUPD timing irrelevant

        // Playing loop. IS_PLAYING is an AtomicBool read — no waker registration,
        // no risk of the signal being lost if send() returns Poll::Ready immediately.
        loop {
            unwrap!(stream.send().await);
            if !IS_PLAYING.load(Ordering::Relaxed) {
                info!("I2S: stopping");
                stream.stop().await;
                info!("I2S: stopped");
                break;
            }
        }
    }
}

fn build_advertisement() -> (AdvertisementParameters, Advertisement<'static>) {
    let mut params = AdvertisementParameters::default();
    params.interval_min = Duration::from_millis(250);
    params.interval_max = Duration::from_millis(250);

    // NUS service UUID must be in the primary advertisement (not scan response) so iOS passive
    // scanning picks it up — Bluefruit Connect uses this to decide which modules to offer.
    static ADV_BUF: StaticCell<[u8; 31]> = StaticCell::new();
    let adv_buf = ADV_BUF.init([0u8; 31]);
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteServiceUuids128(&[nus_uuid_le(0x0001)]),
            // Adafruit company ID (0x0822) — required for Bluefruit Connect to show UART/Controller modules.
            AdStructure::ManufacturerSpecificData { company_identifier: 0x0822, payload: &[] },
        ],
        adv_buf,
    )
    .unwrap();

    // Device name in scan response — fetched on active scan, not needed for module detection.
    static SCAN_BUF: StaticCell<[u8; 31]> = StaticCell::new();
    let scan_buf = SCAN_BUF.init([0u8; 31]);
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(b"RustyFeather")],
        scan_buf,
    )
    .unwrap();

    (
        params,
        Advertisement::ConnectableScannableUndirected {
            adv_data: &adv_buf[..adv_len],
            scan_data: &scan_buf[..scan_len],
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

    const SDC_MEM_SIZE: usize = 1424;
    static SDC_MEM: StaticCell<sdc::Mem<SDC_MEM_SIZE>> = StaticCell::new();
    let sdc_mem = SDC_MEM.init(sdc::Mem::new());

    let sdc_controller = unwrap!(
        sdc::Builder::new()
            .unwrap()
            .support_adv()
            .support_peripheral()
            .build(sdc_p, rng_driver, mpsl_layer, sdc_mem)
    );

    // Derive unique random static address from FICR device ID.
    // Top 2 bits of last byte must be 0b11 per BLE spec (Vol 6, Part B, §1.3.2.1).
    let ficr_lo = unsafe { (0x1000_0060 as *const u32).read_volatile() };
    let ficr_hi = unsafe { (0x1000_0064 as *const u32).read_volatile() };
    let ble_address = [
        ficr_lo as u8,
        (ficr_lo >> 8) as u8,
        (ficr_lo >> 16) as u8,
        (ficr_lo >> 24) as u8,
        ficr_hi as u8,
        (ficr_hi >> 8) as u8 | 0xC0,
    ];

    static RESOURCES: StaticCell<HostResources<BleController, DefaultPacketPool, 1, 1>> =
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

    let mpsl_p = mpsl::Peripherals::new(
        p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31,
    );
    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21,
        p.PPI_CH22, p.PPI_CH23, p.PPI_CH24, p.PPI_CH25,
        p.PPI_CH26, p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );
    let (mut ble_runner, mut ble_peripheral) = init_ble_stack(&spawner, mpsl_p, sdc_p, p.RNG);

    spawner.spawn(unwrap!(i2s_task(
        p.I2S,
        p.P0_12, // MCK — free GPIO output; change if pin is occupied
        p.P0_14, // SCK
        p.P0_13, // LRCK / WS
        p.P0_15, // SDOUT / SD
    )));

    info!("Starting...");

    let _ = join(ble_runner.run(), async {
        let server = unwrap!(NusServer::new_default("RustyFeather"));
        let (adv_params, advertisement) = build_advertisement();
        loop {
            info!("Advertising...");
            let advertiser = unwrap!(ble_peripheral.advertise(&adv_params, advertisement).await);
            let conn = unwrap!(advertiser.accept().await)
                .with_attribute_server(&server)
                .unwrap();
            info!("Connected");
            loop {
                match conn.next().await {
                    GattConnectionEvent::Disconnected { reason } => {
                        info!("BLE: disconnected, reason={:?}", reason);
                        break;
                    }
                    GattConnectionEvent::Gatt { event } => {
                        if let GattEvent::Write(e) = event {
                            handle_controller_packet(e.data());
                            unwrap!(e.accept()).send().await;
                        }
                    }
                    _ => {}
                }
            }
        }
    })
    .await;
}
