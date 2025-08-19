#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, Instant};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::Blocking;
use esp_hal::{
    timer::timg::TimerGroup,
    uart::*,
};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use static_cell::StaticCell;

static UART_CELL: StaticCell<Mutex<RawMutex, Uart<'static, Blocking>>> = StaticCell::new();
static ACTIVE_CELL: StaticCell<Mutex<RawMutex, bool>> = StaticCell::new();
static LAST_SEEN_CELL: StaticCell<Mutex<RawMutex, Instant>> = StaticCell::new();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let uart = Uart::new(
        peripherals.UART0,
        Config::default().with_baudrate(115200),
    )
    .unwrap()
    .with_rx(peripherals.GPIO3)
    .with_tx(peripherals.GPIO1);

    let mut led = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    let uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>> =
        UART_CELL.init(Mutex::new(uart));

    // shared state
    let active_flag: &'static Mutex<RawMutex, bool> =
        ACTIVE_CELL.init(Mutex::new(false));

    let last_seen: &'static Mutex<RawMutex, Instant> =
        LAST_SEEN_CELL.init(Mutex::new(Instant::now()));

    // spawn tasks
    spawner.spawn(uart_task(uart_mutex, active_flag)).unwrap();
    spawner.spawn(watchdog_task(active_flag, last_seen)).unwrap();

    loop {
        let mut buf = [0u8; 1];
        {
            let mut uart_guard = uart_mutex.lock().await;
            if uart_guard.read_buffered(&mut buf).is_ok() {
                match buf[0] {
                    0x01 => {
                        *active_flag.lock().await = true;
                        *last_seen.lock().await = Instant::now(); // cập nhật timestamp
                        uart_guard.write(b"SENSOR_BOARD_OK\n").ok();
                    }
                    0xA1 => {
                        if *active_flag.lock().await {
                            led.set_high();
                        }
                    }
                    0xA2 => {
                        if *active_flag.lock().await {
                            led.set_low();
                        }
                    }
                    _ => {}
                }
            }
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embassy_executor::task]
async fn uart_task(
    uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>>,
    active_flag: &'static Mutex<RawMutex, bool>,
) {
    let mut counter: i32 = 0;

    loop {
        {
            let mut uart_guard = uart_mutex.lock().await;
            if *active_flag.lock().await {
                let bytes = counter.to_be_bytes();
                uart_guard.write(&bytes).ok();
                counter += 1;
            }
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn watchdog_task(
    active_flag: &'static Mutex<RawMutex, bool>,
    last_seen: &'static Mutex<RawMutex, Instant>,
) {
    loop {
        Timer::after(Duration::from_millis(100)).await;

        let elapsed = Instant::now() - *last_seen.lock().await;
        if elapsed >= Duration::from_secs(10) {
            // hết hạn lease → disable
            *active_flag.lock().await = false;
        }
    }
}
