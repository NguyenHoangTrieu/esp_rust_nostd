#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::Blocking;
use esp_hal::{
    timer::timg::TimerGroup,
    //handler, 
    uart::*,
};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use static_cell::StaticCell;

static UART_CELL: StaticCell<Mutex<RawMutex, Uart<'static, Blocking>>> = StaticCell::new();


#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.3.1

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
    spawner.spawn(uart_task(uart_mutex)).unwrap();

    loop {
        let mut buf = [0u8; 64];
        {
            let mut uart_guard = uart_mutex.lock().await;
            let _ = uart_guard.read_buffered(&mut buf);
        }
        match buf[0] {
            0xA1 => led.set_high(),
            0xA2 => led.set_low(),
            _ => {}
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embassy_executor::task]
async fn uart_task(uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>>) {
    let mut counter: i32 = 0;

    loop {
        let bytes = counter.to_be_bytes();

        {
            // Chỉ giữ lock trong lúc ghi
            let mut uart_guard = uart_mutex.lock().await;
            let _ = uart_guard.write(&bytes);
        } // thả lock

        counter += 1;
        Timer::after(Duration::from_secs(1)).await;
    }
}