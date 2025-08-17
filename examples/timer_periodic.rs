#![no_std]
#![no_main]

//use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    gpio::*,
    timer::*,
    timer::timg::TimerGroup,
    time::Duration,
    //handler, 
    main,
};
use esp_println::println;
//use core::cell::Cell;
//static FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    println!("Hello world!");
    let mut periodic = PeriodicTimer::new(timg0.timer0);
    periodic.start(Duration::from_secs(1)).unwrap();
    // Set GPIO7 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());

    // ANCHOR_END: critical_section
    loop {
       periodic.wait(); // use like delay, basicly it waits for the interrupt flag set and clean it
        led.toggle();
         println!("Timer interrupt");
    }
}
// #[handler]
// fn handler() {
//     critical_section::with(|cs| {
//         FLAG.borrow(cs).set(true);
//     });
// }
