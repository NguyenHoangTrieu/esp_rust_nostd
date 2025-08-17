#![no_std]
#![no_main]

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
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // Set GPIO7 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    println!("Hello world!");
    let timer0 = timg0.timer0;
    timer0.load_value(Duration::from_secs(1)).unwrap();
    timer0.start();
    loop {
        if timer0.is_interrupt_set(){
        led.toggle();
        timer0.clear_interrupt();
         println!("Timer interrupt");
        }
    }
}