#![no_std]
#![no_main]
use core::time::Duration;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay, gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull, WakeEvent}, 
    rtc_cntl::{reset_reason, sleep::*, wakeup_cause, Rtc}, system::Cpu};
use esp_println::println;

#[esp_hal::main]
fn main() -> ! {
    let delay = Delay::new();
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut rtc = Rtc::new(peripherals.LPWR);
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    led.set_high();
    let timer = TimerWakeupSource::new(Duration::from_secs(10));
    let mut button1 = Input::new(peripherals.GPIO9, InputConfig::default().with_pull(Pull::Up));
    button1.wakeup_enable(true, WakeEvent::LowLevel).unwrap();
    let button = GpioWakeupSource::new();
    delay.delay_millis(100);
    rtc.sleep_light(&[&timer, &button]);
    loop{
        led.toggle();
        //println!("WTF?????");
        delay.delay_millis(1000);
    }
}