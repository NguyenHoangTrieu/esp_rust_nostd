#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{rmt::Rmt, time::Rate, Config, delay::Delay};
use esp_hal_smartled::{smart_led_buffer, SmartLedsAdapter};
use smart_leds::{brightness, colors::*, SmartLedsWrite as _};


#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(Config::default());
    let mut led = {
        let frequency = Rate::from_mhz(80);
        let rmt = Rmt::new(p.RMT, frequency).expect("Failed to initialize RMT0");
        SmartLedsAdapter::new(rmt.channel0, p.GPIO2, smart_led_buffer!(1))
    };
    let level = 10;
    led.write(brightness([RED].into_iter(), level)).unwrap();
    let delay = Delay::new();
    loop {
        led.write(brightness([GREEN].into_iter(), level)).unwrap();
        delay.delay_millis(500);
        led.write(brightness([CYAN].into_iter(), level)).unwrap();
        delay.delay_millis(500);
        led.write(brightness([PINK].into_iter(), level)).unwrap();
        delay.delay_millis(500);
    } // loop forever
}