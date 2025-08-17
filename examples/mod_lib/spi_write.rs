#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
	delay::Delay, main,
	spi::master::*,
	time::Rate,
	spi::Mode,
	gpio::{Level, Output, OutputConfig},
};
#[main]
fn main() -> ! {
	let peripherals = esp_hal::init(esp_hal::Config::default());
	let mut led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
	let mut spi = Spi::new(
    peripherals.SPI2,
    Config::default()
        .with_frequency(Rate::from_khz(100))
        .with_mode(Mode::_0),
).unwrap()
.with_sck(peripherals.GPIO7)
.with_mosi(peripherals.GPIO6)
.with_miso(peripherals.GPIO5);
let delay = Delay::new();
let word = [1u8;100];
let mut read_word =[0u8, 100];
let _ = spi.write(&word);
	loop{
		let _ = spi.read(& mut read_word);
		led.toggle();
		delay.delay_millis(500);
	}
}
