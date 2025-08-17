#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
	main,
	i2c::master::*,
	delay::Delay,
	time::{Rate},
};
use embedded_graphics::{
	 mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
	 pixelcolor::BinaryColor,
	 prelude::*,
	 text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
#[main]
fn main() -> ! {
	let peripherals = esp_hal::init(esp_hal::Config::default());
	let i2c = I2c::new(
		peripherals.I2C0,
		Config::default().with_frequency(Rate::from_khz(100)),
	).unwrap().with_sda(peripherals.GPIO8).with_scl(peripherals.GPIO9);

	let interface = I2CDisplayInterface::new(i2c);
	let mut display = Ssd1306::new(
		interface,
		DisplaySize128x64,
		DisplayRotation::Rotate0,
	).into_buffered_graphics_mode();

	display.init().unwrap();
	let text_style = MonoTextStyleBuilder::new()
		.font(&FONT_6X10)
		.text_color(BinaryColor::On)
		.build();

	Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
    .draw(&mut display)
    .unwrap();

	Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
    .draw(&mut display)
    .unwrap();

	display.flush().unwrap();
	let delay = Delay::new();
	loop{
		delay.delay_millis(1000);
	}
}
