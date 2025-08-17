#![no_std]
#![no_main]

use esp_backtrace as _;
use core::fmt::Write; // dùng cho write! macro
use heapless::String; // dùng heapless nếu không có std
use esp_hal::{
	analog::adc::*, delay::Delay, i2c::master::*, main, peripherals::GPIO, time::Rate
};
use embedded_graphics::{
	 mono_font::{ascii::FONT_6X10, iso_8859_1::FONT_10X20, MonoTextStyleBuilder},
	 pixelcolor::BinaryColor,
	 //image::{Image, ImageRaw},
	 prelude::*,
	 text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
mod mod_lib{
	pub mod image;
	pub mod joystick;
}
use mod_lib::joystick::*;

use crate::mod_lib::joystick::get_direct;
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
		.font(&FONT_10X20)
		.text_color(BinaryColor::On)
		.build();

	Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
    .draw(&mut display)
    .unwrap();

	Text::with_baseline("Hello Rust!", Point::new(0, 15), text_style, Baseline::Top)
    .draw(&mut display)
    .unwrap();
	display.flush().unwrap();
	
	let delay = Delay::new();
	delay.delay_millis(1945);
	

	let x_axis = peripherals.GPIO0;
	let y_axis = peripherals.GPIO1;
    let mut adc1_config = AdcConfig::new();
    let mut pin_x = adc1_config.enable_pin(
    x_axis,
    Attenuation::_11dB,
    );
	let mut pin_y = adc1_config.enable_pin(
    y_axis,
    Attenuation::_11dB,
    );
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);
	loop{
		display.clear(BinaryColor::Off).unwrap();
		let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut pin_x)).unwrap();
		let mut adc_str: String<32> = String::new(); // 32 là số ký tự tối đa
   		write!(adc_str, "ADC_X: {}", pin_value).unwrap();

		Text::with_baseline(&adc_str, Point::new(0, 0), text_style, Baseline::Top)
    	.draw(&mut display)
    	.unwrap();

		let pin_value_2: u16 = nb::block!(adc1.read_oneshot(&mut pin_y)).unwrap();
		let mut adc_str_2: String<32> = String::new(); // 32 là số ký tự tối đa
   		write!(adc_str_2, "ADC_Y: {}", pin_value_2).unwrap();

		Text::with_baseline(&adc_str_2, Point::new(0, 15), text_style, Baseline::Top)
    	.draw(&mut display)
    	.unwrap();
		let direct = get_direct(pin_value, pin_value_2);
		let mut buf: String<32> = String::new(); // 32 là số ký tự tối đa
   		write!(buf, "DIRECT: {}", direct).unwrap();

		Text::with_baseline(&buf, Point::new(0, 30), text_style, Baseline::Top)
    	.draw(&mut display)
    	.unwrap();

		display.flush().unwrap();
		//delay.delay_millis(1000);
	}
}
