#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
	delay::Delay, main,
	gpio::{Level, Output, OutputConfig},
};
mod mod_lib{
	pub mod step_motor;
}
use mod_lib::step_motor::*;
#[main]
fn main() -> ! {
	let peripherals = esp_hal::init(esp_hal::Config::default());
	let pin_1 = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());
	let pin_2 = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
	let pin_3 = Output::new(peripherals.GPIO20, Level::Low, OutputConfig::default());
	let pin_4 = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());
	let mut delay = Delay::new();
	let mut stepper = Stepper::new(pin_1, pin_2, pin_3, pin_4, & mut delay);
	let delay2 = Delay::new();
	loop{
		stepper.step_backward(100, 5);
		delay2.delay_millis(500);
		stepper.step_forward(200, 5);
	}
}
