#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
	delay::Delay, main,
	spi::master::*,
	time::Rate,
	spi::Mode,
	gpio::{Level, Output, OutputConfig},
	ledc::{
        channel::ChannelIFace,
		channel,
        timer::{self},
        LSGlobalClkSource, LowSpeed, Ledc,
		timer::TimerIFace,
    },
};
#[main]
fn main() -> ! {
	let peripherals = esp_hal::init(esp_hal::Config::default());
	let mut ledc = Ledc::new(peripherals.LEDC);
	ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
	let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
	lstimer0
    .configure(timer::config::Config {
        duty: timer::config::Duty::Duty14Bit,
        clock_source: timer::LSClockSource::APBClk,
        frequency: Rate::from_hz(50),
    }).unwrap();
	let led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
	let mut channel0 = ledc.channel(channel::Number::Channel0, led);
	channel0
    .configure(channel::config::Config {
        timer: &lstimer0,
        duty_pct: 10,
        pin_config: channel::config::PinConfig::PushPull,
    }).unwrap();
	let delay = Delay::new();
	loop{
		// channel0.start_duty_fade(1, 100, 20).unwrap();
    	// while channel0.is_duty_fade_running() {}
    	// channel0.start_duty_fade(10, 1, 20).unwrap();
    	// while channel0.is_duty_fade_running() {}
		//delay.delay_millis(1000);
		let _ = channel0.set_duty(3);
		delay.delay_millis(1000);
		let _ = channel0.set_duty(13);
		delay.delay_millis(1000);
	}
}
