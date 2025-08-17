#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
	main,
	i2c::master::*,
	delay::Delay,
	time::{Rate},
};
mod mpu6050_lib{
	pub mod lib;
	pub mod mpu;
	pub mod bits;
}
use esp_println::println;
use mpu6050_lib::lib::*;
use micromath::vector::{Vector2d, Vector3d};
#[main]
fn main() -> ! {
	let peripherals = esp_hal::init(esp_hal::Config::default());
	let i2c = I2c::new(
		peripherals.I2C0,
		Config::default().with_frequency(Rate::from_khz(100)),
	).unwrap().with_sda(peripherals.GPIO8).with_scl(peripherals.GPIO9);
	let mut mpu = Mpu6050::new(i2c);
	  let mut delay = Delay::new();
	  mpu.init(&mut delay).unwrap();
	//let interface = I2CDisplayInterface::new(i2c);
	let delay = Delay::new();
	loop{
		let acc = mpu.get_acc_angles().unwrap();
        let Vector2d {x:rp_x, y: rp_y} = acc;
		println!("r/p: x:{}, y:{}", rp_x, rp_y);

		// get temp
		let temp = mpu.get_temp().unwrap();
		println!("temp: {:?}c", temp);

		// get gyro data, scaled with sensitivity 
		let gyro = mpu.get_gyro().unwrap();
        let Vector3d { x: gyro_x, y: gyro_y, z: gyro_z } = gyro;
		println!("gyro: x:{}, y:{}, z:{}", gyro_x, gyro_y, gyro_z);

		// get accelerometer data, scaled with sensitivity
		let acc = mpu.get_acc().unwrap();
        let Vector3d { x: acc_x, y: acc_y, z: acc_z } = acc;
		println!("acc: x:{}, y:{}, z:{}", acc_x, acc_y, acc_z);
		delay.delay_millis(500);
	}
}
