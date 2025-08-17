#![no_std]
#![no_main]

use esp_backtrace as _;
use nb;
use esp_hal::{
    gpio::*,
    timer::*,
    timer::timg::TimerGroup,
    time::Duration,
    analog::adc::*,
    //handler, 
    uart::*,
    main,
};
use esp_println::println;
use heapless::String;
use core::fmt::Write;
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
    let analog_pin = peripherals.GPIO2;
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(
    analog_pin,
    Attenuation::_11dB,
    );
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    //Config UART:
    let mut uart = Uart::new(
    peripherals.UART0,
    Config::default().with_baudrate(57600)).unwrap().with_rx(peripherals.GPIO20).with_tx(peripherals.GPIO21);
    loop {
        if timer0.is_interrupt_set(){
        let pin_value: u16 = nb::block!(adc1.read_oneshot(&mut pin)).unwrap();
        led.toggle();
        timer0.clear_interrupt();
         //println!("ADC Value {}", pin_value);
         let mut buf: String<32> = String::new(); // max 32 ký tự
        write!(buf, "ADC Value: {}\r\n", pin_value).unwrap();   
        uart.write(buf.as_bytes()).unwrap();
        }
    }
}