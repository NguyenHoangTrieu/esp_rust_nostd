#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::*,
    timer::*,
    timer::timg::TimerGroup,
    time::Duration,
    uart::*,
    main,
};
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    // Set GPIO7 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    let mut uart0 = Uart::new(
        peripherals.UART0,
        Config::default().with_baudrate(115200),
    )
    .unwrap()
    .with_rx(peripherals.GPIO3)
    .with_tx(peripherals.GPIO1);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0 = timg0.timer0;
    timer0.load_value(Duration::from_secs(1)).unwrap();
    timer0.start();
    let mut counter: i32 = 0;
    loop {
        let mut buf = [0u8; 1];
        let _n = uart0.read_buffered(&mut buf);
        match buf[0] {
            0xA1 => {
                led.set_high();
            }
            0xA2 => {
                led.set_low();
            }
            _ => {}
        }
        if timer0.is_interrupt_set(){
            led.toggle();
            timer0.clear_interrupt();
            let bytes = counter.to_be_bytes();
            let _ = uart0.write(&bytes);
            counter += 1;
        }
    }
}