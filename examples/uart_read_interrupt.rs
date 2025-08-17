#![no_std]
#![no_main]

use esp_backtrace as _;
use core::cell::RefCell;
use core::cell::Cell;
use critical_section::Mutex;
use esp_hal::{
    handler, 
    main,
    uart::*, Blocking,
};
use esp_println::println;
static SERIAL: Mutex<RefCell<Option<Uart<Blocking>>>> = Mutex::new(RefCell::new(None));
static FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut uart0 = Uart::new( peripherals.UART0,
    Config::default().with_baudrate(57600)).unwrap().with_rx(peripherals.GPIO20).with_tx(peripherals.GPIO21);
    uart0.set_interrupt_handler(interrupt_handler);
    critical_section::with(|cs| {
        uart0.set_at_cmd(AtCmdConfig::default().with_cmd_char(b'#'));
        uart0.listen(UartInterrupt::AtCmd | UartInterrupt::RxTimeout);
        SERIAL.borrow_ref_mut(cs).replace(uart0);
    });
    // Set GPIO7 as an output, and set its state high initially.
    //Config UART:
    loop {
        //println!("Send `#` character or >=30 characters");
        if critical_section::with(|cs| FLAG.borrow(cs).get()){
            critical_section::with(|cs| {
            let mut serial = SERIAL.borrow_ref_mut(cs);
            if let Some(serial) = serial.as_mut() {
                let mut buf = [0u8; 64];
                if let Ok(cnt) = serial.read(&mut buf) {
                    println!("Read {} bytes", cnt);
                    println!("As bytes: {:?}", &buf[..cnt]);
                    let len = buf[..cnt].iter().position(|&b| b == 0).unwrap_or(cnt);
                    let sub = &buf[..len];

                    if let Ok(s) = core::str::from_utf8(sub) {
                        println!("As string: {}", s);
                    } else {
                        println!("Invalid UTF-8 string");
                    }
                }
                FLAG.borrow(cs).set(false);
                serial.listen(UartInterrupt::AtCmd | UartInterrupt::RxTimeout);
            }
            });
        }
    }
}

#[handler]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut serial = SERIAL.borrow_ref_mut(cs);
        if let Some(serial) = serial.as_mut() {
            FLAG.borrow(cs).set(true);
            let pending_interrupts = serial.interrupts();
            println!(
                "Interrupt AT-CMD: {} RX-Timeout: {}",
                pending_interrupts.contains(UartInterrupt::AtCmd),
                pending_interrupts.contains(UartInterrupt::RxTimeout),
            );
            serial.clear_interrupts(
                    UartInterrupt::AtCmd | UartInterrupt::RxTimeout
            );
            serial.unlisten(UartInterrupt::AtCmd | UartInterrupt::RxTimeout);
        }
    });
}