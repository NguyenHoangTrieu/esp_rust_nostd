#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    handler, main,
};
use esp_println::println;
use core::cell::Cell;

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static FLAG: Mutex<Cell<bool>> = Mutex::new(Cell::new(false)); // set global variable value.
#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    println!("Hello world!");

    let mut io = Io::new(peripherals.IO_MUX);
    // Set the interrupt handler for GPIO interrupts.
    io.set_interrupt_handler(handler);

    // Set GPIO7 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());

    // Set GPIO9 as an input
    let mut button = Input::new(peripherals.GPIO9, InputConfig::default().with_pull(Pull::Up));

    // ANCHOR: critical_section
    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });
    // ANCHOR_END: critical_section

    let delay = Delay::new();
    loop {
        if critical_section::with(|cs| FLAG.borrow(cs).get()){
        led.toggle();
        critical_section::with(|cs| {
            FLAG.borrow(cs).set(false);
        });
        }
    }
}

#[handler]
fn handler() {
    critical_section::with(|cs| {
        println!("GPIO interrupt");
        FLAG.borrow(cs).set(true);
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
