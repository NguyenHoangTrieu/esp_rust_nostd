#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    time,
    timer::{Timer, timg},
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    handler, main,
    interrupt,
    peripherals::{Interrupt, TIMG0},
    // Interrupt,
};
// use esp_hal::interrupt::{self as intr, interrupt, Interrupt};
use esp_println::println;

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));
static INTERRUPT_FLAG: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static TIMER_FLAG: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

static TIMER: Mutex<RefCell<Option<timg::Timer>>> = Mutex::new(RefCell::new(None));

// static mut interrupt_flag: bool = false;

#[main]
fn main() -> ! {
    
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = timg::TimerGroup::new(peripherals.TIMG0);
    let timer0 = timg0.timer0;

    // Get the current timestamp, in microseconds:
    // let now = timer0.now();

    // Wait for timeout:
    let _ = timer0.load_value(time::Duration::from_millis(1000));
    timer0.enable_interrupt(true);              // Enable interrupt
    // timer0.enable_auto_reload(true);        // Auto-reload on expiry
    timer0.start();

    timer0.set_interrupt_handler(timer_interrupt_handler);

    // Enable timer interrupt in the interrupt controller
    critical_section::with(|cs|{
        TIMER.borrow_ref_mut(cs).replace(timer0);
    });

    println!("Hello world!");

    let mut io = Io::new(peripherals.IO_MUX);
    // Set the interrupt handler for GPIO interrupts.
    io.set_interrupt_handler(handler_button);

    // Set GPIO8 as an output, and set its state high initially.
    let mut led = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());

    // Set GPIO9 as an input
    let mut button = Input::new(peripherals.GPIO9, InputConfig::default().with_pull(Pull::Up));

    // ANCHOR: critical_section
    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });
    // ANCHOR_END: critical_section

    // let delay = Delay::new();
    loop {

        let triggered = critical_section::with(|cs| {
            let mut flag = INTERRUPT_FLAG.borrow_ref_mut(cs);
            let current = *flag;
            *flag = false; // Clear it
            current
        });

        let fired = critical_section::with(|cs| {
            let mut flag = TIMER_FLAG.borrow_ref_mut(cs);
            let was_fired = *flag;
            *flag = false;
            // let _ = timer0.load_value(time::Duration::from_millis(1000));
            // timer0.start();

            was_fired
        });

        if triggered || fired {
            led.toggle();
        }
            
        // delay.delay_millis(10u32);
    }
}

#[handler]
fn handler_button() {
    critical_section::with(|cs| {
        println!("GPIO interrupt");

    // ANCHOR: critical_section
        // Set the interrupt flag
        *INTERRUPT_FLAG.borrow_ref_mut(cs) = true;
    // ANCHOR_END: critical_section

        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}

#[handler]
// #[interrupt]
fn timer_interrupt_handler() {

    critical_section::with(|cs| {
        println!("timer interrupt");

        *TIMER_FLAG.borrow_ref_mut(cs) = true;

        TIMER
        .borrow_ref_mut(cs)
        .as_mut()
        .unwrap()
        .clear_interrupt();
    });

    // let timg0 = unsafe { &*TIMG0::ptr() };
    // timg0.clear_interrupt();
    // timg0.int_clr_timers.write(|w| w.t0().set_bit()); // Clear interrupt flag
}