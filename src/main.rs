#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::{Blocking};
use esp_hal::{
    timer::timg::TimerGroup,
    uart::*,
    delay::Delay,
    analog::adc::*,
};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use esp_println::print;
use static_cell::StaticCell;

// Static cells
static UART_CELL: StaticCell<Mutex<RawMutex, Uart<'static, Blocking>>> = StaticCell::new();

// GPIO Register - 8-bit register to manage 8 GPIO pins (GLOBAL ONLY)
static GPIO_REGISTER: StaticCell<Mutex<RawMutex, u8>> = StaticCell::new();
static mut ADC_REGISTER: StaticCell<Mutex<RawMutex, u16>> = StaticCell::new();
// Static cells for RS485 direction control pins
static RE_PIN_CELL: StaticCell<Mutex<RawMutex, Output<'static>>> = StaticCell::new();
static DE_PIN_CELL: StaticCell<Mutex<RawMutex, Output<'static>>> = StaticCell::new();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

mod mod_lib{
    pub mod modbus_handler;
}
use mod_lib::modbus_handler::*;

// Modbus node address
const NODE_ADDRESS: u8 = 0x03;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let uart = Uart::new(
        peripherals.UART1,
        Config::default().with_baudrate(115200),
    )
    .unwrap()
    .with_rx(peripherals.GPIO19)
    .with_tx(peripherals.GPIO21);

    // Initialize 8 GPIO outputs (LOCAL - avoid pins 20,21,22,23)
    // Using pins: 2,4,5,12,13,14,15,16
    let mut gpio0 = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());   // Bit 0
    let mut gpio1 = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());   // Bit 1
    let mut gpio2 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());   // Bit 2
    let mut gpio3 = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());  // Bit 3
    let mut gpio4 = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());  // Bit 4
    let mut gpio5 = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());  // Bit 5
    let mut gpio6 = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());  // Bit 6
    let mut gpio7 = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());  // Bit 7

    // Initialize RS485 direction control pins
    let re_pin = Output::new(peripherals.GPIO22, Level::Low, OutputConfig::default());   // Receiver Enable (active low)
    let de_pin = Output::new(peripherals.GPIO23, Level::Low, OutputConfig::default());   // Driver Enable (active high)
    // ADC pin set up:
        let analog_pin = peripherals.GPIO2;
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(
    analog_pin,
    Attenuation::_11dB,
    );
    // Initialize static cells
    let uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>> =
        UART_CELL.init(Mutex::new(uart));

    let gpio_register: &'static Mutex<RawMutex, u8> =
        GPIO_REGISTER.init(Mutex::new(0));

    // Initialize RS485 control pin mutexes
    let re_pin_mutex: &'static Mutex<RawMutex, Output<'static>> =
        RE_PIN_CELL.init(Mutex::new(re_pin));
    
    let de_pin_mutex: &'static Mutex<RawMutex, Output<'static>> =
        DE_PIN_CELL.init(Mutex::new(de_pin));

    // Set Modbus UART handle
    unsafe {
        crate::mod_lib::modbus_handler::MODBUS_UART_HANDLE = Some(uart_mutex);
    }

    // Spawn tasks with RE/DE pin parameters
    spawner.spawn(uart_task(uart_mutex, gpio_register, re_pin_mutex, de_pin_mutex)).unwrap();

    // MAIN TASK: Bidirectional GPIO <-> Register sync
    loop {
        // 1. Read GPIO states and save to register
        let mut register_value = 0u8;
        if gpio0.is_set_high() { register_value |= 1 << 0; }
        if gpio1.is_set_high() { register_value |= 1 << 1; }
        if gpio2.is_set_high() { register_value |= 1 << 2; }
        if gpio3.is_set_high() { register_value |= 1 << 3; }
        if gpio4.is_set_high() { register_value |= 1 << 4; }
        if gpio5.is_set_high() { register_value |= 1 << 5; }
        if gpio6.is_set_high() { register_value |= 1 << 6; }
        if gpio7.is_set_high() { register_value |= 1 << 7; }

        // Update register
        {
            let mut reg_guard = gpio_register.lock().await;
            *reg_guard = register_value;
        }

        // 2. Read register and set GPIO states (bidirectional sync)
        let current_register = *gpio_register.lock().await;
        
        if current_register & (1 << 0) != 0 { gpio0.set_high(); } else { gpio0.set_low(); }
        if current_register & (1 << 1) != 0 { gpio1.set_high(); } else { gpio1.set_low(); }
        if current_register & (1 << 2) != 0 { gpio2.set_high(); } else { gpio2.set_low(); }
        if current_register & (1 << 3) != 0 { gpio3.set_high(); } else { gpio3.set_low(); }
        if current_register & (1 << 4) != 0 { gpio4.set_high(); } else { gpio4.set_low(); }
        if current_register & (1 << 5) != 0 { gpio5.set_high(); } else { gpio5.set_low(); }
        if current_register & (1 << 6) != 0 { gpio6.set_high(); } else { gpio6.set_low(); }
        if current_register & (1 << 7) != 0 { gpio7.set_high(); } else { gpio7.set_low(); }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn uart_task(
    uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>>,
    gpio_register: &'static Mutex<RawMutex, u8>,
    re_pin_mutex: &'static Mutex<RawMutex, Output<'static>>,  // RE pin mutex
    de_pin_mutex: &'static Mutex<RawMutex, Output<'static>>,  // DE pin mutex
) {
    let mut modbus_buffer = [0u8; 8];
    let mut buffer_pos = 0;
    let mut byte = 0;
    let delay = Delay::new();
    
    loop {
        let mut uart_guard = uart_mutex.lock().await;
        
        {
            let mut re_pin = re_pin_mutex.lock().await;
            let mut de_pin = de_pin_mutex.lock().await;
            re_pin.set_low();  // Enable receiver (active low)
            de_pin.set_low();  // Disable driver (active high)
        }
        
        // Try to read Modbus frame
        let mut temp_buf = [0u8; 1];
        loop {
            if uart_guard.read(&mut temp_buf).is_ok(){
                byte = temp_buf[0];
                delay.delay_millis(2);
            }
            // Buffer Modbus data
            if buffer_pos < modbus_buffer.len() {
                print!("Byte received: {} \n", byte);
                modbus_buffer[buffer_pos] = byte;
                buffer_pos += 1;
            } 
            if buffer_pos >= modbus_buffer.len() {
                // Buffer full, break to process
                buffer_pos = 0;
                break;
            }
        }
        
        print!("Received Modbus frame: {:?} \n", &modbus_buffer);
        print!("CRC data: 0x{:04X}\n", modbus_calculate_crc(&modbus_buffer[..6]));

        // Check node address and CRC using modbus_handler
        // if modbus_buffer[0] != NODE_ADDRESS {
        //     continue;
        // }
        
        if modbus_buffer[0] == NODE_ADDRESS && modbus_verify_crc(&modbus_buffer[..8]) {
            buffer_pos = 0; // Reset buffer position for next frame
            {
                let mut re_pin = re_pin_mutex.lock().await;
                let mut de_pin = de_pin_mutex.lock().await;
                re_pin.set_high();  // Disable receiver
                de_pin.set_high();  // Enable driver
            }
            
            // Get function code (byte 1 after address)
            let function_code = modbus_buffer[1];
            
            match function_code {
                // Function Code 0x01: Read Coils
                0x01 => {
                    // Parse starting address (bytes 2-3)
                    let start_addr = ((modbus_buffer[2] as u16) << 8) | (modbus_buffer[3] as u16);
                    // Parse quantity of coils requested (bytes 4-5)  
                    let requested_quantity = ((modbus_buffer[4] as u16) << 8) | (modbus_buffer[5] as u16);
                    
                    // Validate starting address (0x0000 - 0x0007 for 8 coils)
                    if start_addr < 8 {
                        // Calculate maximum coils available from start_addr
                        let max_available = 8 - start_addr;
                        // Actual quantity to read (limited by available coils)
                        let actual_quantity = if requested_quantity > max_available {
                            max_available
                        } else {
                            requested_quantity
                        };
                        
                        // Get current register value
                        let current_register = *gpio_register.lock().await;
                        
                        // Extract bits from start_addr to start_addr + actual_quantity - 1
                        let mut coil_status = 0u8;
                        for i in 0..actual_quantity {
                            let bit_pos = start_addr + i;
                            if current_register & (1 << bit_pos) != 0 {
                                coil_status |= 1 << i; // Pack bits LSB first
                            }
                        }
                        
                        // Calculate byte count for response 
                        let byte_count = ((actual_quantity + 7) / 8) as u8; // Ceiling division
                        
                        // Build response using modbus_handler
                        let response_data = [byte_count, coil_status];
                        if let Some(response_frame) = modbus_build_frame(
                            NODE_ADDRESS,
                            FunctionCode::Read(ReadCode::Coils),
                            &response_data
                        ) 
                        {
                            // Send response using modbus_handler
                            drop(uart_guard); // Release UART lock before async call
                            let _ = modbus_write_frame(&response_frame).await;
                            Timer::after(Duration::from_millis(1)).await;
                            continue; // Skip to next iteration since uart_guard is dropped
                        }
                    }
                },
                0x04 =>{

                },
                // Function Code 0x05: Write Single Coil
                0x05 => {
                    // Parse coil address (bytes 2-3)
                    let coil_addr = ((modbus_buffer[2] as u16) << 8) | (modbus_buffer[3] as u16);
                    // Parse coil value (bytes 4-5)
                    let coil_value = ((modbus_buffer[4] as u16) << 8) | (modbus_buffer[5] as u16);
                    
                    // Validate coil address (0x0000 - 0x0007 for 8 coils)
                    if coil_addr < 8 {
                        let mut current_register = *gpio_register.lock().await;
                        
                        // Update register bit based on coil value
                        match coil_value {
                            0xFF00 => {
                                // Turn ON: Set bit to 1
                                current_register |= 1 << coil_addr;
                            },
                            0x0000 => {
                                // Turn OFF: Set bit to 0
                                current_register &= !(1 << coil_addr);
                            },
                            _ => {
                                // Invalid coil value - ignore request
                                continue;
                            }
                        }
                        
                        // Update register
                        *gpio_register.lock().await = current_register;
                        
                        // Echo response using modbus_handler (standard for Write Single Coil)
                        let echo_data = [
                            (coil_addr >> 8) as u8,  // Coil addr high
                            coil_addr as u8,         // Coil addr low  
                            (coil_value >> 8) as u8, // Value high
                            coil_value as u8,        // Value low
                        ];
                        
                        if let Some(echo_frame) = modbus_build_frame(
                            NODE_ADDRESS,
                            FunctionCode::Write(WriteCode::SingleCoil),
                            &echo_data
                        ) {
                            // Send echo response using modbus_handler
                            drop(uart_guard); // Release UART lock before async call
                            let _ = modbus_write_frame(&echo_frame).await;
                            Timer::after(Duration::from_millis(1)).await;
                            continue; // Skip to next iteration since uart_guard is dropped
                        }
                    }
                },
                
                _ => {
                    // Unsupported function code - ignore request
                }
            }
        }
        
        drop(uart_guard);
        Timer::after(Duration::from_millis(1)).await;
    }
}
