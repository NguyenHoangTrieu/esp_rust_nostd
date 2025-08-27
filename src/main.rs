#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::Blocking;
use esp_hal::{
    timer::timg::TimerGroup,
    uart::*,
};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use static_cell::StaticCell;

// Static cells
static UART_CELL: StaticCell<Mutex<RawMutex, Uart<'static, Blocking>>> = StaticCell::new();

// GPIO Register - 8-bit register để quản lý 8 GPIO pins (GLOBAL ONLY)
static GPIO_REGISTER: StaticCell<Mutex<RawMutex, u8>> = StaticCell::new();

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

// CRC calculation for Modbus (using modbus_handler functions)
fn calculate_crc16(data: &[u8]) -> u16 {
    modbus_calculate_crc(data)
}

// Verify CRC (big endian format) 
fn verify_crc(data: &[u8]) -> bool {
    modbus_verify_crc(data)
}

// Send Modbus response with CRC
async fn send_modbus_response(uart_guard: &mut embassy_sync::mutex::MutexGuard<'_, RawMutex, Uart<'static, Blocking>>, response: &[u8]) {
    let crc = calculate_crc16(response);
    let mut full_response = [0u8; 10];
    let len = response.len();
    full_response[..len].copy_from_slice(response);
    full_response[len] = ((crc >> 8) & 0xFF) as u8;     // High byte first (big endian)
    full_response[len + 1] = (crc & 0xFF) as u8;       // Low byte second
    uart_guard.write(&full_response[..len + 2]).ok();
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timer0 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timer0.timer0);

    let uart = Uart::new(
        peripherals.UART0,
        Config::default().with_baudrate(115200),
    )
    .unwrap()
    .with_rx(peripherals.GPIO3)
    .with_tx(peripherals.GPIO1);

    // Khởi tạo 8 GPIO outputs (LOCAL - tránh pin 20,21,22,23)
    // Sử dụng pins: 2,4,5,12,13,14,15,16
    let mut gpio0 = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());   // Bit 0
    let mut gpio1 = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());   // Bit 1
    let mut gpio2 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());   // Bit 2
    let mut gpio3 = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());  // Bit 3
    let mut gpio4 = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());  // Bit 4
    let mut gpio5 = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());  // Bit 5
    let mut gpio6 = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());  // Bit 6
    let mut gpio7 = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());  // Bit 7

    // Initialize static cells
    let uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>> =
        UART_CELL.init(Mutex::new(uart));

    let gpio_register: &'static Mutex<RawMutex, u8> =
        GPIO_REGISTER.init(Mutex::new(0));

    // Set Modbus UART handle
    unsafe {
        crate::mod_lib::modbus_handler::MODBUS_UART_HANDLE = Some(uart_mutex);
    }

    // Spawn tasks
    spawner.spawn(uart_task(uart_mutex, gpio_register)).unwrap();

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

        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn uart_task(
    uart_mutex: &'static Mutex<RawMutex, Uart<'static, Blocking>>,
    gpio_register: &'static Mutex<RawMutex, u8>,
) {
    let mut modbus_buffer = [0u8; 32];
    let mut buffer_pos = 0;

    loop {
        let mut uart_guard = uart_mutex.lock().await;
        
        // Try to read Modbus frame
        let mut temp_buf = [0u8; 1];
        if uart_guard.read_buffered(&mut temp_buf).is_ok() {
            let byte = temp_buf[0];
            
            // Buffer Modbus data
            if buffer_pos < modbus_buffer.len() {
                modbus_buffer[buffer_pos] = byte;
                buffer_pos += 1;
            }

            // Check for complete Modbus frames (8 bytes minimum)
            if buffer_pos >= 8 {
                // Check node address (byte 0)
                if modbus_buffer[0] == NODE_ADDRESS && verify_crc(&modbus_buffer[..8]) {
                    
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
                                
                                // Build response: [Address][Function][Byte_Count][Data][CRC]
                                let response = [NODE_ADDRESS, 0x01, byte_count, coil_status];
                                send_modbus_response(&mut uart_guard, &response).await;
                                
                            } else {
                                // Starting address out of range - could send exception response
                                // For now, ignore invalid requests
                            }
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
                                
                                // Echo response (standard for Write Single Coil)
                                // Response is identical to request for successful write
                                uart_guard.write(&modbus_buffer[..8]).ok();
                                
                            } else {
                                // Coil address out of range - could send exception response
                                // For now, ignore invalid requests
                            }
                        },
                        
                        _ => {
                            // Unsupported function code - ignore request
                        }
                    }
                }
                
                buffer_pos = 0; // Reset buffer after processing
            }

            // Reset buffer if it gets too full without finding valid frame
            if buffer_pos >= modbus_buffer.len() {
                buffer_pos = 0;
            }
        }

        drop(uart_guard);
        Timer::after(Duration::from_millis(1)).await;
    }
}

