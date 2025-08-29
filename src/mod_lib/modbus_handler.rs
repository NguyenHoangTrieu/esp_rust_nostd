#![allow(dead_code)]

use heapless::Vec;
use esp_hal::Blocking;
use esp_hal::{
    uart::*,
};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;

// -----------------------------------------------------------------------------
// Custom function code ranges
// -----------------------------------------------------------------------------
pub const CUSTOM_FUNCTION_START_0: u8 = 65;
pub const CUSTOM_FUNCTION_END_0: u8 = 72;
pub const CUSTOM_FUNCTION_START_1: u8 = 100;
pub const CUSTOM_FUNCTION_END_1: u8 = 110;

// -----------------------------------------------------------------------------
// Modbus function codes - most commonly used public codes
// -----------------------------------------------------------------------------

/// Read operation function codes (1-bit and 16-bit read operations)
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReadCode {
    Coils = 0x01,              // Read Coils (1-bit read/write)
    DiscreteInputs = 0x02,     // Read Discrete Inputs (1-bit read-only)
    HoldingRegisters = 0x03,   // Read Holding Registers (16-bit read/write)
    InputRegisters = 0x04,     // Read Input Registers (16-bit read-only)
}

/// Write operation function codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum WriteCode {
    SingleCoil = 0x05,                 // Write Single Coil
    SingleRegister = 0x06,             // Write Single Register
    MultipleCoils = 0x0F,              // Write Multiple Coils
    MultipleRegisters = 0x10,          // Write Multiple Registers
    MaskWriteRegister = 0x16,          // Mask Write Register
    ReadWriteMultipleRegisters = 0x17, // Read/Write Multiple Registers
}

/// Diagnostic and other operation function codes
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DiagnosticCode {
    ReadExceptionStatus = 0x07,      // Read Exception Status
    Diagnostics = 0x08,              // Diagnostics (with subcodes)
    GetCommEventCounter = 0x0B,      // Get Comm Event Counter
    GetCommEventLog = 0x0C,          // Get Comm Event Log
    ReportServerId = 0x11,           // Report Server ID
    ReadFileRecord = 0x14,           // Read File Record
    WriteFileRecord = 0x15,          // Write File Record
    ReadFifoQueue = 0x18,            // Read FIFO Queue
    ReadDeviceIdentification = 0x2B, // Read Device Identification (with subcode 0x0E)
}

/// Tagged union for function codes - replaces C union with type safety
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FunctionCode {
    Read(ReadCode),
    Write(WriteCode),
    Diagnostic(DiagnosticCode),
    Custom(u8), // For user-defined codes (65-72, 100-110) or reserved
}

impl FunctionCode {
    /// Extract the raw u8 value for serialization
    pub fn as_u8(&self) -> u8 {
        match *self {
            FunctionCode::Read(code) => code as u8,
            FunctionCode::Write(code) => code as u8,
            FunctionCode::Diagnostic(code) => code as u8,
            FunctionCode::Custom(raw) => raw,
        }
    }
}

// -----------------------------------------------------------------------------
// Data structures
// -----------------------------------------------------------------------------

/// Maximum data payload size for Modbus frame (adjust based on memory constraints)
pub const MAX_MODBUS_DATA_LEN: usize = 252;

/// Complete Modbus data frame structure
/// Uses heapless::Vec for no-std compatibility instead of raw pointer
#[derive(Clone, Debug)]
pub struct DataFrame {
    pub address: u8,                                    // Modbus slave address
    pub function: FunctionCode,                         // Function code and category
    pub data: Vec<u8, MAX_MODBUS_DATA_LEN>,            // Payload bytes (stack-allocated)
    pub crc: u16,                                       // CRC-16 checksum
    pub frame_length: u8,                               // Total frame length in bytes
}

impl DataFrame {
    /// Create a new empty data frame
    pub fn new() -> Self {
        Self {
            address: 0,
            function: FunctionCode::Custom(0),
            data: Vec::new(),
            crc: 0,
            frame_length: 0,
        }
    }
}
pub static mut MODBUS_UART_HANDLE: Option<&'static Mutex<RawMutex, Uart<'static, Blocking>>> = None;

// -----------------------------------------------------------------------------
// Public API function stubs - implement these based on ESP32 HAL
// -----------------------------------------------------------------------------

/// Get a reference to the Modbus UART mutex (for use in other functions)
pub fn get_modbus_uart() -> Option<&'static Mutex<RawMutex, Uart<'static, Blocking>>> {
    unsafe { MODBUS_UART_HANDLE }
}
/// Send a Modbus request frame over UART
/// 
/// # Arguments
/// * `frame` - Reference to the frame to transmit
pub async fn modbus_write_frame(frame: &DataFrame) -> Result<(), &'static str> {
    if let Some(uart_mutex) = get_modbus_uart() {
        // Serialize frame to buffer
        let mut buffer = [0u8; 256]; // Max Modbus frame size
        let mut frame_copy = frame.clone();
        
        if let Some(bytes_written) = modbus_frame_to_buffer(&mut frame_copy, &mut buffer) {
            let mut uart_guard = uart_mutex.lock().await;
            uart_guard.write(&buffer[..bytes_written])
                .map_err(|_| "Failed to write frame to UART")?;
            Ok(())
        } else {
            Err("Failed to serialize frame")
        }
    } else {
        Err("Modbus UART not initialized")
    }
}

/// Calculate CRC-16 checksum for Modbus data
/// 
/// # Arguments
/// * `data` - Byte slice to calculate CRC for
/// 
/// # Returns
/// * CRC-16 value as u16
pub fn modbus_calculate_crc(data: &[u8]) -> u16 {
    if data.is_empty() {
        return 0;
    }

    let mut crc: u16 = 0xFFFF;
    
    for &byte in data {
        crc ^= byte as u16;
        
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc >>= 1;
                crc ^= 0xA001; // Modbus CRC-16 polynomial (reversed 0x8005)
            } else {
                crc >>= 1;
            }
        }
    }
    
    crc
}

/// Verify CRC-16 checksum of received data
/// 
/// # Arguments
/// * `data` - Complete frame including CRC bytes at the end
/// 
/// # Returns
/// * `true` if CRC is valid
/// * `false` if CRC mismatch
pub fn modbus_verify_crc(data: &[u8]) -> bool {
    if data.len() < 4 {
        return false; // Minimum frame: address + function + 1 data byte + CRC (2 bytes)
    }
    
    // Calculate CRC for all data except the last 2 CRC bytes
    let calculated_crc = modbus_calculate_crc(&data[..data.len() - 2]);
    
    // Extract received CRC (BIG-ENDIAN: high byte first, then low byte)
    let received_crc = ((data[data.len() - 2] as u16) << 8) | (data[data.len() - 1] as u16);
    
    calculated_crc == received_crc
}

/// Build a complete Modbus frame from components
/// 
/// # Arguments
/// * `address` - Slave address (1-247)
/// * `function_code` - Function code enum
/// * `payload` - Data payload bytes
/// 
/// # Returns
/// * `Some(DataFrame)` if successful
/// * `None` if payload too large for MAX_MODBUS_DATA_LEN
pub fn modbus_build_frame(
    address: u8,
    function_code: FunctionCode,
    payload: &[u8],
) -> Option<DataFrame> {
    if payload.len() > MAX_MODBUS_DATA_LEN {
        return None; // Payload too large
    }
    
    let mut frame = DataFrame::new();
    frame.address = address;
    frame.function = function_code;
    
    // Copy payload to data vector
    if frame.data.extend_from_slice(payload).is_err() {
        return None; // Failed to copy payload (shouldn't happen if length check passed)
    }
    
    // Calculate frame length: address + function + data + CRC (2 bytes)
    frame.frame_length = 4 + payload.len() as u8;
    frame.crc = 0; // Will be calculated when serializing to buffer
    
    Some(frame)
}

/// Helper function: serialize DataFrame to byte buffer for transmission
/// 
/// # Arguments
/// * `frame` - Mutable reference to frame (CRC will be updated)
/// * `buffer` - Output buffer for serialized frame
/// 
/// # Returns
/// * `Some(bytes_written)` if successful
/// * `None` if buffer too small
fn modbus_frame_to_buffer(
    frame: &mut DataFrame,
    buffer: &mut [u8],
) -> Option<usize> {
    // Check if buffer is large enough
    if buffer.len() < frame.frame_length as usize {
        return None;
    }
    
    let mut idx = 0usize;
    
    // 1. Address (1 byte)
    buffer[idx] = frame.address;
    idx += 1;
    
    // 2. Function code (1 byte)
    buffer[idx] = frame.function.as_u8();
    idx += 1;
    
    // 3. Data payload
    let data_len = frame.frame_length as usize - 4; // Total - (addr + func + crc_16)
    if data_len > 0 && !frame.data.is_empty() {
        buffer[idx..idx + data_len].copy_from_slice(&frame.data[..data_len]);
        idx += data_len;
    }
    
    // 4. Calculate CRC for all bytes written so far
    frame.crc = modbus_calculate_crc(&buffer[..idx]);
    buffer[idx] = ((frame.crc >> 8) & 0xFF) as u8; // High byte  
    idx += 1;
    buffer[idx] = (frame.crc & 0xFF) as u8;        // Low byte
    idx += 1;
    
    Some(idx)
}
