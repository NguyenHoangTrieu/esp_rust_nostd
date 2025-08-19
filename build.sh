cargo build --release
espflash save-image --chip esp32 target/xtensa-esp32-none-elf/release/test_no_std test_no_std.bin
cp test_no_std.bin /mnt/c/embedded/Embedded_Linux/Distributed_Control_System/
cp test_no_std.bin /mnt/c/embedded/DATN_Gateway
cp test_no_std.bin /mnt/c/embedded/SmartVision