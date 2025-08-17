cargo build --release --target xtensa-esp32-espidf
espflash save-image --chip esp32 target/xtensa-esp32-espidf/release/dcs-test dcs-test.bin
cp dcs-test.bin /mnt/c/embedded/Embedded_Linux/Distributed_Control_System/
cp dcs-test.bin /mnt/c/embedded/DATN_Gateway
cp dcs-test.bin /mnt/c/embedded/SmartVision
cp target/xtensa-esp32-espidf/release/partition-table.bin /mnt/c/embedded/SmartVision
cp target/xtensa-esp32-espidf/release/bootloader.bin /mnt/c/embedded/SmartVision