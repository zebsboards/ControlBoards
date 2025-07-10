REM Put controller into bootloader mode using the buttons on the board.
REM Change COM number in following line to match what device manager shows for the bootloader (ESP32) displays

esptool.exe --chip esp32s2 --port COM6 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 CabBlaster_R1.bootloader.bin 0x8000 CabBlaster_R1.partitions.bin 0xe000 boot_app0.bin 0x10000 CabBlaster_R1.bin

REM Save the file and run

pause