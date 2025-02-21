# WiFi RID - ESP32 Flashing Guide

## Overview
This guide explains how to flash ESP32-based boards using `esptool.py`, covering both fresh installations and firmware updates.

## Supported Boards
- Seeed XIAO ESP32C3
- ESP32S3 Generic  
- T-Halow (ESP32-C6)

## Prerequisites
- Python installed
- esptool.py installed (`pip install esptool`)
- Required binary files
- USB connection to your board

## Flash Memory Addresses
All supported boards use these standard addresses:
- Bootloader: 0x0
- Partition Table: 0x8000  
- Firmware: 0x10000

## Fresh Installation
Use this command when flashing a board for the first time or completely reflashing:

```bash
esptool.py --chip [chiptype] --port COM3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m 0x0 bootloader.bin 0x8000 partitions.bin 0x10000 firmware.bin
```

Replace `[chiptype]` with:

`esp32c3` for XIAO ESP32C3
`esp32s3` for ESP32S3
`esp32c6` for T-Halow
- Or leave it out or use `auto` to let the tool detect it

## Firmware Update

For updating only the firmware on an already configured board:
```bash
esptool.py --chip [chiptype] --port COM3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m 0x10000 firmware.bin
```

### Parameters Explained

- `--chip`: Specifies the target ESP32 variant
- `--port`: Serial port (COM3, /dev/ttyUSB0, etc.)
- `--baud`: Data transfer rate (921600 recommended)
- `--before/--after`: Reset behavior
- `-z`: Automatically detect flash size
- `--flash_mode`: Flash interface mode (dio recommended)
- `--flash_freq`: Flash frequency (80m recommended)

## Troubleshooting

- Ensure correct port selection
- Try lower baud rate if experiencing issues
- Verify binary files match your board type
- Check USB cable connection
- Install/update drivers if needed

Notes

- Fresh installation recommended for first-time setup
- Update command sufficient for routine firmware updates
- Backup existing firmware before flashing
- Some boards may require different flash parameters
