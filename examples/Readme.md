## Flashing

### Using `esptool.py`
**Most ESP32S3/C3**

`python3 esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 16MB 0x10000 firmware_T-Halow_DragonOS_RID_Scanner_2024_09_26.bin`

> [!NOTE]
> Flash size `detect` may not always detect the SPI size. The offset is also a variable to verify. If you run into an issue replace with the size and offset for your board by searching that specific module. 
