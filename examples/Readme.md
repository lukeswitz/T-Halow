## Flashing

### Using `esptool.py`
**T-Halow**

`python3 esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 16MB 0x10000 firmware_T-Halow_DragonOS_RID_Scanner_2024_09_26.bin`


**Most ESP32S3 Variants**

`python3 esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x0 WiFi-RID-esp32s3.bin`

> [!NOTE]
> Flash size `detect` may not always detect the SPI size. If you run into an issue replace with the size for your board from below:

| **Module**                | **Flash Size** |
|---------------------------|----------------|
| ESP32-WROOM-32            | 4MB            |
| ESP32-WROOM-32D           | 4MB            |
| ESP32-WROOM-32E           | 4MB            |
| ESP32-WROVER              | 4MB            |
| ESP32-WROVER-B            | 4MB            |
| ESP32-WROVER-E            | 8MB            |
| ESP32-C3-WROOM-02         | 4MB            |
| ESP32-S3-WROOM-1          | 8MB            |
| ESP32-S3-WROOM-1U         | 8MB            |
| ESP32-S3-WROVER           | 8MB            |
| Seeed Studio XIAO ESP32S3 | 8MB            |
| ESP32-CAM                 | 4MB            |
| ESP32-PICO-D4             | 4MB            |
| ESP32-S2-Saola-1          | 4MB            |
| ESP32-S2-WROOM            | 4MB            |
| ESP32-S2-WROVER           | 8MB            |
| LILYGO T-HaLow            | 16MB           |
| LILYGO T-Display          | 16MB           |
| LILYGO T-Display S3       | 16MB           |
| LILYGO T-SIM7000G         | 4MB            |
| LILYGO T7                 | 16MB           |
| LILYGO T7 S3              | 16MB           |
| LILYGO T-Energy           | 4MB            |
| LILYGO T-Energy S3        | 16MB           |
| LILYGO T-Camera ESP32     | 4MB            |
| LILYGO T-Camera Plus      | 8MB            |
| LILYGO T-CAN485           | 8MB            |
| LILYGO T-BEAM             | 4MB            |
| LILYGO T-RGB              | 16MB           |

> [!IMPORTANT]
> Some variants may offer different flash sizes; it's advisable to check the specific module's datasheet for precise information.

- LILYGO T-SIM7000G module comes with 4MB flash and 8MB PSRAM. 
- LILYGO T7 S3 module features 16MB flash and 8MB PSRAM. 
- LILYGO T-Energy S3 module is equipped with 16MB flash and 8MB PSRAM
