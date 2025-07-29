# Firmware

## WarDragon Scanner 
![image](https://github.com/user-attachments/assets/5823cf88-0d82-4b68-9610-1b4c0fb24432)
**Files/Supported Board:**
`tHalow_WiFi_AP_RID_firmware.bin`
`xiao_s3_WiFi_AP_RID_firmware.bin`
`xiao_c3_WiFi_AP_RID_firmware.bin`

### What is it? 

A standalone ESP32 WiFi Remote ID Scanner with ZMQ Publisher

Made for [DragonSync iOS & macOS](https://github.com/Root-Down-Digital/DragonSync-iOS) as a portable alternative to the full detection stack. 

## Build/Flash

Modify and build yourself, or flash an existing binary file above:

### Build from Source

- Download or clone the repo
- Open the `examples/DragonOS_RID_AP` [folder](https://github.com/lukeswitz/T-Halow/tree/master/examples/DragonOS_RID_AP) in VSCode.
- Modify [main.cpp](https://github.com/lukeswitz/T-Halow/blob/4fab340f2151863235d8f8e206daa02261055869/examples/DragonOS_RID_AP/main.cpp#L63) and change the AP credentials before use.
- Build and upload using PlatformIO

### Flash Precompiled Binary
- Use default credentials, flash precompiled binary with `esptool.py`

   ```
  esptool.py --chip auto --port /dev/yourportname --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x10000 firmwareFile.bin
   ```

## Usage 

**Default WiFi AP Credentials**

AP: `WarDragon-Scanner`

PW: `wardragon123`

IP:  `192.168.4.1`

#### A. Use the App
- Set the IP in the app
  ![image](https://github.com/user-attachments/assets/059c5efa-5a8a-4af8-8401-c8fa00273610)

#### B. View WebUI
- Connect to the Wardragon-Scanner AP
- Visit 192.168.4.1 in a browser for raw data
  

## Notes
**This project not affiliated with WarDragon, DragonOS etc. Thanks to cemaxecuter for the original WiFi RID FW- It can be found in this repo.**

> [!IMPORTANT]
> This is a work in progress, expect breaking changes and possible stability issues.
>
> Use within legal and ethical bounds. Author not responsible for anything that happens should you use any code or knowledge provided here.
