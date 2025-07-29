# WarDragon Scanner 
_**A standalone ESP32 WiFi Remote ID Scanner with ZMQ Publisher**_

**Files/Currently Supported Boards:**
   - `tHalow_WiFi_AP_RID_firmware.bin`
   - `xiao_s3_WiFi_AP_RID_firmware.bin`
   - `xiao_c3_WiFi_AP_RID_firmware.bin`

Made for [DragonSync iOS & macOS](https://github.com/Root-Down-Digital/DragonSync-iOS) as a portable alternative to the full detection stack. 

## Getting Started

Modify and build yourself, or flash an existing binary file listed above:

### Flash Precompiled Binary
- Download the firmware for your board above
- Flash precompiled binary with `esptool.py`

  (Uses default credentials for the AP. Build from source to change)

   ```
  esptool.py --chip auto --port /dev/yourportname --baud 115200 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x10000 firmwareFile.bin
   ```
### Build from Source

- Download or clone the repo
- Open the `examples/DragonOS_RID_AP` [folder](https://github.com/lukeswitz/T-Halow/tree/master/examples/DragonOS_RID_AP) in VSCode.
- Modify [main.cpp](https://github.com/lukeswitz/T-Halow/blob/4fab340f2151863235d8f8e206daa02261055869/examples/DragonOS_RID_AP/main.cpp#L63) and change the AP credentials before use.
- Build and upload using PlatformIO

## Usage 

**Default WiFi AP Credentials**

AP: `WarDragon-Scanner`

PW: `wardragon123`

IP:  `192.168.4.1`

#### A. Use the App
   
   <img src="https://github.com/user-attachments/assets/059c5efa-5a8a-4af8-8401-c8fa00273610" width="50%" >

#### B. Use the WebUI
   - Connect to the Wardragon-Scanner AP
   - Visit 192.168.4.1 in a browser for a simple webUI
   <img src="https://github.com/user-attachments/assets/5823cf88-0d82-4b68-9610-1b4c0fb24432" width="80%" >

## Notes
**This project not affiliated with WarDragon, DragonOS etc. Thanks to cemaxecuter for the original WiFi RID FW- It can be found in this repo.**

> [!IMPORTANT]
> This is a work in progress, expect breaking changes and possible stability issues.
>
> Use within legal and ethical bounds. Author not responsible for anything that happens should you use any code or knowledge provided here.
