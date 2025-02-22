# Colonel Panic Mesh Board Flasher
### WiFi Drone ID Detection FW

This script automates the entire process of flashing firmware to the Colonel Panic Mesh Board.

## What It Does
- Clones `esptool` if not already installed  
- Downloads the DJI firmware if missing  
- Detects and lets you select the correct USB serial device  
- Flashes the firmware to the ESP32-C3

## Get the Script
```bash
wget https://raw.githubusercontent.com/lukeswitz/T-Halow/refs/heads/wifi_rid_mesh/firmware/firmware_Xiao_c3_Mesh_RID_Scanner_WiFi/flashDJI.sh
```

## How to Use  
1. Give the script permission to run:  
   ```bash
   chmod +x flash_firmware.sh
   ```
  
2. Run the script:
   ```bash
   ./flash_firmware.sh
   ```

Follow the on-screen prompts.
That’s it! The script handles everything.

Credit: All based on work by cemaxecuter / alphafox02.

- Original alphafox RID T-halow: https://github.com/alphafox02/T-Halow

