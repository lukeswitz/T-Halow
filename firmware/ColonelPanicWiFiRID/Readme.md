# Mesh Detect RID FW Flasher Script
For the board by Colonel Panic:
[Tindie](https://www.tindie.com/products/colonel_panic/mesh-detect-2/)

This script automates flashing WiFi Drone ID firmware to the Colonel Panic Mesh Board. 

**[WiFi Drone ID Detection FW](https://github.com/lukeswitz/T-Halow/tree/wifi_rid_mesh/firmware/firmware_Xiao_c3_Mesh_RID_Scanner_WiFi)**


Detected drone messages include ID, RSSI, MAC, Operator ID, Location and more.

### Esptool Dependencies & Setup

#### Set the mesh device to use serial mode - configure the pins

- Official Docs https://meshtastic.org/docs/configuration/module/serial/

![image](https://github.com/user-attachments/assets/1fee0617-447a-454c-ac78-10243ec7da5c)


```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y git python3 python3-pip wget

# Install Python dependencies
pip3 install esptool pyserial
```

### Run & Flash

```bash
# Download the script
wget https://raw.githubusercontent.com/lukeswitz/T-Halow/refs/heads/wifi_rid_mesh/firmware/firmware_Xiao_c3_Mesh_RID_Scanner_WiFi/flashDJI.sh

# Make executable
chmod +x flash_firmware.sh

# Run the script
./flash_firmware.sh
   ```

Follow the on-screen prompts.
That’s it! The script handles everything.

Credit: All based on work by cemaxecuter / alphafox02.

- Original alphafox RID T-halow: https://github.com/alphafox02/T-Halow

