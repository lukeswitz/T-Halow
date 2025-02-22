# Mesh Detect Flasher

**For the board by Colonel Panic:
[Tindie](https://www.tindie.com/products/colonel_panic/mesh-detect-2/)**

This script automates flashing WiFi Drone ID firmware to the Colonel Panic Mesh Board. 

## Mesh Detect Firmwares

- [ESP32 OUI Sniffer](https://github.com/colonelpanichacks/esp32-oui-sniffer/tree/Xiao-esp32-c3-serial)
Alert when specific OUI(s) is seen by the Bluetooth scanner. Flash via Aurduino IDE. 
  

- [Deepwoods Device Detection](https://github.com/colonelpanichacks/deepwoods_device_detection/tree/Xiao-esp-32-c3)
Scan WiFi and BT to form a baseline and alert to new devices. Flash via Aurduino IDE. 

- [WiFi Drone ID Detection FW](https://github.com/lukeswitz/T-Halow/tree/wifi_rid_mesh/firmware/firmware_Xiao_c3_Mesh_RID_Scanner_WiFi)
Sends drone detection messages include ID, RSSI, MAC, Operator ID, Location and more.

## Configure Mesh Nodes

- Official Docs https://meshtastic.org/docs/configuration/module/serial/

#### Set the pcb mesh device to use serial mode, and configure the pins:

Ensure both the heltec and your receiver node are on the same channel. Set the pcb mesh settings to:
  - Serial mode
  - 115200 baud rate
  - Text message mode
  - Pins defined RX/TX 19/20

![image](https://github.com/user-attachments/assets/1fee0617-447a-454c-ac78-10243ec7da5c)

### Esptool Dependencies & Setup

```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y git python3 python3-pip wget

# Install Python dependencies
pip3 install esptool pyserial
```

### Flashing 

For WiFi drone detection follow the instructions below. *Prebuilt binaries coming soon for OUI & Deepwoods FW.* 

```bash
# Download the script
wget https://raw.githubusercontent.com/lukeswitz/T-Halow/refs/heads/wifi_rid_mesh/firmware/firmware_Xiao_c3_Mesh_RID_Scanner_WiFi/flashDJI.sh

# Make executable
chmod +x flash_firmware.sh

# Plug in your esp32

# Run the script
./flash_firmware.sh
   ```
 
Follow the on-screen prompts. 
That’s it! The script handles everything.

Credit: Firmware for RID and flasher based on work by cemaxecuter aka alphafox02. Modded with help of Colonel Panic. 

- Original alphafox RID T-halow: https://github.com/alphafox02/T-Halow

