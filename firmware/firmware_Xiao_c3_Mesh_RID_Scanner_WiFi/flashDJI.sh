#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Variables
ESPTOOL_REPO="https://github.com/alphafox02/esptool"
FIRMWARE_OPTIONS=(
    "WiFi Drone Detection:https://github.com/colonelpanichacks/wifi-rid-to-mesh/raw/refs/heads/main/remoteid-mesh/firmware.bin"
    "Camera Detect:https://github.com/lukeswitz/esp32-oui-sniffer/raw/refs/heads/Xiao-esp32-c3-serial/build/meshdetect_privacy.bin"
    "Deepwoods Baseline Scanner:https://github.com/colonelpanichacks/deepwoods_device_detection/raw/refs/heads/Xiao-esp-32-c3/build/esp32c3_device_fingerprint.bin"
)
ESPTOOL_DIR="esptool"

# PlatformIO Config Values
MONITOR_SPEED=115200
UPLOAD_SPEED=115200
ESP32_PORT=""

# Function to find serial devices
find_serial_devices() {
    local devices=""
    
    # Linux devices
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Try physical devices first
        devices=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
        
        # If no devices found, try by-id paths
        if [ -z "$devices" ] && [ -d "/dev/serial/by-id" ]; then
            devices=$(ls /dev/serial/by-id/* 2>/dev/null || true)
        fi
        
        # If still no devices, try by-path
        if [ -z "$devices" ] && [ -d "/dev/serial/by-path" ]; then
            devices=$(ls /dev/serial/by-path/* 2>/dev/null || true)
        fi
    # macOS devices
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # On macOS, prefer /dev/cu.* over /dev/tty.* as they work better for flashing
        devices=$(ls /dev/cu.* 2>/dev/null | grep -i -E 'usb|serial|usbmodem' || true)
    fi
    
    echo "$devices"
}

# Clear screen for better UX
clear

echo "• ▌ ▄ ·.▄▄▄ .▄▄ · ▄ .·▄▄▄▄ ▄▄▄ ▄▄▄▄▄▄▄ .▄▄·▄▄▄▄▄"
echo "·██ ▐███▀▄.▀▐█ ▀.██▪▐██▪ ██▀▄.▀•██ ▀▄.▀▐█ ▌•██  "
echo "▐█ ▌▐▌▐█▐▀▀▪▄▀▀▀███▀▐▐█· ▐█▐▀▀▪▄▐█.▐▀▀▪██ ▄▄▐█.▪"
echo "██ ██▌▐█▐█▄▄▐█▄▪▐██▌▐██. ██▐█▄▄▌▐█▌▐█▄▄▐███▌▐█▌·"
echo "▀▀  █▪▀▀▀▀▀▀ ▀▀▀▀▀▀▀ ▀▀▀▀▀• ▀▀▀ ▀▀▀ ▀▀▀·▀▀▀ ▀▀▀"
echo ""
echo "==================================================="
echo "MeshDetect Firmware Flasher"
echo "==================================================="

# Clone the esptool repository if it doesn't already exist
if [ ! -d "$ESPTOOL_DIR" ]; then
    echo "Cloning esptool repository..."
    git clone "$ESPTOOL_REPO"
else
    echo "Directory '$ESPTOOL_DIR' already exists."
fi

# Change to the esptool directory
cd "$ESPTOOL_DIR"

# Display firmware options and prompt user
echo ""
echo "==================================================="
echo "Available firmware options:"
echo "==================================================="

# Create array for user selection
declare -a options_array
for i in "${!FIRMWARE_OPTIONS[@]}"; do
    echo "$((i+1)). ${FIRMWARE_OPTIONS[$i]%%:*}"
    options_array[i]="${FIRMWARE_OPTIONS[$i]%%:*}"
done
echo ""

# Get user input directly instead of using select
while true; do
    read -p "Select firmware number to flash (1-${#FIRMWARE_OPTIONS[@]}): " choice
    
    # Validate input
    if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#FIRMWARE_OPTIONS[@]}" ]; then
        firmware_choice="${options_array[$((choice-1))]}"
        
        # Find the corresponding URL for the selected firmware
        for option in "${FIRMWARE_OPTIONS[@]}"; do
            if [[ "$option" == "$firmware_choice:"* ]]; then
                FIRMWARE_URL="${option#*:}"
                FIRMWARE_FILE=$(basename "$FIRMWARE_URL")
                break
            fi
        done
        
        echo ""
        echo "Downloading fresh $firmware_choice firmware..."
        wget "$FIRMWARE_URL" -O "$FIRMWARE_FILE"
        
        break
    else
        echo "Invalid selection. Please enter a number between 1 and ${#FIRMWARE_OPTIONS[@]}."
    fi
done

# Find available USB serial devices
echo ""
echo "Searching for USB serial devices..."
serial_devices=$(find_serial_devices)

if [ -z "$serial_devices" ]; then
    echo "ERROR: No USB serial devices found."
    echo "Please check your connection and try again."
    exit 1
fi

# Display serial devices
echo ""
echo "==================================================="
echo "Found USB serial devices:"
echo "==================================================="
device_array=($serial_devices)
for i in "${!device_array[@]}"; do
    echo "$((i+1)). ${device_array[$i]}"
done
echo ""

# Get user selection for device
while true; do
    read -p "Select USB serial device number (1-${#device_array[@]}): " device_choice
    
    # Validate input
    if [[ "$device_choice" =~ ^[0-9]+$ ]] && [ "$device_choice" -ge 1 ] && [ "$device_choice" -le "${#device_array[@]}" ]; then
        ESP32_PORT="${device_array[$((device_choice-1))]}"
        echo ""
        echo "Selected USB serial device: $ESP32_PORT"
        break
    else
        echo "Invalid selection. Please enter a number between 1 and ${#device_array[@]}."
    fi
done

# Flash the firmware using esptool.py for the ESP32-C3
echo ""
echo "Flashing $firmware_choice firmware to the device..."
python3 esptool.py \
    --chip esp32c3 \
    --port "$ESP32_PORT" \
    --baud "$UPLOAD_SPEED" \
    --before default_reset \
    --after hard_reset \
    write_flash -z \
    --flash_mode dio \
    --flash_freq 80m \
    --flash_size 4MB \
    0x10000 "$FIRMWARE_FILE"

echo ""
echo "==================================================="
echo "Firmware flashing complete!"
echo "==================================================="
