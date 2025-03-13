#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Variables
ESPTOOL_REPO="https://github.com/alphafox02/esptool"
FIRMWARE_OPTIONS=(
    "WiFi Drone Detection:https://github.com/lukeswitz/T-Halow/raw/refs/heads/wifi_rid_mesh/firmware/firmware_Xiao_s3andc3_Mesh_RID_Scanner_WiFi/dji_mesh_wifi_firmware.bin"
    "Device Detect:https://github.com/lukeswitz/esp32-oui-sniffer/raw/refs/heads/Xiao-esp32-c3-serial/build/meshdetect_privacy.bin"
    "Deepwoods Baseline Scanner:https://github.com/lukeswitz/deepwoods_device_detection/raw/refs/heads/Xiao-esp-32-c3/build/esp32c3_device_fingerprint.bin"
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
for i in "${!FIRMWARE_OPTIONS[@]}"; do
    echo "$((i+1)). ${FIRMWARE_OPTIONS[$i]%%:*}"
done
echo ""


PS3="Select firmware number to flash (1-${#FIRMWARE_OPTIONS[@]}): "
select firmware_choice in "${FIRMWARE_OPTIONS[@]%%:*}"; do
    if [ -n "$firmware_choice" ]; then
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

# Display serial devices and let user select one
echo ""
echo "==================================================="
echo "Found USB serial devices:"
echo "==================================================="
select device in $serial_devices; do
    if [ -n "$device" ]; then
        ESP32_PORT="$device"
        echo ""
        echo "Selected USB serial device: $ESP32_PORT"
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# Stop any interfering services
stop_services

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
