#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Variables
ESPTOOL_REPO="https://github.com/alphafox02/esptool"
FIRMWARE_URL="https://github.com/lukeswitz/T-Halow/raw/refs/heads/wifi_rid_mesh/firmware/firmware_Xiao_s3andc3_Mesh_RID_Scanner_WiFi/dji_mesh_wifi_firmware.bin"
FIRMWARE_FILE="dji_mesh_wifi_firmware.bin"
ESPTOOL_DIR="esptool"
SERVICE_NAME="zmq-decoder.service"

# PlatformIO Config Values
MONITOR_SPEED=115200
UPLOAD_SPEED=115200
ESP32_PORT=""

# Clone the esptool repository if it doesn't already exist
if [ ! -d "$ESPTOOL_DIR" ]; then
    echo "Cloning esptool repository..."
    git clone "$ESPTOOL_REPO"
else
    echo "Directory '$ESPTOOL_DIR' already exists. Skipping clone."
fi

# Change to the esptool directory
cd "$ESPTOOL_DIR"

# Download the firmware if it doesn't already exist
if [ ! -f "$FIRMWARE_FILE" ]; then
    echo "Downloading firmware..."
    wget "$FIRMWARE_URL" -O "$FIRMWARE_FILE"
else
    echo "Firmware file '$FIRMWARE_FILE' already exists. Skipping download."
fi

# Find available USB serial devices (limit to recognized USB serial ports)
echo "Searching for USB serial devices..."
serial_devices=$(ls /dev/cu.* /dev/tty.* 2>/dev/null)
serial_devices=$(echo "$serial_devices" | grep -i -E 'usb|serial')

if [ -z "$serial_devices" ]; then
    echo "No USB serial devices found. Exiting."
    exit 1
fi

# Display serial devices and let user select one
echo "Found USB serial devices:"
select device in $serial_devices; do
    if [ -n "$device" ]; then
        ESP32_PORT="$device"
        echo "Selected USB serial device: $ESP32_PORT"
        break
    else
        echo "Invalid selection. Please try again."
    fi
done

# Stop the zmq-decoder service if running (assuming systemd)
echo "Stopping zmq-decoder service if running..."
sudo systemctl stop "$SERVICE_NAME" || echo "$SERVICE_NAME is not running or could not be stopped."

# Flash the firmware using esptool.py for the ESP32-C3
echo "Flashing firmware to the device..."
python3 esptool.py \
    --chip esp32c3 \
    --port "$ESP32_PORT" \
    --baud "$UPLOAD_SPEED" \
    --before default_reset \
    --after hard_reset \
    write_flash -z \
    --flash_mode dio \
    --flash_freq 80m \
    --flash_size 16MB \
    0x10000 "$FIRMWARE_FILE"

echo "Firmware flashing complete."

# Restart the zmq-decoder service
echo "Starting zmq-decoder service..."
sudo systemctl start "$SERVICE_NAME" || echo "$SERVICE_NAME could not be started."

echo "Service restarted. Script complete."

