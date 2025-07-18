#if !defined(ARDUINO_ARCH_ESP32)
  #error "This program requires an ESP32"
#endif

#include <Arduino.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "opendroneid.h"
#include "odid_wifi.h"
#include <esp_timer.h>

// Custom UART pin definitions for Serial1
const int SERIAL1_RX_PIN = 4;  // GPIO4
const int SERIAL1_TX_PIN = 5;  // GPIO5

struct uav_data {
  uint8_t  mac[6];
  uint8_t  padding[1];
  int8_t   rssi;
  char     op_id[ODID_ID_SIZE + 1];
  char     uav_id[ODID_ID_SIZE + 1];
  double   lat_d;
  double   long_d;
  double   base_lat_d;
  double   base_long_d;
  int      altitude_msl;
  int      height_agl;
  int      speed;
  int      heading;
  int      speed_vertical;
  int      altitude_pressure;
  int      horizontal_accuracy;
  int      vertical_accuracy;
  int      baro_accuracy;
  int      speed_accuracy;
  int      timestamp;
  int      status;
  int      height_type;
  int      operator_location_type;
  int      classification_type;
  int      area_count;
  int      area_radius;
  int      area_ceiling;
  int      area_floor;
  int      operator_altitude_geo;
  uint32_t system_timestamp;
  int      operator_id_type;
  uint8_t  ua_type;
  uint8_t  auth_type;
  uint8_t  auth_page;
  uint8_t  auth_length;
  uint32_t auth_timestamp;
  char     auth_data[ODID_AUTH_PAGE_NONZERO_DATA_SIZE + 1];
  uint8_t  desc_type;
  char     description[ODID_STR_SIZE + 1];
  uint32_t last_seen;
  int      flag;
};

// Function declarations
static void wifi_callback(void *buffer, wifi_promiscuous_pkt_type_t type);
static void parse_odid(struct uav_data *UAV, ODID_UAS_Data *UAS_data);
static void parse_french_id(struct uav_data *UAV, uint8_t *payload);
static void print_json(struct uav_data *UAV, int index);
static void store_mac(struct uav_data *UAV, uint8_t *payload);
void bleScanTask(void *pvParameters);
void wifiProcessTask(void *pvParameters);

// Globals
static int packetCount = 0;
static ODID_UAS_Data UAS_data;
unsigned long last_status = 0;
BLEScan* pBLEScan = nullptr;

// Storage for UAV data
#define MAX_UAVS 8
static uav_data uavs[MAX_UAVS] = {0};

// Helper: Find or allocate an entry for a given MAC address
uav_data* next_uav(uint8_t* mac) {
  for (int i = 0; i < MAX_UAVS; i++) {
    if (memcmp(uavs[i].mac, mac, 6) == 0)
      return &uavs[i];
  }
  for (int i = 0; i < MAX_UAVS; i++) {
    if (uavs[i].mac[0] == 0)
      return &uavs[i];
  }
  return &uavs[0];  // Fallback if all are used
}

// BLE Advertised Device Callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
public:
  void onResult(BLEAdvertisedDevice device) override {
    int len = device.getPayloadLength();
    if (len <= 0)
      return;
      
    uint8_t* payload = device.getPayload();
    // Check for a Remote ID advertisement signature
    if (len > 5 &&
        payload[1] == 0x16 &&
        payload[2] == 0xFA &&
        payload[3] == 0xFF &&
        payload[4] == 0x0D)
    {
      uint8_t* mac = (uint8_t*) device.getAddress().getNative();
      uav_data* UAV = next_uav(mac);
      UAV->last_seen = millis();
      UAV->rssi = device.getRSSI();
      UAV->flag = 1;
      memcpy(UAV->mac, mac, 6);
      
      // Assume the Remote ID message payload starts at index 6
      uint8_t* odid = &payload[6];
      switch (odid[0] & 0xF0) {
        case 0x00: {  // Basic ID message
          ODID_BasicID_data basic;
          decodeBasicIDMessage(&basic, (ODID_BasicID_encoded*) odid);
          strncpy(UAV->uav_id, (char*) basic.UASID, ODID_ID_SIZE);
          break;
        }
        case 0x10: {  // Location message
          ODID_Location_data loc;
          decodeLocationMessage(&loc, (ODID_Location_encoded*) odid);
          UAV->lat_d = loc.Latitude;
          UAV->long_d = loc.Longitude;
          UAV->altitude_msl = (int) loc.AltitudeGeo;
          UAV->height_agl = (int) loc.Height;
          UAV->speed = (int) loc.SpeedHorizontal;
          UAV->heading = (int) loc.Direction;
          UAV->speed_vertical = (int)loc.SpeedVertical;
          UAV->altitude_pressure = (int)loc.AltitudeBaro;
          UAV->height_type = loc.HeightType;
          UAV->horizontal_accuracy = loc.HorizAccuracy;
          UAV->vertical_accuracy = loc.VertAccuracy;
          UAV->baro_accuracy = loc.BaroAccuracy;
          UAV->speed_accuracy = loc.SpeedAccuracy;
          UAV->timestamp = (int)loc.TimeStamp;
          UAV->status = loc.Status;
          break;
        }
        case 0x40: {  // System message
          ODID_System_data sys;
          decodeSystemMessage(&sys, (ODID_System_encoded*) odid);
          UAV->base_lat_d = sys.OperatorLatitude;
          UAV->base_long_d = sys.OperatorLongitude;
          UAV->operator_location_type = sys.OperatorLocationType;
          UAV->classification_type = sys.ClassificationType;
          UAV->area_count = sys.AreaCount;
          UAV->area_radius = sys.AreaRadius;
          UAV->area_ceiling = sys.AreaCeiling;
          UAV->area_floor = sys.AreaFloor;
          UAV->operator_altitude_geo = sys.OperatorAltitudeGeo;
          UAV->system_timestamp = sys.Timestamp;
          break;
        }
        case 0x50: {  // Operator ID message
          ODID_OperatorID_data op;
          decodeOperatorIDMessage(&op, (ODID_OperatorID_encoded*) odid);
          strncpy(UAV->op_id, (char*) op.OperatorId, ODID_ID_SIZE);
          break;
        }
        default:
          break;
      }
    }
  }
};

void setup() {
  setCpuFrequencyMhz(160);
  esp_log_level_set("BLEScan", ESP_LOG_ERROR);  
  // Initialize serial ports
  Serial.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN); uncomment to use the mesh over serial function
  Serial.println("{ \"message\": \"Starting ESP32 WiFi/BLE Dual-Core Remote ID Scanner\" }");
  
  // Initialize WiFi in promiscuous mode
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_callback);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); // ch 1,6,11 and BT can overlap
  
  // Initialize BLE scanner
  BLEDevice::init("BLE RemoteID Scanner");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  // Clear UAV storage
  memset(uavs, 0, sizeof(uavs));
  
  // Create tasks on separate cores
  xTaskCreatePinnedToCore(
    bleScanTask,      // Function to implement the task
    "BLEScanTask",    // Name of the task
    16384,            // Stack size in words (increased)
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL,             // Task handle
    0                 // Core where the task should run (Core 0)
  );
  
  xTaskCreatePinnedToCore(
    wifiProcessTask,     // Function to implement the task
    "WiFiProcessTask",   // Name of the task
    10000,               // Stack size in words
    NULL,                // Task input parameter
    1,                   // Priority of the task
    NULL,                // Task handle
    1                    // Core where the task should run (Core 1)
  );
}

void loop() {
  // Tasks run on separate cores, so loop can be empty
  vTaskDelete(NULL);
}

// WiFi callback function processes incoming packets
static void wifi_callback(void *buffer, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT)
    return; // We only care about management frames
  
  wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buffer;
  uint8_t *payload = packet->payload;
  int length = packet->rx_ctrl.sig_len;

  // Bail out on empty or too-short frames
  if (length <= 0 || length < 10) {
    return;
  }
  
  // Check for NAN action frame (OpenDroneID)
  static const uint8_t nan_dest[6] = { 0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00 };
  if (memcmp(nan_dest, &payload[4], 6) == 0) {
      // reserve a 6-byte buffer of type char for the parser
      char mac_buf[6] = { 0 };
      if (odid_wifi_receive_message_pack_nan_action_frame(
             &UAS_data,
              mac_buf,
              payload,
              length
          ) == 0) {

          uav_data UAV;
          memset(&UAV, 0, sizeof(UAV));
          memcpy(UAV.mac, reinterpret_cast<uint8_t*>(mac_buf), 6);
          UAV.rssi      = packet->rx_ctrl.rssi;
          UAV.last_seen = millis();

          parse_odid(&UAV, &UAS_data);

          // Store/update in DB
          uav_data* dbUAV = next_uav(UAV.mac);
          memcpy(dbUAV, &UAV, sizeof(UAV));
          dbUAV->flag = 1;
      }
  }

  // Check for beacon frames (WiFi beacons)
  else if (payload[0] == 0x80) {
    int offset = 36;
    while (offset < length) {
      int typ = payload[offset];
      int len = payload[offset + 1];
      
      // Check for ODID Wi-Fi beacon
      if ((typ == 0xdd) &&
          (((payload[offset + 2] == 0x90 && payload[offset + 3] == 0x3a && payload[offset + 4] == 0xe6)) ||
           ((payload[offset + 2] == 0xfa && payload[offset + 3] == 0x0b && payload[offset + 4] == 0xbc)))) {
        int j = offset + 7;
        if (j < length) {
          memset(&UAS_data, 0, sizeof(UAS_data));
          odid_message_process_pack(&UAS_data, &payload[j], length - j);
          
          uav_data UAV;
          memset(&UAV, 0, sizeof(UAV));
          memcpy(UAV.mac, &payload[10], 6);
          UAV.rssi = packet->rx_ctrl.rssi;
          UAV.last_seen = millis();
          
          // Extract data from UAS_data
          parse_odid(&UAV, &UAS_data);
          
          // Store in database
          uav_data* dbUAV = next_uav(UAV.mac);
          memcpy(dbUAV, &UAV, sizeof(UAV));
          dbUAV->flag = 1;
        }
      }
      // Check for French format
      else if ((typ == 0xdd) && 
               (payload[offset + 2] == 0x6a && payload[offset + 3] == 0x5c && payload[offset + 4] == 0x35)) {
        uav_data UAV;
        memset(&UAV, 0, sizeof(UAV));
        memcpy(UAV.mac, &payload[10], 6);
        UAV.rssi = packet->rx_ctrl.rssi;
        UAV.last_seen = millis();
        
        parse_french_id(&UAV, &payload[offset]);
        
        // Store in database
        uav_data* dbUAV = next_uav(UAV.mac);
        memcpy(dbUAV, &UAV, sizeof(UAV));
        dbUAV->flag = 1;
      }
      
      if (len <= 0)
        break;
        
      offset += len + 2;
    }
  }
}

// Parse OpenDroneID data into UAV structure
static void parse_odid(struct uav_data *UAV, ODID_UAS_Data *UAS_data2) {
  if (UAS_data2->BasicIDValid[0]) {
    strncpy(UAV->uav_id, (char *)UAS_data2->BasicID[0].UASID, ODID_ID_SIZE);
    UAV->ua_type = UAS_data2->BasicID[0].UAType;
  }
  
  if (UAS_data2->LocationValid) {
    UAV->lat_d = UAS_data2->Location.Latitude;
    UAV->long_d = UAS_data2->Location.Longitude;
    UAV->altitude_msl = (int)UAS_data2->Location.AltitudeGeo;
    UAV->height_agl = (int)UAS_data2->Location.Height;
    UAV->speed = (int)UAS_data2->Location.SpeedHorizontal;
    UAV->heading = (int)UAS_data2->Location.Direction;
    UAV->speed_vertical = (int)UAS_data2->Location.SpeedVertical;
    UAV->altitude_pressure = (int)UAS_data2->Location.AltitudeBaro;
    UAV->height_type = UAS_data2->Location.HeightType;
    UAV->horizontal_accuracy = UAS_data2->Location.HorizAccuracy;
    UAV->vertical_accuracy = UAS_data2->Location.VertAccuracy;
    UAV->baro_accuracy = UAS_data2->Location.BaroAccuracy;
    UAV->speed_accuracy = UAS_data2->Location.SpeedAccuracy;
    UAV->timestamp = (int)UAS_data2->Location.TimeStamp;
    UAV->status = UAS_data2->Location.Status;
  }
  
  if (UAS_data2->SystemValid) {
    UAV->base_lat_d = UAS_data2->System.OperatorLatitude;
    UAV->base_long_d = UAS_data2->System.OperatorLongitude;
    UAV->operator_location_type = UAS_data2->System.OperatorLocationType;
    UAV->classification_type = UAS_data2->System.ClassificationType;
    UAV->area_count = UAS_data2->System.AreaCount;
    UAV->area_radius = UAS_data2->System.AreaRadius;
    UAV->area_ceiling = UAS_data2->System.AreaCeiling;
    UAV->area_floor = UAS_data2->System.AreaFloor;
    UAV->operator_altitude_geo = UAS_data2->System.OperatorAltitudeGeo;
    UAV->system_timestamp = UAS_data2->System.Timestamp;
  }
  
  if (UAS_data2->AuthValid[0]) {
    UAV->auth_type = UAS_data2->Auth[0].AuthType;
    UAV->auth_page = UAS_data2->Auth[0].DataPage;
    UAV->auth_length = UAS_data2->Auth[0].Length;
    UAV->auth_timestamp = UAS_data2->Auth[0].Timestamp;
    memcpy(UAV->auth_data, UAS_data2->Auth[0].AuthData, sizeof(UAV->auth_data) - 1);
  }
  
  if (UAS_data2->SelfIDValid) {
    UAV->desc_type = UAS_data2->SelfID.DescType;
    strncpy(UAV->description, UAS_data2->SelfID.Desc, ODID_STR_SIZE);
  }
  
  if (UAS_data2->OperatorIDValid) {
    UAV->operator_id_type = UAS_data2->OperatorID.OperatorIdType;
    strncpy(UAV->op_id, (char *)UAS_data2->OperatorID.OperatorId, ODID_ID_SIZE);
  }
}

// Parse French ID format
static void parse_french_id(struct uav_data *UAV, uint8_t *payload) {
  int length = payload[1];
  int j = 6;
  
  union {
    int32_t i32;
    uint32_t u32;
  } uav_lat, uav_long, base_lat, base_long;
  
  union {
    int16_t i16;
    uint16_t u16;
  } alt, height;
  
  uav_lat.u32 = uav_long.u32 = base_lat.u32 = base_long.u32 = 0;
  alt.u16 = height.u16 = 0;
  
  while (j < length) {
    int t = payload[j];
    int l = payload[j + 1];
    uint8_t *v = &payload[j + 2];
    
    switch (t) {
    case 2: // Operator ID
      for (int i = 0; (i < (l - 6)) && (i < ODID_ID_SIZE); ++i) {
        UAV->op_id[i] = (char)v[i + 6];
      }
      break;
    case 3: // UAV ID
      for (int i = 0; (i < l) && (i < ODID_ID_SIZE); ++i) {
        UAV->uav_id[i] = (char)v[i];
      }
      break;
    case 4:
      for (int i = 0; i < 4; ++i) {
        uav_lat.u32 <<= 8;
        uav_lat.u32 |= v[i];
      }
      break;
    case 5:
      for (int i = 0; i < 4; ++i) {
        uav_long.u32 <<= 8;
        uav_long.u32 |= v[i];
      }
      break;
    case 6:
      alt.u16 = (((uint16_t)v[0]) << 8) | (uint16_t)v[1];
      break;
    case 7:
      height.u16 = (((uint16_t)v[0]) << 8) | (uint16_t)v[1];
      break;
    case 8:
      for (int i = 0; i < 4; ++i) {
        base_lat.u32 <<= 8;
        base_lat.u32 |= v[i];
      }
      break;
    case 9:
      for (int i = 0; i < 4; ++i) {
        base_long.u32 <<= 8;
        base_long.u32 |= v[i];
      }
      break;
    case 10:
      UAV->speed = v[0];
      break;
    case 11:
      UAV->heading = (((uint16_t)v[0]) << 8) | (uint16_t)v[1];
      break;
    default:
      break;
    }
    
    j += l + 2;
  }
  
  UAV->lat_d = 1.0e-5 * (double)uav_lat.i32;
  UAV->long_d = 1.0e-5 * (double)uav_long.i32;
  UAV->base_lat_d = 1.0e-5 * (double)base_lat.i32;
  UAV->base_long_d = 1.0e-5 * (double)base_long.i32;
  UAV->altitude_msl = alt.i16;
  UAV->height_agl = height.i16;
}

// Store MAC address in UAV structure
static void store_mac(struct uav_data *uav, uint8_t *payload) {
  memcpy(uav->mac, &payload[10], 6);
}

// Print JSON format of the UAV data
static void print_json(struct uav_data *UAV, int index) {
  // Calculate uptime in seconds using esp_timer_get_time()
  unsigned long uptime_seconds = esp_timer_get_time() / 1000000UL;
  
  // Convert latitude and longitude to string with desired precision
  char lat[16], lon[16], op_lat[16], op_lon[16];
  dtostrf(UAV->lat_d, 11, 6, lat);
  dtostrf(UAV->long_d, 11, 6, lon);
  dtostrf(UAV->base_lat_d, 11, 6, op_lat);
  dtostrf(UAV->base_long_d, 11, 6, op_lon);
  
  // Format the MAC address
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           UAV->mac[0], UAV->mac[1], UAV->mac[2],
           UAV->mac[3], UAV->mac[4], UAV->mac[5]);
  
  // Create the JSON string with the actual uptime
  char json[4096];
  snprintf(json, sizeof(json),
           "{"
           "\"index\": %d,"
           "\"runtime\": %lu,"
           "\"Basic ID\": {"
           "\"id\": \"%s\","
           "\"id_type\": \"Serial Number (ANSI/CTA-2063-A)\","
           "\"ua_type\": %d,"
           "\"MAC\": \"%s\","
           "\"RSSI\": %d"
           "},"
           "\"Location/Vector Message\": {"
           "\"latitude\": %s,"
           "\"longitude\": %s,"
           "\"speed\": %d,"
           "\"vert_speed\": %d,"
           "\"geodetic_altitude\": %d,"
           "\"height_agl\": %d,"
           "\"status\": %d,"
           "\"direction\": %d,"
           "\"alt_pressure\": %d,"
           "\"height_type\": %d,"
           "\"horiz_acc\": %d,"
           "\"vert_acc\": %d,"
           "\"baro_acc\": %d,"
           "\"speed_acc\": %d,"
           "\"timestamp\": %d"
           "},"
           "\"Self-ID Message\": {"
           "\"text\": \"UAV %s operational\","
           "\"description_type\": %d,"
           "\"description\": \"%s\""
           "},"
           "\"System Message\": {"
           "\"latitude\": %s,"
           "\"longitude\": %s,"
           "\"operator_lat\": %s,"
           "\"operator_lon\": %s,"
           "\"area_count\": %d,"
           "\"area_radius\": %d,"
           "\"area_ceiling\": %d,"
           "\"area_floor\": %d,"
           "\"operator_alt_geo\": %d,"
           "\"classification\": %d,"
           "\"timestamp\": %d"
           "},"
           "\"Auth Message\": {"
           "\"type\": %d,"
           "\"page\": %d,"
           "\"length\": %d,"
           "\"timestamp\": %d,"
           "\"data\": \"%s\""
           "}"
           "}",
           index,
           uptime_seconds, // Actual uptime in seconds
           strlen(UAV->uav_id) > 0 ? UAV->uav_id : "NONE",
           UAV->ua_type,
           mac_str,
           UAV->rssi,
           lat,
           lon,
           UAV->speed,
           UAV->speed_vertical,
           UAV->altitude_msl,
           UAV->height_agl,
           UAV->status,
           UAV->heading,
           UAV->altitude_pressure,
           UAV->height_type,
           UAV->horizontal_accuracy,
           UAV->vertical_accuracy,
           UAV->baro_accuracy,
           UAV->speed_accuracy,
           UAV->timestamp,
           mac_str,
           UAV->desc_type,
           UAV->description,
           lat,
           lon,
           op_lat,
           op_lon,
           UAV->area_count,
           UAV->area_radius,
           UAV->area_ceiling,
           UAV->area_floor,
           UAV->operator_altitude_geo,
           UAV->classification_type,
           UAV->system_timestamp,
           UAV->auth_type,
           UAV->auth_page,
           UAV->auth_length,
           UAV->auth_timestamp,
           UAV->auth_data);
  
  Serial.println(json);
}

// Print compact mesh message
void print_compact_message(const uav_data *UAV) {
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 5000;  // 5-second interval
  
  if (millis() - lastSendTime < sendInterval)
    return;
  lastSendTime = millis();
  
  const int MAX_MESH_SIZE = 230;
  char mesh_msg[MAX_MESH_SIZE];
  int msg_len = 0;
  
  // Format MAC address
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           UAV->mac[0], UAV->mac[1], UAV->mac[2],
           UAV->mac[3], UAV->mac[4], UAV->mac[5]);
  
  msg_len += snprintf(mesh_msg + msg_len, sizeof(mesh_msg) - msg_len,
                     "Drone: %s RSSI:%d", mac_str, UAV->rssi);
  
  if (msg_len < MAX_MESH_SIZE && UAV->lat_d != 0.0 && UAV->long_d != 0.0) {
    msg_len += snprintf(mesh_msg + msg_len, sizeof(mesh_msg) - msg_len,
                       " https://maps.google.com/?q=%.6f,%.6f",
                       UAV->lat_d, UAV->long_d);
  }
  
  if (Serial1.availableForWrite() >= msg_len) {
    Serial1.println(mesh_msg);
  }
  
  delay(1000);
  
  if (UAV->base_lat_d != 0.0 && UAV->base_long_d != 0.0) {
    char pilot_msg[MAX_MESH_SIZE];
    int pilot_len = snprintf(pilot_msg, sizeof(pilot_msg),
                           "Pilot: https://maps.google.com/?q=%.6f,%.6f",
                           UAV->base_lat_d, UAV->base_long_d);
    if (Serial1.availableForWrite() >= pilot_len) {
      Serial1.println(pilot_msg);
    }
  }
}

// BLE scanning task - runs on Core 0
void bleScanTask(void *pvParameters) {
  for(;;) {
    // Start BLE scan
    pBLEScan->start(1, false);
    pBLEScan->clearResults();
    
    // Process any flagged UAVs
    for (int i = 0; i < MAX_UAVS; i++) {
      if (uavs[i].flag) {
        packetCount++;
        print_json(&uavs[i], packetCount);
         print_compact_message(&uavs[i]);
        uavs[i].flag = 0;
      }
    }
    
    // Periodic heartbeat
    unsigned long current_millis = millis();
    if ((current_millis - last_status) > 60000UL) {
      Serial.println("{\"heartbeat\":\"Device is active and running.\"}");
      last_status = current_millis;
    }
    
    delay(100);
  }
}

// WiFi processing task - runs on Core 1
void wifiProcessTask(void *pvParameters) {
  for(;;) {
    // Process any flagged UAVs (WiFi detections are flagged in the callback)
    for (int i = 0; i < MAX_UAVS; i++) {
      if (uavs[i].flag) {
        packetCount++;
        print_json(&uavs[i], packetCount);
        // print_compact_message(&uavs[i]);  uncomment to enable mesh messages over serial
        uavs[i].flag = 0;
      }
    }
    delay(10);
  }
}