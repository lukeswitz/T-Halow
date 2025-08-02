/* -*- tab-width: 2; mode: c; -*-
 * SUB AP for WiFi direct remote id.
 * Handles both opendroneid and French formats.
 *
 * Copyright (c) 2020-2021, Steve Jack.
 *
 * MIT licence.
 *
 * CEMAXECUTER
 * Minimal scanner for WiFi direct remote ID (OpenDroneID and French formats).
 * Prints results in the same JSON format as originally shown, immediately upon decode.
 *
 *  * MIT License.
 *
 * Luke Switzer
 * July '25 AP and Dual RID FW adaptations
 */

#if !defined(ARDUINO_ARCH_ESP32)
#error "This program requires an ESP32"
#endif

#include <Arduino.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <WiFi.h>
#include <esp_timer.h>
#include <esp_now.h>
#include "opendroneid.h"
#include "odid_wifi.h"

static ODID_UAS_Data UAS_data;
static SemaphoreHandle_t espNowChannelMutex;

struct uav_data
{
  uint8_t mac[6];
  uint8_t padding[1];
  int8_t rssi;  
  char op_id[ODID_ID_SIZE + 1];
  char uav_id[ODID_ID_SIZE + 1];
  double lat_d;
  double long_d;
  double base_lat_d;
  double base_long_d;
  int altitude_msl;
  int height_agl;
  int speed;
  int heading;
  int speed_vertical;
  int altitude_pressure;
  int horizontal_accuracy;
  int vertical_accuracy;
  int baro_accuracy;
  int speed_accuracy;
  int timestamp;
  int status;
  int height_type;
  int operator_location_type;
  int classification_type;
  int area_count;
  int area_radius;
  int area_ceiling;
  int area_floor;
  int operator_altitude_geo;
  uint32_t system_timestamp;
  int operator_id_type;
  uint8_t ua_type;
  uint8_t auth_type;
  uint8_t auth_page;
  uint8_t auth_length;
  uint32_t auth_timestamp;
  char auth_data[ODID_AUTH_PAGE_NONZERO_DATA_SIZE + 1];
  uint8_t desc_type;
  char description[ODID_STR_SIZE + 1];
  uint8_t source_node_mac[6];
  uint32_t node_uptime;
  float node_temperature;
  uint32_t node_packet_count;
};

uint8_t hostMacAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const String NODE_NAME = "Drag0Net-Node-" + String(random(1000, 9999));

TaskHandle_t ScannerTask;
TaskHandle_t StatusTask;

static int packetCount = 0;
static unsigned long last_status = 0;
static unsigned long last_status_send = 0;
static bool hostFound = false;

typedef struct {
  uint8_t msgType;
  uint8_t mac[6];
  int8_t rssi;
  char uav_id[21];
  char op_id[21]; 
  double lat_d;
  double long_d;
  double base_lat_d;
  double base_long_d;
  int altitude_msl;
  int height_agl;
  int speed;
  int heading;
  uint8_t ua_type;
  uint32_t timestamp;
  char nodeName[32];
  uint32_t nodeUptime;
  float nodeTemperature;
  uint32_t nodePacketCount;
} esp_now_drone_message;

typedef struct {
  uint8_t msgType;
  char nodeName[32];
  uint32_t uptime;
  float temperature;
  uint32_t totalPackets;
} esp_now_status_message;

enum MessageType {
  MSG_DRONE_DATA = 1,
  MSG_NODE_STATUS = 2,
  MSG_NODE_REGISTER = 3
};

static void callback(void *, wifi_promiscuous_pkt_type_t);
static void parse_odid(struct uav_data *, ODID_UAS_Data *);
static void parse_french_id(struct uav_data *, uint8_t *);
static void store_mac(struct uav_data *uav, uint8_t *payload);
static float getESP32Temperature();
static void sendDroneData(struct uav_data *UAV);
static void sendStatusMessage();
static int findHost();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    // Serial.println("Data sent successfully");
  } else {
    Serial.println("Failed to send data");
  }
}

static void sendDroneData(struct uav_data *UAV) {
  if (xSemaphoreTake(espNowChannelMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    esp_now_drone_message msg;
    msg.msgType = MSG_DRONE_DATA;
    
    memcpy(msg.mac, UAV->mac, 6);
    msg.rssi = UAV->rssi;
    strncpy(msg.uav_id, UAV->uav_id, 20);
    msg.uav_id[20] = '\0';
    strncpy(msg.op_id, UAV->op_id, 20);
    msg.op_id[20] = '\0';
    msg.lat_d = UAV->lat_d;
    msg.long_d = UAV->long_d;
    msg.base_lat_d = UAV->base_lat_d;
    msg.base_long_d = UAV->base_long_d;
    msg.altitude_msl = UAV->altitude_msl;
    msg.height_agl = UAV->height_agl;
    msg.speed = UAV->speed;
    msg.heading = UAV->heading;
    msg.ua_type = UAV->ua_type;
    msg.timestamp = UAV->timestamp;
    
    msg.nodeUptime = esp_timer_get_time() / 1000000UL;
    msg.nodeTemperature = getESP32Temperature();
    msg.nodePacketCount = packetCount;
    NODE_NAME.toCharArray(msg.nodeName, sizeof(msg.nodeName));
    
    esp_err_t result = esp_now_send(hostMacAddress, (uint8_t *) &msg, sizeof(msg));
    if (result == ESP_OK) {
      Serial.println("Drone data sent successfully");
    } else {
      Serial.println("Error sending drone data: " + String(result));
    }
    
    xSemaphoreGive(espNowChannelMutex);
  } else {
    Serial.println("Failed to acquire channel mutex for drone data");
  }
}

static void sendStatusMessage() {
  esp_now_status_message msg;
  msg.msgType = MSG_NODE_STATUS;
  NODE_NAME.toCharArray(msg.nodeName, sizeof(msg.nodeName));
  msg.uptime = esp_timer_get_time() / 1000000UL;
  msg.temperature = getESP32Temperature();
  msg.totalPackets = packetCount;
  
  esp_err_t result = esp_now_send(hostMacAddress, (uint8_t *) &msg, sizeof(msg));
  if (result != ESP_OK) {
    Serial.println("Error sending status message: " + String(result));
  }
}

static int findHost() {
  Serial.println("Scanning for Drag0Net host...");
  
  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  
  for (int i = 0; i < n; ++i) {
    if (WiFi.SSID(i) == "Drag0Net-Host") {
      uint8_t* bssid = WiFi.BSSID(i);
      memcpy(hostMacAddress, bssid, 6);
      
      int hostChannel = 6;
          
      Serial.printf("Found host at MAC: %02X:%02X:%02X:%02X:%02X:%02X on Channel %d\n",
                    hostMacAddress[0], hostMacAddress[1], hostMacAddress[2],
                    hostMacAddress[3], hostMacAddress[4], hostMacAddress[5], hostChannel);
      return hostChannel;  // Return the actual channel, TODO maybe make this dynamic, hardcoded for now 
    }
  }
  
  Serial.println("Host not found, using broadcast");
  memset(hostMacAddress, 0xFF, 6);
  return 1;  // Default to channel 1 if not found
}

void StatusTaskCode(void *pvParameters) {
  Serial.println("Node status task starting on core " + String(xPortGetCoreID()));
  
  for(;;) {
    unsigned long current_millis = millis();
    
    if ((current_millis - last_status_send) > 30000UL) {
      if (xSemaphoreTake(espNowChannelMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
        vTaskDelay(pdMS_TO_TICKS(10));
        sendStatusMessage();
        xSemaphoreGive(espNowChannelMutex);
        last_status_send = current_millis;
      } else {
        Serial.println("Failed to acquire channel mutex for status send");
      }
    }
    
    if ((current_millis - last_status) > 60000UL) {
      Serial.println("Node heartbeat: Scanner active, packets: " + String(packetCount));
      last_status = current_millis;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void ScannerTaskCode(void *pvParameters) {
  // Node channel configuration to match host
  wifi_country_t country = {
    .cc = "US",
    .schan = 1,
    .nchan = 11,
    .max_tx_power = 84,
    .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  esp_wifi_set_country(&country);

  for(;;) {
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(700));
    
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("{ \"message\": \"Starting Drag0Net ESP32 NODE with ESP-NOW\" }");
  Serial.println("Node Name: " + NODE_NAME);
  
  WiFi.mode(WIFI_STA);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  espNowChannelMutex = xSemaphoreCreateMutex();
  if (espNowChannelMutex == NULL) {
      Serial.println("Error creating espNowChannelMutex. System halted.");
      while(true); // Halt if mutex creation fails
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  int hostChannel = 1;

  // Add peer for ESP-NOW communication on the discovered channel
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, hostMacAddress, 6);
  peerInfo.channel = hostChannel; 
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&callback);
  
  // Add small delay for peer establishment
  delay(100);

  esp_now_status_message registerMsg;
  registerMsg.msgType = MSG_NODE_REGISTER;
  NODE_NAME.toCharArray(registerMsg.nodeName, sizeof(registerMsg.nodeName));
  registerMsg.uptime = esp_timer_get_time() / 1000000UL;
  registerMsg.temperature = getESP32Temperature();
  registerMsg.totalPackets = 0;
  
  esp_now_send(hostMacAddress, (uint8_t *) &registerMsg, sizeof(registerMsg));
  Serial.println("Registration message sent to host");
  
  delay(200);
  
  xTaskCreatePinnedToCore(
    StatusTaskCode,
    "StatusTask",
    4096,
    NULL,
    1,
    &StatusTask,
    1
  );
  
  vTaskDelay(pdMS_TO_TICKS(3000));
  delay(200);
  
  xTaskCreatePinnedToCore(
    ScannerTaskCode,
    "ScannerTask",
    8192,
    NULL,
    2,
    &ScannerTask,
    0
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(5000));
  delay(10);
}

static void callback(void *buffer, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT)
    return;

  wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buffer;
  uint8_t *payload = packet->payload;
  int length = packet->rx_ctrl.sig_len;

  struct uav_data currentUAV;
  memset(&currentUAV, 0, sizeof(currentUAV));

  store_mac(&currentUAV, payload);
  currentUAV.rssi = packet->rx_ctrl.rssi;

  static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};

  if (memcmp(nan_dest, &payload[4], 6) == 0) {
    if (odid_wifi_receive_message_pack_nan_action_frame(&UAS_data, (char *)currentUAV.op_id, payload, length) == 0) {
      parse_odid(&currentUAV, &UAS_data);
      
      uint8_t nodeMac[6];
      WiFi.macAddress(nodeMac);
      memcpy(currentUAV.source_node_mac, nodeMac, 6);
      currentUAV.node_uptime = esp_timer_get_time() / 1000000UL;
      currentUAV.node_temperature = getESP32Temperature();
      currentUAV.node_packet_count = packetCount;
      
      packetCount++;
      Serial.println("NODE DETECTED DRONE: Sending to host via ESP-NOW");
      sendDroneData(&currentUAV);
    }
  }
  else if (payload[0] == 0x80) {
    int offset = 36;
    bool printed = false;

    while (offset < length) {
      int typ = payload[offset];
      int len = payload[offset + 1];
      uint8_t *val = &payload[offset + 2];

      if (!printed) {
        if ((typ == 0xdd) && (val[0] == 0x6a) && (val[1] == 0x5c) && (val[2] == 0x35)) {
          parse_french_id(&currentUAV, val);
          
          uint8_t nodeMac[6];
          WiFi.macAddress(nodeMac);
          memcpy(currentUAV.source_node_mac, nodeMac, 6);
          currentUAV.node_uptime = esp_timer_get_time() / 1000000UL;
          currentUAV.node_temperature = getESP32Temperature();
          currentUAV.node_packet_count = packetCount;
          
          packetCount++;
          Serial.println("NODE DETECTED DRONE: Sending to host via ESP-NOW");
          sendDroneData(&currentUAV);
          printed = true;
        }
      }
      
      offset += len + 2;
      if (offset >= length) break;
    }
  }
}

static void store_mac(struct uav_data *uav, uint8_t *payload) {
  memcpy(uav->mac, &payload[10], 6);
}

static void parse_odid(struct uav_data *UAV, ODID_UAS_Data *UAS_data2) {
  if (UAS_data2->BasicIDValid[0]) {
    UAV->ua_type = UAS_data2->BasicID[0].UAType;
    strncpy(UAV->uav_id, (char *)UAS_data2->BasicID[0].UASID, ODID_ID_SIZE);
  }

  if (UAS_data2->LocationValid) {
    UAV->lat_d = UAS_data2->Location.Latitude;
    UAV->long_d = UAS_data2->Location.Longitude;
    UAV->altitude_msl = UAS_data2->Location.AltitudeGeo;
    UAV->height_agl = UAS_data2->Location.Height;
    UAV->speed = UAS_data2->Location.SpeedHorizontal;
    UAV->heading = UAS_data2->Location.Direction;
    UAV->speed_vertical = UAS_data2->Location.SpeedVertical;
    UAV->altitude_pressure = UAS_data2->Location.AltitudeBaro;
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

static void parse_french_id(struct uav_data *UAV, uint8_t *payload) {
  union {
    uint32_t u32;
    int32_t i32;
  } uav_lat, uav_long, base_lat, base_long;
  
  union {
    uint16_t u16;
    int16_t i16;
  } alt, height;

  int j = 9;
  int frame_length = payload[1];

  while (j < frame_length) {
    uint8_t t = payload[j];
    uint8_t l = payload[j + 1];
    uint8_t *v = &payload[j + 2];

    switch (t) {
      case 1:
        strncpy(UAV->uav_id, (char *)v, l);
        UAV->uav_id[l] = '\0';
        break;

      case 4:
        if (l >= 4) {
          uav_lat.u32 = (v[0] << 24) | (v[1] << 16) | (v[2] << 8) | v[3];
          UAV->lat_d = uav_lat.i32 / 10000000.0;
        }
        if (l >= 8) {
          uav_long.u32 = (v[4] << 24) | (v[5] << 16) | (v[6] << 8) | v[7];
          UAV->long_d = uav_long.i32 / 10000000.0;
        }
        break;

      case 5:
        if (l >= 2) {
          alt.u16 = (v[0] << 8) | v[1];
          UAV->altitude_msl = alt.i16;
        }
        break;

      case 6:
        if (l >= 2) {
          height.u16 = (v[0] << 8) | v[1];
          UAV->height_agl = height.i16;
        }
        break;

      case 7:
        if (l >= 1) UAV->speed = v[0];
        break;

      case 8:
        if (l >= 2) {
          UAV->heading = (v[0] << 8) | v[1];
        }
        break;
    }

    j += l + 2;
  }
}

static float getESP32Temperature() {
  return temperatureRead();
}