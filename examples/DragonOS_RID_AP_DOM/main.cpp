/* -*- tab-width: 2; mode: c; -*-
 * DOM AP Scanner for WiFi direct remote id.
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
#include <WebServer.h>
#include <esp_timer.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include <map>
#include "opendroneid.h"
#include "odid_wifi.h"

static ODID_UAS_Data UAS_data;

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

struct node_info {
  uint8_t mac[6];
  unsigned long last_seen;
  uint32_t total_packets;
  float temperature;
  uint32_t uptime;
  bool is_online;
  char node_name[32];
};

const char* ap_ssid = "Drag0Net-Host";
const char* ap_password = "wardragon123";

WebServer server(80);
WiFiServer telemetryServer(4224);
WiFiServer statusServer(4225);

TaskHandle_t ScannerTask;
TaskHandle_t WebServerTask;
TaskHandle_t ZMQTask;
TaskHandle_t StatusTask;
TaskHandle_t NodeManagementTask;

static String latestDroneData = "";
static int packetCount = 0;
static unsigned long last_status = 0;
static SemaphoreHandle_t dataMutex;
static SemaphoreHandle_t nodeDataMutex;

std::vector<WiFiClient> telemetryClients;
static SemaphoreHandle_t clientMutex;

std::map<uint64_t, node_info> connectedNodes;
static const int MAX_NODES = 10;
static const unsigned long NODE_TIMEOUT_MS = 60000;

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

static esp_err_t scanner_event_handler(void *, system_event_t *);
static void callback(void *, wifi_promiscuous_pkt_type_t);
static void parse_odid(struct uav_data *, ODID_UAS_Data *);
static void parse_french_id(struct uav_data *, uint8_t *);
static void store_mac(struct uav_data *uav, uint8_t *payload);
static String create_json_response(struct uav_data *UAV, int index);
static void update_latest_data(struct uav_data *UAV, int index);
static void publishToZMQClients(const String& data, bool isStatus);
static String create_status_json();
static void sendZMTPGreeting(WiFiClient& client);
static void sendZMTPMessage(WiFiClient& client, const String& message);
static void publishToZMTPClients(const String& data);
static float getESP32Temperature();
static String create_host_html();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint64_t macInt = 0;
  for(int i = 0; i < 6; i++) {
    macInt = (macInt << 8) + mac[i];
  }
  
  if (len == sizeof(esp_now_drone_message)) {
    esp_now_drone_message* msg = (esp_now_drone_message*)incomingData;
    if (msg->msgType == MSG_DRONE_DATA) {
      
      char nodeMacStr[18];
      snprintf(nodeMacStr, sizeof(nodeMacStr), "%02x:%02x:%02x:%02x:%02x:%02x",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      
      Serial.println("HOST RECEIVED DRONE DATA from node: " + String(nodeMacStr));
      
      // Convert compact message back to full uav_data
      struct uav_data currentUAV;
      memset(&currentUAV, 0, sizeof(currentUAV));
      
      memcpy(currentUAV.mac, msg->mac, 6);
      currentUAV.rssi = msg->rssi;
      strncpy(currentUAV.uav_id, msg->uav_id, ODID_ID_SIZE);
      strncpy(currentUAV.op_id, msg->op_id, ODID_ID_SIZE);
      currentUAV.lat_d = msg->lat_d;
      currentUAV.long_d = msg->long_d;
      currentUAV.base_lat_d = msg->base_lat_d;
      currentUAV.base_long_d = msg->base_long_d;
      currentUAV.altitude_msl = msg->altitude_msl;
      currentUAV.height_agl = msg->height_agl;
      currentUAV.speed = msg->speed;
      currentUAV.heading = msg->heading;
      currentUAV.ua_type = msg->ua_type;
      currentUAV.timestamp = msg->timestamp;
      
      memcpy(currentUAV.source_node_mac, mac, 6);
      currentUAV.node_uptime = msg->nodeUptime;
      currentUAV.node_temperature = msg->nodeTemperature;
      currentUAV.node_packet_count = msg->nodePacketCount;
      
      packetCount++;
      update_latest_data(&currentUAV, packetCount);
      
      if (xSemaphoreTake(nodeDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (connectedNodes.find(macInt) != connectedNodes.end()) {
          connectedNodes[macInt].last_seen = millis();
          connectedNodes[macInt].total_packets++;
          connectedNodes[macInt].temperature = msg->nodeTemperature;
          connectedNodes[macInt].uptime = msg->nodeUptime;
          connectedNodes[macInt].is_online = true;
          strncpy(connectedNodes[macInt].node_name, msg->nodeName, sizeof(connectedNodes[macInt].node_name)-1);
        }
        xSemaphoreGive(nodeDataMutex);
      }
    }
  } else if (len == sizeof(esp_now_status_message)) {
    esp_now_status_message* msg = (esp_now_status_message*)incomingData;
    if (msg->msgType == MSG_NODE_STATUS || msg->msgType == MSG_NODE_REGISTER) {
      
      if (xSemaphoreTake(nodeDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        node_info& node = connectedNodes[macInt];
        memcpy(node.mac, mac, 6);
        node.last_seen = millis();
        node.temperature = msg->temperature;
        node.uptime = msg->uptime;
        node.total_packets = msg->totalPackets;
        node.is_online = true;
        strncpy(node.node_name, msg->nodeName, sizeof(node.node_name)-1);
        xSemaphoreGive(nodeDataMutex);
      }
      
      if (msg->msgType == MSG_NODE_REGISTER) {
        Serial.println("New node registered: " + String(msg->nodeName));
      }
    }
  }
}

void NodeManagementTaskCode(void *pvParameters) {
  Serial.println("Node management task starting on core " + String(xPortGetCoreID()));
  
  for(;;) {
    unsigned long currentTime = millis();
    
    if (xSemaphoreTake(nodeDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (auto& pair : connectedNodes) {
        if (currentTime - pair.second.last_seen > NODE_TIMEOUT_MS) {
          pair.second.is_online = false;
        }
      }
      xSemaphoreGive(nodeDataMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void ZMQTaskCode(void *pvParameters) {
  Serial.println("ZMQ server task starting on core " + String(xPortGetCoreID()));
  
  clientMutex = xSemaphoreCreateMutex();
  telemetryServer.begin();
  telemetryServer.setNoDelay(true);
  
  Serial.println("ZMTP server started on port 4224");
  
  for(;;) {
    WiFiClient newClient = telemetryServer.available();
    if (newClient) {
      Serial.println("New ZMQ client connected from " + newClient.remoteIP().toString());
      newClient.setNoDelay(true);
      sendZMTPGreeting(newClient);
      
      if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        telemetryClients.push_back(newClient);
        xSemaphoreGive(clientMutex);
      }
    }
    
    if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      for (auto it = telemetryClients.begin(); it != telemetryClients.end();) {
        if (!it->connected()) {
          it->stop();
          it = telemetryClients.erase(it);
          Serial.println("ZMQ client disconnected");
        } else {
          ++it;
        }
      }
      xSemaphoreGive(clientMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void StatusTaskCode(void *pvParameters) {
  Serial.println("Status server task starting on core " + String(xPortGetCoreID()));
  
  statusServer.begin();
  statusServer.setNoDelay(true);
  
  Serial.println("Status server started on port 4225");
  
  std::vector<WiFiClient> statusClients;
  
  for(;;) {
    WiFiClient newStatusClient = statusServer.available();
    if (newStatusClient) {
      Serial.println("New status client connected");
      newStatusClient.setNoDelay(true);
      sendZMTPGreeting(newStatusClient);
      statusClients.push_back(newStatusClient);
    }
    
    for (auto it = statusClients.begin(); it != statusClients.end();) {
      if (!it->connected()) {
        it->stop();
        it = statusClients.erase(it);
        Serial.println("Status client disconnected");
      } else {
        ++it;
      }
    }
    
    if (!statusClients.empty()) {
      String statusJson = create_status_json();
      for (auto& client : statusClients) {
        if (client.connected()) {
          sendZMTPMessage(client, statusJson);
        }
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

void ScannerTaskCode(void *pvParameters) {
  // Add country configuration first
  wifi_country_t country = {
    .cc = "US",
    .schan = 1,
    .nchan = 11,
    .max_tx_power = 84,
    .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  esp_wifi_set_country(&country);

  // Set promiscuous filters
  wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA
  };
  esp_wifi_set_promiscuous_filter(&filter);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&callback);

  for(;;) {
    // AP - maintain staiblity across nodes
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(700));

    // Scan for RID packets on host
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void WebServerTaskCode(void *pvParameters) {
  Serial.println("Web server task starting on core " + String(xPortGetCoreID()));
  
  
  // Initialize WiFi in AP+STA mode
  WiFi.mode(WIFI_AP_STA); 
  WiFi.softAP(ap_ssid, ap_password, 1); 
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback
  esp_now_register_recv_cb(OnDataRecv);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Host AP IP address: ");
  Serial.println(IP);
  Serial.println("Connect to WiFi: " + String(ap_ssid));
  Serial.println("Password: " + String(ap_password));
  Serial.println("Open browser to: http://" + IP.toString());
  Serial.println("ZMTP Telemetry: tcp://" + IP.toString() + ":4224");
  
  server.on("/", [](){
    String html = create_host_html();
    server.send(200, "text/html", html);
  });

  server.on("/data", [](){
    String response = "{";
    response += "\"host_packets\":" + String(packetCount) + ",";
    response += "\"host_uptime\":" + String(esp_timer_get_time() / 1000000UL) + ",";
    response += "\"host_memory\":" + String(ESP.getFreeHeap()) + ",";
    response += "\"host_memory_total\":" + String(ESP.getHeapSize()) + ",";
    response += "\"host_cpu_freq\":" + String(ESP.getCpuFreqMHz()) + ",";
    response += "\"host_flash_size\":" + String(ESP.getFlashChipSize()) + ",";
    response += "\"sdk_version\":\"" + String(ESP.getSdkVersion()) + "\",";
    
    float temp_c = getESP32Temperature();
    float temp_f = (temp_c * 9.0/5.0) + 32.0;
    response += "\"host_temperature\":" + String(temp_f, 1) + ",";
    
    response += "\"connected_nodes\":[";
    if (xSemaphoreTake(nodeDataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      bool firstNode = true;
      for (const auto& pair : connectedNodes) {
        if (!firstNode) response += ",";
        firstNode = false;
        
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
                 pair.second.mac[0], pair.second.mac[1], pair.second.mac[2],
                 pair.second.mac[3], pair.second.mac[4], pair.second.mac[5]);
        
        response += "{";
        response += "\"mac\":\"" + String(macStr) + "\",";
        response += "\"name\":\"" + String(pair.second.node_name) + "\",";
        response += "\"online\":" + String(pair.second.is_online ? "true" : "false") + ",";
        response += "\"packets\":" + String(pair.second.total_packets) + ",";
        response += "\"temperature\":" + String((pair.second.temperature * 9.0/5.0) + 32.0, 1) + ",";
        response += "\"uptime\":" + String(pair.second.uptime) + ",";
        response += "\"last_seen\":" + String((millis() - pair.second.last_seen) / 1000);
        response += "}";
      }
      xSemaphoreGive(nodeDataMutex);
    }
    response += "],";
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      String escapedData = latestDroneData;
      escapedData.replace("\"", "\\\"");
      escapedData.replace("\n", "\\n");
      escapedData.replace("\r", "\\r");
      response += "\"latestDrone\":\"" + escapedData + "\"";
      xSemaphoreGive(dataMutex);
    } else {
      response += "\"latestDrone\":\"Data locked\"";
    }
    
    response += "}";
    server.send(200, "application/json", response);
  });

  server.on("/stream", [](){
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      server.send(200, "application/json", latestDroneData);
      xSemaphoreGive(dataMutex);
    } else {
      server.send(503, "application/json", "{\"error\":\"Data unavailable\"}");
    }
  });

  server.begin();
  Serial.println("Host web server started successfully on core " + String(xPortGetCoreID()));
  
  for(;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("{ \"message\": \"Starting Drag0Net ESP32 HOST with ESP-NOW Network\" }");
  
  dataMutex = xSemaphoreCreateMutex();
  nodeDataMutex = xSemaphoreCreateMutex();
  delay(200);
  
  xTaskCreatePinnedToCore(
    WebServerTaskCode,
    "WebServerTask",
    8192,
    NULL,
    1,
    &WebServerTask,
    1
  );
  
  vTaskDelay(pdMS_TO_TICKS(3000));
  delay(200);
  
  xTaskCreatePinnedToCore(
    ZMQTaskCode,
    "ZMQTask",
    8192,
    NULL,
    1,
    &ZMQTask,
    1
  );

  xTaskCreatePinnedToCore(
    StatusTaskCode,
    "StatusTask",
    8192,
    NULL,
    1,
    &StatusTask,
    1
  );
  
  xTaskCreatePinnedToCore(
    NodeManagementTaskCode,
    "NodeManagementTask",  
    4096,
    NULL,
    1,
    &NodeManagementTask,
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

  store_mac(&currentUAV, payload + 4);
  currentUAV.rssi = packet->rx_ctrl.rssi;

  static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};

  if (memcmp(nan_dest, &payload[4], 6) == 0) {
    if (odid_wifi_receive_message_pack_nan_action_frame(&UAS_data, (char *)currentUAV.op_id, payload, length) == 0) {
      parse_odid(&currentUAV, &UAS_data);
      
      uint8_t hostMac[6];
      WiFi.macAddress(hostMac);
      memcpy(currentUAV.source_node_mac, hostMac, 6);
      currentUAV.node_uptime = esp_timer_get_time() / 1000000UL;
      currentUAV.node_temperature = getESP32Temperature();
      currentUAV.node_packet_count = packetCount;
      
      packetCount++;
      Serial.println("HOST DETECTED DRONE: Processing locally");
      update_latest_data(&currentUAV, packetCount);
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
          parse_french_id(&currentUAV, &payload[offset]);

          uint8_t hostMac[6];
          WiFi.macAddress(hostMac);
          memcpy(currentUAV.source_node_mac, hostMac, 6);
          currentUAV.node_uptime = esp_timer_get_time() / 1000000UL;
          currentUAV.node_temperature = getESP32Temperature();
          currentUAV.node_packet_count = packetCount;
          
          packetCount++;
          update_latest_data(&currentUAV, packetCount);
          printed = true;
        }
        else if ((typ == 0xdd) &&
                 (((val[0] == 0x90 && val[1] == 0x3a && val[2] == 0xe6)) ||
                  ((val[0] == 0xfa && val[1] == 0x0b && val[2] == 0xbc)))) {
          int j = offset + 7;
          if (j < length) {
            memset(&UAS_data, 0, sizeof(UAS_data));
            odid_message_process_pack(&UAS_data, &payload[j], length - j);
            parse_odid(&currentUAV, &UAS_data);

            uint8_t hostMac[6];
            WiFi.macAddress(hostMac);
            memcpy(currentUAV.source_node_mac, hostMac, 6);
            currentUAV.node_uptime = esp_timer_get_time() / 1000000UL;
            currentUAV.node_temperature = getESP32Temperature();
            currentUAV.node_packet_count = packetCount;
            
            packetCount++;
            Serial.println("HOST DETECTED DRONE: Processing locally");
            update_latest_data(&currentUAV, packetCount);
            printed = true;
          }
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

static String create_json_response(struct uav_data *UAV, int index) {
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", 
           UAV->mac[0], UAV->mac[1], UAV->mac[2], UAV->mac[3], UAV->mac[4], UAV->mac[5]);

  char source_mac_str[18];
  snprintf(source_mac_str, sizeof(source_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", 
           UAV->source_node_mac[0], UAV->source_node_mac[1], UAV->source_node_mac[2], 
           UAV->source_node_mac[3], UAV->source_node_mac[4], UAV->source_node_mac[5]);

  String lat = (UAV->lat_d != 0.0) ? String(UAV->lat_d, 6) : "0.0";
  String lon = (UAV->long_d != 0.0) ? String(UAV->long_d, 6) : "0.0";
 String op_lat = (UAV->base_lat_d != 0.0) ? String(UAV->base_lat_d, 6) : "0.0";
 String op_lon = (UAV->base_long_d != 0.0) ? String(UAV->base_long_d, 6) : "0.0";

 String json = "{";
 json += "\"index\":" + String(index) + ",";
 json += "\"timestamp\":" + String(millis()) + ",";
 json += "\"source_node\":\"" + String(source_mac_str) + "\",";
 json += "\"node_uptime\":" + String(UAV->node_uptime) + ",";
 json += "\"node_temperature\":" + String(UAV->node_temperature, 1) + ",";
 json += "\"node_packet_count\":" + String(UAV->node_packet_count) + ",";
 json += "\"Basic ID Message\":{";
 json += "\"id_type\":0,";
 json += "\"uas_id\":\"" + String(strlen(UAV->uav_id) > 0 ? UAV->uav_id : "NONE") + "\",";
 json += "\"id_type\":\"Serial Number (ANSI/CTA-2063-A)\",";
 json += "\"ua_type\":" + String(UAV->ua_type) + ",";
 json += "\"MAC\":\"" + String(mac_str) + "\",";
 json += "\"RSSI\":" + String(UAV->rssi);
 json += "},";
 json += "\"Location/Vector Message\":{";
 json += "\"latitude\":" + lat + ",";
 json += "\"longitude\":" + lon + ",";
 json += "\"speed\":" + String(UAV->speed) + ",";
 json += "\"vert_speed\":" + String(UAV->speed_vertical) + ",";
 json += "\"geodetic_altitude\":" + String(UAV->altitude_msl) + ",";
 json += "\"height_agl\":" + String(UAV->height_agl) + ",";
 json += "\"status\":" + String(UAV->status) + ",";
 json += "\"direction\":" + String(UAV->heading) + ",";
 json += "\"alt_pressure\":" + String(UAV->altitude_pressure) + ",";
 json += "\"height_type\":" + String(UAV->height_type) + ",";
 json += "\"horiz_acc\":" + String(UAV->horizontal_accuracy) + ",";
 json += "\"vert_acc\":" + String(UAV->vertical_accuracy) + ",";
 json += "\"baro_acc\":" + String(UAV->baro_accuracy) + ",";
 json += "\"speed_acc\":" + String(UAV->speed_accuracy) + ",";
 json += "\"timestamp\":" + String(UAV->timestamp);
 json += "},";
 json += "\"Self-ID Message\":{";
 json += "\"text\":\"UAV " + String(mac_str) + " operational\",";
 json += "\"description_type\":" + String(UAV->desc_type) + ",";
 json += "\"description\":\"" + String(UAV->description) + "\"";
 json += "},";
 json += "\"System Message\":{";
 json += "\"latitude\":" + op_lat + ",";
 json += "\"longitude\":" + op_lon + ",";
 json += "\"operator_lat\":" + op_lat + ",";
 json += "\"operator_lon\":" + op_lon + ",";
 json += "\"area_count\":" + String(UAV->area_count) + ",";
 json += "\"area_radius\":" + String(UAV->area_radius) + ",";
 json += "\"area_ceiling\":" + String(UAV->area_ceiling) + ",";
 json += "\"area_floor\":" + String(UAV->area_floor) + ",";
 json += "\"operator_alt_geo\":" + String(UAV->operator_altitude_geo) + ",";
 json += "\"classification\":" + String(UAV->classification_type) + ",";
 json += "\"timestamp\":" + String(UAV->system_timestamp);
 json += "},";
 json += "\"Auth Message\":{";
 json += "\"type\":" + String(UAV->auth_type) + ",";
 json += "\"page\":" + String(UAV->auth_page) + ",";
 json += "\"length\":" + String(UAV->auth_length) + ",";
 json += "\"timestamp\":" + String(UAV->auth_timestamp) + ",";
 json += "\"data\":\"" + String(UAV->auth_data) + "\"";
 json += "}";
 json += "}";

 return json;
}

static void update_latest_data(struct uav_data *UAV, int index) {
 String json = create_json_response(UAV, index);
 
 if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
   latestDroneData = json;
   xSemaphoreGive(dataMutex);
 }
 
 publishToZMTPClients(json);
}

static void sendZMTPGreeting(WiFiClient& client) {
  if (!client.connected()) return;
  
  // ZMTP greeting: anonymous identity (empty string)
  uint8_t greeting[2] = {0x01, 0x00}; // length=1, flags=0 (final frame)
  client.write(greeting, 2);
  client.flush();
}

static void sendZMTPMessage(WiFiClient& client, const String& message) {
  if (!client.connected()) return;
  
  uint32_t msgLen = message.length();
  
  // ZMTP frame format: [length][flags][body]
  if (msgLen < 255) {
    // Short frame: length as single byte
    uint8_t shortLen = (uint8_t)(msgLen + 1); // +1 for flags byte
    uint8_t flags = 0x00; // Final frame, no more frames
    
    client.write(&shortLen, 1);
    client.write(&flags, 1);
    client.write((const uint8_t*)message.c_str(), msgLen);
  } else {
    // Long frame: 0xFF + 8-byte length
    uint8_t longMarker = 0xFF;
    uint64_t longLen = msgLen + 1; // +1 for flags byte
    uint8_t flags = 0x00; // Final frame
    
    client.write(&longMarker, 1);
    
    // Write 64-bit length in network byte order
    for (int i = 7; i >= 0; i--) {
      uint8_t byte = (longLen >> (i * 8)) & 0xFF;
      client.write(&byte, 1);
    }
    
    client.write(&flags, 1);
    client.write((const uint8_t*)message.c_str(), msgLen);
  }
  
  client.flush();
}

static void publishToZMTPClients(const String& data) {
 if (xSemaphoreTake(clientMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
   for (auto it = telemetryClients.begin(); it != telemetryClients.end();) {
     if (it->connected()) {
       sendZMTPMessage(*it, data);
       ++it;
     } else {
       it->stop();
       it = telemetryClients.erase(it);
     }
   }
   xSemaphoreGive(clientMutex);
 }
}

static String create_status_json() {
 unsigned long uptime_seconds = esp_timer_get_time() / 1000000UL;
 float esp32_temp_c = getESP32Temperature();
 float esp32_temp_f = (esp32_temp_c * 9.0/5.0) + 32.0;
 
 String json = "{";
 json += "\"device_type\":\"host\",";
 json += "\"serial_number\":\"ESP32-DragonHost\",";
 json += "\"system_stats\":{";
 json += "\"memory\":{";
 json += "\"total\":" + String(ESP.getHeapSize()) + ",";
 json += "\"available\":" + String(ESP.getFreeHeap()) + ",";
 json += "\"used\":" + String(ESP.getHeapSize() - ESP.getFreeHeap()) + ",";
 json += "\"percent\":" + String((float)(ESP.getHeapSize() - ESP.getFreeHeap()) / ESP.getHeapSize() * 100.0, 1);
 json += "},";
 json += "\"uptime\":" + String(uptime_seconds) + ",";
 json += "\"esp32_temperature_c\":" + String(esp32_temp_c, 1) + ",";
 json += "\"esp32_temperature_f\":" + String(esp32_temp_f, 1) + ",";
 json += "\"temperature\":" + String(esp32_temp_c, 1) + ",";
 json += "\"packet_count\":" + String(packetCount) + ",";
 json += "\"cpu_freq\":" + String(ESP.getCpuFreqMHz()) + ",";
 json += "\"flash_size\":" + String(ESP.getFlashChipSize() / 1024) + ",";
 json += "\"sdk_version\":\"" + String(ESP.getSdkVersion()) + "\",";
 json += "\"connected_nodes\":" + String(connectedNodes.size());
 json += "}";
 json += "}";
 
 return json;
}

static float getESP32Temperature() {
  return temperatureRead();
}

static String create_host_html() {
 return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Drag0Net Host Network</title>
    <style>
        :root {
            --bg-color: #0a0a0a;
            --text-color: #00ff41;
            --accent-color: #00ff41;
            --panel-bg: #111111;
            --panel-border: #00ff41;
            --header-color: #00ff41;
            --data-color: #00ff41;
            --warning-color: #ff3e3e;
            --node-online: #00ff41;
            --node-offline: #ff3e3e;
        }
        
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            font-family: 'Courier New', monospace;
        }
        
        body {
            background-color: var(--bg-color);
            color: var(--text-color);
            padding: 20px;
            line-height: 1.6;
            background-image: 
                radial-gradient(rgba(0, 255, 65, 0.1) 1px, transparent 1px),
                radial-gradient(rgba(0, 255, 65, 0.1) 1px, transparent 1px);
            background-size: 50px 50px;
            background-position: 0 0, 25px 25px;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        
        .header {
            text-align: center;
            margin-bottom: 30px;
            border-bottom: 1px solid var(--accent-color);
            padding-bottom: 15px;
            position: relative;
        }
        
        .header h1 {
            font-size: 2.5rem;
            letter-spacing: 2px;
            text-transform: uppercase;
            margin-bottom: 5px;
            text-shadow: 0 0 10px var(--accent-color);
        }
        
        .header p {
            font-size: 1rem;
            opacity: 0.8;
        }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .panel {
            background-color: var(--panel-bg);
            border: 1px solid var(--panel-border);
            border-radius: 4px;
            padding: 20px;
            box-shadow: 0 0 10px rgba(0, 255, 65, 0.2);
            position: relative;
            overflow: hidden;
        }
        
        .panel::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            width: 100%;
            height: 2px;
            background: linear-gradient(90deg, transparent, var(--accent-color), transparent);
        }
        
        .panel h2 {
            margin-bottom: 15px;
            font-size: 1.2rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            border-bottom: 1px solid rgba(0, 255, 65, 0.3);
            padding-bottom: 5px;
        }
        
        .stat {
            display: flex;
            justify-content: space-between;
            margin-bottom: 10px;
            font-size: 0.9rem;
        }
        
        .stat-label {
            opacity: 0.8;
        }
        
        .stat-value {
            font-weight: bold;
        }
        
        .drone-data {
            background-color: var(--panel-bg);
            border: 1px solid var(--panel-border);
            border-radius: 4px;
            padding: 20px;
            font-family: 'Courier New', monospace;
            white-space: pre-wrap;
            max-height: 400px;
            overflow-y: auto;
            margin-bottom: 20px;
            box-shadow: 0 0 10px rgba(0, 255, 65, 0.2);
        }
        
        .drone-data::-webkit-scrollbar {
            width: 8px;
        }
        
        .drone-data::-webkit-scrollbar-track {
            background: var(--panel-bg);
        }
        
        .drone-data::-webkit-scrollbar-thumb {
            background-color: var(--accent-color);
            border-radius: 4px;
        }
        
        .button {
            background-color: transparent;
            color: var(--accent-color);
            border: 1px solid var(--accent-color);
            padding: 10px 20px;
            cursor: pointer;
            font-size: 0.9rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            transition: all 0.3s ease;
            border-radius: 4px;
            margin-bottom: 20px;
        }
        
        .button:hover {
            background-color: var(--accent-color);
            color: var(--bg-color);
            box-shadow: 0 0 15px var(--accent-color);
        }
        
        .connection-info {
            margin-bottom: 20px;
        }
        
        .connection-info h2 {
            margin-bottom: 15px;
            font-size: 1.2rem;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .connection-detail {
            display: flex;
            margin-bottom: 10px;
        }
        
        .connection-label {
            width: 150px;
            opacity: 0.8;
        }
        
        .connection-value {
            font-weight: bold;
        }
        
        .blink {
            animation: blink 1.5s infinite;
        }
        
        @keyframes blink {
            0% { opacity: 1; }
            50% { opacity: 0.3; }
            100% { opacity: 1; }
        }
        
        .status-indicator {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 10px;
            background-color: var(--accent-color);
            box-shadow: 0 0 10px var(--accent-color);
        }
        
        .status-active {
            animation: pulse 1.5s infinite;
        }
        
        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(0, 255, 65, 0.7); }
            70% { box-shadow: 0 0 0 10px rgba(0, 255, 65, 0); }
            100% { box-shadow: 0 0 0 0 rgba(0, 255, 65, 0); }
        }
        
        .node-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        
        .node-card {
            background-color: rgba(0, 0, 0, 0.3);
            border: 1px solid rgba(0, 255, 65, 0.3);
            border-radius: 4px;
            padding: 15px;
            position: relative;
        }
        
        .node-status {
            position: absolute;
            top: 10px;
            right: 10px;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            box-shadow: 0 0 10px currentColor;
        }
        
        .node-online {
            background-color: var(--node-online);
            color: var(--node-online);
        }
        
        .node-offline {
            background-color: var(--node-offline);
            color: var(--node-offline);
        }
        
        .node-name {
            font-size: 1.1rem;
            font-weight: bold;
            margin-bottom: 10px;
        }
        
        .node-mac {
            font-size: 0.8rem;
            opacity: 0.7;
            margin-bottom: 10px;
        }
        
        .footer {
            text-align: center;
            margin-top: 30px;
            font-size: 0.8rem;
            opacity: 0.6;
        }
        
        @media (max-width: 768px) {
            .grid {
                grid-template-columns: 1fr;
            }
            .node-grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Drag0Net Host Network</h1>
            <p>WiFi RID with ESP-NOW Nodes & ZMQ Feed</p>
        </div>
        
        <div class="panel connection-info">
            <h2>Host Connection Info</h2>
            <div class="connection-detail">
                <div class="connection-label">AP SSID:</div>
                <div class="connection-value">Drag0Net-Host</div>
            </div>
            <div class="connection-detail">
                <div class="connection-label">IP Address:</div>
                <div class="connection-value">192.168.4.1</div>
            </div>
            <div class="connection-detail">
                <div class="connection-label">Telemetry Port:</div>
                <div class="connection-value">4224</div>
            </div>
            <div class="connection-detail">
                <div class="connection-label">Status Port:</div>
                <div class="connection-value">4225</div>
            </div>
        </div>
        
        <div class="grid">
            <div class="panel">
                <h2>Host System Status</h2>
                <div class="stat">
                    <div class="stat-label">Status:</div>
                    <div class="stat-value"><span class="status-indicator status-active"></span>Host Active</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Temperature:</div>
                    <div class="stat-value" id="host-temperature">--°F</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Free Memory:</div>
                    <div class="stat-value" id="host-memory">-- KB</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Packets Received:</div>
                    <div class="stat-value" id="host-packets">--</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Connected Nodes:</div>
                    <div class="stat-value" id="node-count">--</div>
                </div>
            </div>
            
            <div class="panel">
                <h2>Detection Stats</h2>
                <div class="stat">
                    <div class="stat-label">Packets Detected:</div>
                    <div class="stat-value" id="packets">0</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Last Detection:</div>
                    <div class="stat-value" id="last-detection">Never</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Detection Rate:</div>
                    <div class="stat-value" id="detection-rate">0 pkts/min</div>
                </div>
            </div>
            
            <div class="panel">
                <h2>System Info</h2>
                <div class="stat">
                    <div class="stat-label">Uptime:</div>
                    <div class="stat-value" id="uptime">--:--:--</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Flash Size:</div>
                    <div class="stat-value" id="flash-size">-- KB</div>
                </div>
                <div class="stat">
                    <div class="stat-label">Memory Usage:</div>
                    <div class="stat-value" id="memory-usage">--%</div>
                </div>
            </div>
        </div>
        
        <div class="panel">
            <h2>Connected Nodes</h2>
            <div id="nodes-container">
                <div class="node-grid" id="nodes-grid">
                    <div style="text-align: center; padding: 20px; opacity: 0.6;">
                        No nodes connected yet
                    </div>
                </div>
            </div>
        </div>
        
        <button class="button" onclick="refreshData()">Refresh Data</button>
        
        <div class="panel">
            <h2>Latest Drone Data</h2>
            <div class="drone-data" id="droneData">No drone data received yet</div>
        </div>
        
        <div class="footer">
            Drag0Net Host Network v1.0 | ESP32 WiFi Remote ID Scanner with ESP-NOW
        </div>
    </div>

    <script>
        let lastPacketCount = 0;
        let lastPacketTime = Date.now();
        let detectionRate = 0;
        
        function formatUptime(seconds) {
            const hours = Math.floor(seconds / 3600);
            const minutes = Math.floor((seconds % 3600) / 60);
            const secs = seconds % 60;
            return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
        }
        
        function formatBytes(bytes, decimals = 2) {
            if (bytes === 0) return '0 Bytes';
            
            const k = 1024;
            const dm = decimals < 0 ? 0 : decimals;
            const sizes = ['Bytes', 'KB', 'MB', 'GB'];
            
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            
            return parseFloat((bytes / Math.pow(k, i)).toFixed(dm)) + ' ' + sizes[i];
        }
        
        function refreshData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    const currentPackets = parseInt(data.host_packets);
                    const now = Date.now();
                    
                    // Update packet detection rate
                    if (currentPackets > lastPacketCount) {
                        const newPackets = currentPackets - lastPacketCount;
                        const timeElapsed = (now - lastPacketTime) / 1000 / 60; // in minutes
                        detectionRate = newPackets / timeElapsed;
                        document.getElementById('last-detection').textContent = 'Just now';
                    }
                    
                    lastPacketCount = currentPackets;
                    lastPacketTime = now;
                    
                    // Update UI elements
                    document.getElementById('host-temperature').textContent = data.host_temperature + '°F';
                    document.getElementById('host-memory').textContent = Math.round(data.host_memory / 1024) + ' KB';
                    document.getElementById('host-packets').textContent = data.host_packets;
                    document.getElementById('node-count').textContent = data.connected_nodes.length;
                    document.getElementById('packets').textContent = data.host_packets;
                    document.getElementById('uptime').textContent = formatUptime(data.host_uptime);
                    document.getElementById('detection-rate').textContent = detectionRate.toFixed(1) + ' pkts/min';
                    document.getElementById('memory-usage').textContent = ((data.host_memory_total - data.host_memory) / data.host_memory_total * 100).toFixed(1) + '%';
                    document.getElementById('flash-size').textContent = formatBytes(data.host_flash_size);
                    
                    const nodesGrid = document.getElementById('nodes-grid');
                    if (data.connected_nodes.length > 0) {
                        nodesGrid.innerHTML = '';
                        data.connected_nodes.forEach(node => {
                            const nodeCard = document.createElement('div');
                            nodeCard.className = 'node-card';
                            nodeCard.innerHTML = `
                                <div class="node-status ${node.online ? 'node-online' : 'node-offline'}"></div>
                                <div class="node-name">${node.name || 'Unknown Node'}</div>
                                <div class="node-mac">${node.mac}</div>
                                <div class="stat">
                                    <div class="stat-label">Status:</div>
                                    <div class="stat-value">${node.online ? 'Online' : 'Offline'}</div>
                                </div>
                                <div class="stat">
                                    <div class="stat-label">Packets:</div>
                                    <div class="stat-value">${node.packets}</div>
                                </div>
                                <div class="stat">
                                    <div class="stat-label">Temperature:</div>
                                    <div class="stat-value">${node.temperature}°F</div>
                                </div>
                                <div class="stat">
                                    <div class="stat-label">Last Seen:</div>
                                    <div class="stat-value">${node.last_seen}s ago</div>
                                </div>
                            `;
                            nodesGrid.appendChild(nodeCard);
                        });
                    } else {
                        nodesGrid.innerHTML = '<div style="text-align: center; padding: 20px; opacity: 0.6;">No nodes connected yet</div>';
                    }
                    
                    if (data.latestDrone && data.latestDrone !== 'No drone data received yet') {
                        document.getElementById('droneData').textContent = data.latestDrone;
                    }
                })
                .catch(error => {
                    console.error('Error:', error);
                });
        }
        
        // Initial refresh
        refreshData();
        
        // Set up periodic refresh
        setInterval(refreshData, 2000);
    </script>
</body>
</html>
)rawliteral";
}