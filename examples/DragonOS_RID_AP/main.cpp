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
#include <WiFiUdp.h>
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
};

const char* ap_ssid = "WarDragon-Scanner";
const char* ap_password = "wardragon123";

WebServer server(80);
WiFiUDP udpTelemetry;
WiFiUDP udpStatus;

IPAddress multicastIP(239, 2, 3, 1);
const int telemetryPort = 4224;
const int statusPort = 4225;

TaskHandle_t ScannerTask;
TaskHandle_t WebServerTask;
TaskHandle_t MulticastTask;

static String latestDroneData = "";
static int packetCount = 0;
static unsigned long last_status = 0;
static SemaphoreHandle_t dataMutex;

static esp_err_t scanner_event_handler(void *, system_event_t *);
static void callback(void *, wifi_promiscuous_pkt_type_t);
static void parse_odid(struct uav_data *, ODID_UAS_Data *);
static void parse_french_id(struct uav_data *, uint8_t *);
static void store_mac(struct uav_data *uav, uint8_t *payload);
static String create_json_response(struct uav_data *UAV, int index);
static String create_cot_xml(struct uav_data *UAV, int index);
static void update_latest_data(struct uav_data *UAV, int index);
static void publishMulticast(const String& data, bool isStatus);
static String create_status_json();
static String create_status_cot();

void MulticastTaskCode(void *pvParameters) {
  Serial.println("Multicast task starting on core " + String(xPortGetCoreID()));
  
  // Initialize UDP for multicast
  udpTelemetry.begin(telemetryPort);
  udpStatus.begin(statusPort);
  
  Serial.println("Multicast servers started");
  Serial.println("Telemetry multicast: " + multicastIP.toString() + ":" + String(telemetryPort));
  Serial.println("Status multicast: " + multicastIP.toString() + ":" + String(statusPort));
  
  unsigned long lastStatusPublish = 0;
  
  for(;;) {
    unsigned long currentTime = millis();
    
    // Send status updates every 30 seconds
    if (currentTime - lastStatusPublish > 30000) {
      String statusCot = create_status_cot();
      publishMulticast(statusCot, true);
      lastStatusPublish = currentTime;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void publishMulticast(const String& data, bool isStatus) {
  WiFiUDP& udp = isStatus ? udpStatus : udpTelemetry;
  int port = isStatus ? statusPort : telemetryPort;
  
  udp.beginPacket(multicastIP, port);
  udp.print(data);
  udp.endPacket();
  
  Serial.println("Sent " + String(isStatus ? "status" : "telemetry") + " multicast");
}

static String create_status_cot() {
  unsigned long uptime_seconds = esp_timer_get_time() / 1000000UL;
  IPAddress ip = WiFi.softAPIP();
  
  String uid = "ESP32-WarDragon-" + WiFi.macAddress();
  uid.replace(":", "");
  
  return "<?xml version=\"1.0\"?>\n"
         "<event version=\"2.0\" uid=\"" + uid + "\" type=\"b-m-p-s-m\" time=\"" + String(uptime_seconds) + "\" start=\"" + String(uptime_seconds) + "\" stale=\"" + String(uptime_seconds + 300) + "\">\n"
         "  <point lat=\"0.0\" lon=\"0.0\" hae=\"0.0\" ce=\"9999999\" le=\"9999999\"/>\n"
         "  <detail>\n"
         "    <track course=\"0.0\" speed=\"0.0\"/>\n"
         "    <status readiness=\"true\"/>\n"
         "    <remarks>ESP32 WarDragon Scanner - Packets: " + String(packetCount) + ", Memory: " + String(ESP.getFreeHeap()) + " bytes</remarks>\n"
         "  </detail>\n"
         "</event>\n";
}

static String create_cot_xml(struct uav_data *UAV, int index) {
  unsigned long uptime_seconds = esp_timer_get_time() / 1000000UL;
  
  char lat[16], lon[16], op_lat[16], op_lon[16];
  dtostrf(UAV->lat_d, 11, 6, lat);
  dtostrf(UAV->long_d, 11, 6, lon);
  dtostrf(UAV->base_lat_d, 11, 6, op_lat);
  dtostrf(UAV->base_long_d, 11, 6, op_lon);
  
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           UAV->mac[0], UAV->mac[1], UAV->mac[2],
           UAV->mac[3], UAV->mac[4], UAV->mac[5]);
  
  String uid = "drone-" + String(mac_str);
  uid.replace(":", "");
  
  // Use drone coordinates if available, otherwise use 0,0
  String droneLat = (UAV->lat_d != 0.0) ? String(lat) : "0.0";
  String droneLon = (UAV->long_d != 0.0) ? String(lon) : "0.0";
  String altitude = String(UAV->altitude_msl);
  
  String remarks = "Drone ID: " + String(UAV->uav_id) + 
                  ", Operator: " + String(UAV->op_id) + 
                  ", RSSI: " + String(UAV->rssi) + "dBm" +
                  ", Speed: " + String(UAV->speed) + "m/s" +
                  ", Heading: " + String(UAV->heading) + "°" +
                  ", Description: " + String(UAV->description);
  
  return "<?xml version=\"1.0\"?>\n"
         "<event version=\"2.0\" uid=\"" + uid + "\" type=\"a-f-A-M-F-Q\" time=\"" + String(uptime_seconds) + "\" start=\"" + String(uptime_seconds) + "\" stale=\"" + String(uptime_seconds + 60) + "\">\n"
         "  <point lat=\"" + droneLat + "\" lon=\"" + droneLon + "\" hae=\"" + altitude + "\" ce=\"30\" le=\"30\"/>\n"
         "  <detail>\n"
         "    <track course=\"" + String(UAV->heading) + "\" speed=\"" + String(UAV->speed) + "\"/>\n"
         "    <contact endpoint=\"\" phone=\"\" callsign=\"drone-" + String(UAV->uav_id) + "\"/>\n"
         "    <precisionlocation geopointsrc=\"GPS\" altsrc=\"GPS\"/>\n"
         "    <color argb=\"-256\"/>\n"
         "    <usericon iconsetpath=\"34ae1613-9645-4222-a9d2-e5f243dea2865/Military/UAV_quad.png\"/>\n"
         "    <remarks>" + remarks + "</remarks>\n"
         "  </detail>\n"
         "</event>\n";
}

void ScannerTaskCode(void *pvParameters) {
  Serial.println("Scanner task starting on core " + String(xPortGetCoreID()));
  
  vTaskDelay(pdMS_TO_TICKS(5000));
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&callback);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

  for(;;) {
    unsigned long current_millis = millis();
    
    if ((current_millis - last_status) > 60000UL) {
      Serial.println("Heartbeat: Scanner active on core " + String(xPortGetCoreID()));
      last_status = current_millis;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void WebServerTaskCode(void *pvParameters) {
  Serial.println("Web server task starting on core " + String(xPortGetCoreID()));
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.println("Connect to WiFi: " + String(ap_ssid));
  Serial.println("Password: " + String(ap_password));
  Serial.println("Open browser to: http://" + IP.toString());
  Serial.println("Multicast Telemetry: " + multicastIP.toString() + ":" + String(telemetryPort));
  Serial.println("Multicast Status: " + multicastIP.toString() + ":" + String(statusPort));
  
  server.on("/", [](){
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>WarDragon Scanner</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background: #1a1a1a;
            color: #00ff00;
        }
        .container { 
            max-width: 1200px; 
            margin: 0 auto; 
        }
        .header { 
            text-align: center; 
            margin-bottom: 30px;
            border-bottom: 2px solid #00ff00;
            padding-bottom: 20px;
        }
        .multicast-info {
            background: #2d2d2d;
            padding: 20px;
            border-radius: 8px;
            border: 1px solid #00ff00;
            margin-bottom: 20px;
        }
        .stats { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); 
            gap: 20px; 
            margin-bottom: 30px; 
        }
        .stat-box { 
            background: #2d2d2d; 
            padding: 20px; 
            border-radius: 8px; 
            border: 1px solid #00ff00;
        }
        .drone-data { 
            background: #2d2d2d; 
            padding: 20px; 
            border-radius: 8px; 
            border: 1px solid #00ff00;
            white-space: pre-wrap; 
            font-family: monospace; 
            max-height: 500px; 
            overflow-y: auto; 
        }
        .refresh-btn { 
            background: #00aa00; 
            color: white; 
            border: none; 
            padding: 10px 20px; 
            border-radius: 5px; 
            cursor: pointer; 
            margin: 10px 0; 
        }
        .refresh-btn:hover { 
            background: #00ff00; 
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>WarDragon Scanner</h1>
            <p>ESP32 WiFi Remote ID Scanner with Multicast Publisher</p>
        </div>
        
        <div class="multicast-info">
            <h3>Multicast Connection Info</h3>
            <p><strong>Telemetry:</strong> 239.2.3.1:4242</p>
            <p><strong>Status:</strong> 239.2.3.1:4243</p>
            <p>Configure your app to use Multicast mode with these endpoints</p>
        </div>
        
        <div class="stats">
            <div class="stat-box">
                <h3>Status</h3>
                <p id="status">Active</p>
            </div>
            <div class="stat-box">
                <h3>Packets Detected</h3>
                <p id="packets">Loading...</p>
            </div>
            <div class="stat-box">
                <h3>Uptime</h3>
                <p id="uptime">Loading...</p>
            </div>
            <div class="stat-box">
                <h3>Free Memory</h3>
                <p id="memory">Loading...</p>
            </div>
        </div>
        
        <button class="refresh-btn" onclick="refreshData()">Refresh Data</button>
        
        <div class="drone-data" id="droneData">
            <p>Loading latest drone data...</p>
        </div>
    </div>

    <script>
        function refreshData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('packets').textContent = data.packets;
                    document.getElementById('uptime').textContent = data.uptime + 's';
                    document.getElementById('memory').textContent = data.memory + ' bytes';
                    document.getElementById('droneData').textContent = data.latestDrone || 'No drone data received yet';
                })
                .catch(error => {
                    console.error('Error:', error);
                    document.getElementById('droneData').textContent = 'Error loading data';
                });
        }
        
        setInterval(refreshData, 2000);
        refreshData();
    </script>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
  });

  server.on("/data", [](){
    String response = "{";
    response += "\"packets\":" + String(packetCount) + ",";
    response += "\"uptime\":" + String(esp_timer_get_time() / 1000000UL) + ",";
    response += "\"memory\":" + String(ESP.getFreeHeap()) + ",";
    
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

  server.begin();
  Serial.println("Web server started successfully on core " + String(xPortGetCoreID()));
  
  for(;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("{ \"message\": \"Starting WarDragon ESP32 WiFi Remote ID Scanner with Multicast Publisher\" }");
  
  dataMutex = xSemaphoreCreateMutex();
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
    MulticastTaskCode,
    "MulticastTask",
    8192,
    NULL,
    1,
    &MulticastTask,
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
  vTaskDelay(pdMS_TO_TICKS(1000));
  delay(10); // to avoid issues
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
      packetCount++;
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
            packetCount++;
            update_latest_data(&currentUAV, packetCount);
            printed = true;
          }
        }
      }

      offset += len + 2;
    }
  }
}

static void update_latest_data(struct uav_data *UAV, int index) {
  String jsonData = create_json_response(UAV, index);
  String cotXML = create_cot_xml(UAV, index);
  
  Serial.print(jsonData);
  Serial.print("\r\n");
  
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    latestDroneData = jsonData;
    xSemaphoreGive(dataMutex);
  }
  
  // Send CoT XML via multicast for the iOS app
  publishMulticast(cotXML, false);
}

static void parse_odid(struct uav_data *UAV, ODID_UAS_Data *UAS_data2) {
  memset(UAV->op_id, 0, sizeof(UAV->op_id));
  memset(UAV->uav_id, 0, sizeof(UAV->uav_id));
  memset(UAV->description, 0, sizeof(UAV->description));
  memset(UAV->auth_data, 0, sizeof(UAV->auth_data));

  if (UAS_data2->BasicIDValid[0]) {
    strncpy(UAV->uav_id, (char *)UAS_data2->BasicID[0].UASID, ODID_ID_SIZE);
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

  UAV->ua_type = UAS_data2->BasicID[0].UAType;
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
    case 2:
      for (int i = 0; (i < (l - 6)) && (i < ODID_ID_SIZE); ++i) {
        UAV->op_id[i] = (char)v[i + 6];
      }
      break;
    case 3:
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

static void store_mac(struct uav_data *uav, uint8_t *payload) {
  memcpy(uav->mac, &payload[10], 6);
}

static String create_json_response(struct uav_data *UAV, int index) {
  unsigned long uptime_seconds = esp_timer_get_time() / 1000000UL;

  char lat[16], lon[16], op_lat[16], op_lon[16];
  dtostrf(UAV->lat_d, 11, 6, lat);
  dtostrf(UAV->long_d, 11, 6, lon);
  dtostrf(UAV->base_lat_d, 11, 6, op_lat);
  dtostrf(UAV->base_long_d, 11, 6, op_lon);

  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           UAV->mac[0], UAV->mac[1], UAV->mac[2],
           UAV->mac[3], UAV->mac[4], UAV->mac[5]);

  String json = "{";
  json += "\"index\":" + String(index) + ",";
  json += "\"runtime\":" + String(uptime_seconds) + ",";
  json += "\"Basic ID\":{";
  json += "\"id\":\"" + String(strlen(UAV->uav_id) > 0 ? UAV->uav_id : "NONE") + "\",";
  json += "\"id_type\":\"Serial Number (ANSI/CTA-2063-A)\",";
  json += "\"ua_type\":" + String(UAV->ua_type) + ",";
  json += "\"MAC\":\"" + String(mac_str) + "\",";
  json += "\"RSSI\":" + String(UAV->rssi);
  json += "},";
  json += "\"Location/Vector Message\":{";
  json += "\"latitude\":" + String(lat) + ",";
  json += "\"longitude\":" + String(lon) + ",";
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
  json += "\"latitude\":" + String(op_lat) + ",";
  json += "\"longitude\":" + String(op_lon) + ",";
  json += "\"operator_lat\":" + String(op_lat) + ",";
  json += "\"operator_lon\":" + String(op_lon) + ",";
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