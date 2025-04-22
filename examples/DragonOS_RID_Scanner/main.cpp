/* -*- tab-width: 2; mode: c; -*-
 *
 * Scanner for WiFi direct remote id.
 * Handles both opendroneid and French formats.
 *
 * Copyright (c) 2020-2021, Steve Jack.
 *
 * MIT licence.
 *
 * Nov. '21     Added option to dump ODID frame to serial output.
 * Oct. '21     Updated for opendroneid release 1.0.
 * June '21     Added an option to log to an SD card.
 * May '21      Fixed a bug that presented when handing packed ODID data from multiple sources.
 * April '21    Added support for EN 4709-002 WiFi beacons.
 * March '21    Added BLE scan. Doesn't work very well.
 * January '21  Added support for ANSI/CTA 2063 French IDs.
 *
 * Notes
 *
 * May need a semaphore.
 *
 */

/*
 * CEMAXECUTER
 * Minimal scanner for WiFi direct remote ID (OpenDroneID and French formats).
 * Prints results in the same JSON format as originally shown, immediately upon decode.
 *
 * MIT License.
 */

 #if !defined(ARDUINO_ARCH_ESP32)
 #error "This program requires an ESP32"
 #endif
 
 #include <Arduino.h>
 #include <esp_wifi.h>
 #include <esp_event_loop.h>
 #include <nvs_flash.h>
 #include "opendroneid.h"
 #include "odid_wifi.h"
 #include <esp_timer.h>
 
 static ODID_UAS_Data UAS_data;
 
 // This struct holds the UAV data for a single decoded packet
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
 
 // Forward declarations
 static esp_err_t event_handler(void *, system_event_t *);
 static void callback(void *, wifi_promiscuous_pkt_type_t);
 static void parse_odid(struct uav_data *, ODID_UAS_Data *);
 static void parse_french_id(struct uav_data *, uint8_t *);
 static void print_json(struct uav_data *uav, int index);
 static void store_mac(struct uav_data *uav, uint8_t *payload);
 
 // Global counter for how many packets have been printed
 static int packetCount = 0;
 
 // Variables for periodic status messages
 unsigned long last_status = 0; // To track the last status message time
 unsigned long current_millis = 0; //To hold the current time in milliseconds
 
 static esp_err_t event_handler(void *ctx, system_event_t *event)
 {
   return ESP_OK;
 }
 
 void setup()
 {
   setCpuFrequencyMhz(160);
   Serial.begin(115200);
   Serial.println("{ \"message\": \"Starting WarDragon minimalist ESP32 WiFi Remote ID Scanner\" }");
 
   nvs_flash_init();
   tcpip_adapter_init();
   esp_event_loop_init(event_handler, NULL);
 
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   esp_wifi_init(&cfg);
   esp_wifi_set_storage(WIFI_STORAGE_RAM);
   esp_wifi_set_mode(WIFI_MODE_NULL);
   esp_wifi_start();
   esp_wifi_set_promiscuous(true);
   esp_wifi_set_promiscuous_rx_cb(&callback);
   esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
 }
 
 void loop()
 {
   delay(10); // Small delay to keep the loop responsive
 
   // Update the current time
   current_millis = millis();
 
   // Check if 60 seconds have passed since the last status message
   if ((current_millis - last_status) > 60000UL) { // 60,000 milliseconds = 60 seconds
     Serial.println( "Heartbeat: Device is active and running.");
     last_status = current_millis;
   }
 }
 
 static void callback(void *buffer, wifi_promiscuous_pkt_type_t type)
 {
   if (type != WIFI_PKT_MGMT)
     return; // We only care about management frames
 
   wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buffer;
   uint8_t *payload = packet->payload;
   int length = packet->rx_ctrl.sig_len;
 
   struct uav_data currentUAV;
   memset(&currentUAV, 0, sizeof(currentUAV));
 
   store_mac(&currentUAV, payload); // Store source MAC for fallback ID
   currentUAV.rssi = packet->rx_ctrl.rssi; // Store RSSI
 
   static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};
 
   // If it's a NAN action frame (OpenDroneID)
   if (memcmp(nan_dest, &payload[4], 6) == 0)
   {
     if (odid_wifi_receive_message_pack_nan_action_frame(&UAS_data, (char *)currentUAV.op_id, payload, length) == 0)
     {
       parse_odid(&currentUAV, &UAS_data);
       packetCount++;
       print_json(&currentUAV, packetCount);
     }
   }
   else if (payload[0] == 0x80)
   {
     // Beacon frame
     int offset = 36;
     bool printed = false;
 
     while (offset < length)
     {
       int typ = payload[offset];
       int len = payload[offset + 1];
       uint8_t *val = &payload[offset + 2];
 
       if (!printed)
       {
         // Check for French format
         if ((typ == 0xdd) && (val[0] == 0x6a) && (val[1] == 0x5c) && (val[2] == 0x35))
         {
           parse_french_id(&currentUAV, &payload[offset]);
           packetCount++;
           print_json(&currentUAV, packetCount);
           printed = true;
         }
         // Check for ODID Wi-Fi beacon
         else if ((typ == 0xdd) &&
                  (((val[0] == 0x90 && val[1] == 0x3a && val[2] == 0xe6)) ||
                   ((val[0] == 0xfa && val[1] == 0x0b && val[2] == 0xbc))))
         {
           int j = offset + 7;
           if (j < length)
           {
             memset(&UAS_data, 0, sizeof(UAS_data));
             odid_message_process_pack(&UAS_data, &payload[j], length - j);
             parse_odid(&currentUAV, &UAS_data);
             packetCount++;
             print_json(&currentUAV, packetCount);
             printed = true;
           }
         }
       }
 
       offset += len + 2;
     }
   }
 }
 
 static void parse_odid(struct uav_data *UAV, ODID_UAS_Data *UAS_data2)
 {
   memset(UAV->op_id, 0, sizeof(UAV->op_id));
   memset(UAV->uav_id, 0, sizeof(UAV->uav_id));
   memset(UAV->description, 0, sizeof(UAV->description));
   memset(UAV->auth_data, 0, sizeof(UAV->auth_data));
 
   if (UAS_data2->BasicIDValid[0])
   {
     strncpy(UAV->uav_id, (char *)UAS_data2->BasicID[0].UASID, ODID_ID_SIZE);
   }
 
   if (UAS_data2->LocationValid)
   {
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
 
   if (UAS_data2->SystemValid)
   {
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
 
   if (UAS_data2->AuthValid[0])
   {
     UAV->auth_type = UAS_data2->Auth[0].AuthType;
     UAV->auth_page = UAS_data2->Auth[0].DataPage;
     UAV->auth_length = UAS_data2->Auth[0].Length;
     UAV->auth_timestamp = UAS_data2->Auth[0].Timestamp;
     memcpy(UAV->auth_data, UAS_data2->Auth[0].AuthData, sizeof(UAV->auth_data) - 1);
   }
 
   if (UAS_data2->SelfIDValid)
   {
     UAV->desc_type = UAS_data2->SelfID.DescType;
     strncpy(UAV->description, UAS_data2->SelfID.Desc, ODID_STR_SIZE);
   }
 
   if (UAS_data2->OperatorIDValid)
   {
     UAV->operator_id_type = UAS_data2->OperatorID.OperatorIdType;
     strncpy(UAV->op_id, (char *)UAS_data2->OperatorID.OperatorId, ODID_ID_SIZE);
   }
 }
 
 static void parse_french_id(struct uav_data *UAV, uint8_t *payload)
 {
   memset(UAV->op_id, 0, sizeof(UAV->op_id));
   memset(UAV->uav_id, 0, sizeof(UAV->uav_id));
 
   int length = payload[1];
   int j = 6;
 
   union
   {
     int32_t i32;
     uint32_t u32;
   } uav_lat, uav_long, base_lat, base_long;
   union
   {
     int16_t i16;
     uint16_t u16;
   } alt, height;
 
   uav_lat.u32 = uav_long.u32 = base_lat.u32 = base_long.u32 = 0;
   alt.u16 = height.u16 = 0;
 
   while (j < length)
   {
     int t = payload[j];
     int l = payload[j + 1];
     uint8_t *v = &payload[j + 2];
 
     switch (t)
     {
     case 2: // Operator ID
       for (int i = 0; (i < (l - 6)) && (i < ODID_ID_SIZE); ++i)
       {
         UAV->op_id[i] = (char)v[i + 6];
       }
       break;
     case 3: // UAV ID
       for (int i = 0; (i < l) && (i < ODID_ID_SIZE); ++i)
       {
         UAV->uav_id[i] = (char)v[i];
       }
       break;
     case 4:
       for (int i = 0; i < 4; ++i)
       {
         uav_lat.u32 <<= 8;
         uav_lat.u32 |= v[i];
       }
       break;
     case 5:
       for (int i = 0; i < 4; ++i)
       {
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
       for (int i = 0; i < 4; ++i)
       {
         base_lat.u32 <<= 8;
         base_lat.u32 |= v[i];
       }
       break;
     case 9:
       for (int i = 0; i < 4; ++i)
       {
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
 
 static void store_mac(struct uav_data *uav, uint8_t *payload)
 {
   // Source MAC is at payload[10..15] for beacon
   // This might differ depending on frame type, but for mgmt frames it's usually consistent.
   memcpy(uav->mac, &payload[10], 6);
 }
 
 static void print_json(struct uav_data *UAV, int index)
 {
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
 
   Serial.print(json);
   Serial.print("\r\n");
 }