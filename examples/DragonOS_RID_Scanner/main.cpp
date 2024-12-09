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

static ODID_UAS_Data UAS_data;

// This struct holds the UAV data for a single decoded packet
struct uav_data {
  uint8_t mac[6];
  char    op_id[ODID_ID_SIZE + 1];
  char    uav_id[ODID_ID_SIZE + 1];
  double  lat_d;
  double  long_d;
  double  base_lat_d;
  double  base_long_d;
  int     altitude_msl;
  int     height_agl;
  int     speed;
  int     heading;
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

static esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void setup() {
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

void loop() {
  delay(10); // Small delay to keep the loop responsive
}

static void callback(void* buffer, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return; // We only care about management frames

  wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *) buffer;
  uint8_t *payload = packet->payload;
  int length = packet->rx_ctrl.sig_len;

  struct uav_data currentUAV;
  memset(&currentUAV, 0, sizeof(currentUAV));
  
  store_mac(&currentUAV, payload); // Store source MAC for fallback ID

  static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};

  // If it's a NAN action frame (OpenDroneID)
  if (memcmp(nan_dest, &payload[4], 6) == 0) {
    if (odid_wifi_receive_message_pack_nan_action_frame(&UAS_data, (char *)currentUAV.op_id, payload, length) == 0) {
      parse_odid(&currentUAV, &UAS_data);
      packetCount++;
      print_json(&currentUAV, packetCount);
    }
  } else if (payload[0] == 0x80) {
    // Beacon frame
    int offset = 36;
    bool printed = false;

    while (offset < length) {
      int typ = payload[offset];
      int len = payload[offset + 1];
      uint8_t *val = &payload[offset + 2];

      if (!printed) {
        // Check for French format
        if ((typ == 0xdd) && (val[0] == 0x6a) && (val[1] == 0x5c) && (val[2] == 0x35)) {
          parse_french_id(&currentUAV, &payload[offset]);
          packetCount++;
          print_json(&currentUAV, packetCount);
          printed = true;
        }
        // Check for ODID Wi-Fi beacon
        else if ((typ == 0xdd) &&
                 (((val[0] == 0x90 && val[1] == 0x3a && val[2] == 0xe6)) ||
                  ((val[0] == 0xfa && val[1] == 0x0b && val[2] == 0xbc)))) {
          int j = offset + 7;
          if (j < length) {
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

static void parse_odid(struct uav_data *UAV, ODID_UAS_Data *UAS_data2) {
  memset(UAV->op_id, 0, sizeof(UAV->op_id));
  memset(UAV->uav_id, 0, sizeof(UAV->uav_id));

  if (UAS_data2->OperatorIDValid) {
    strncpy(UAV->op_id, (char *)UAS_data2->OperatorID.OperatorId, ODID_ID_SIZE);
  }

  if (UAS_data2->BasicIDValid[0]) {
    strncpy(UAV->uav_id, (char *)UAS_data2->BasicID[0].UASID, ODID_ID_SIZE);
  }

  if (UAS_data2->LocationValid) {
    UAV->lat_d        = UAS_data2->Location.Latitude;
    UAV->long_d       = UAS_data2->Location.Longitude;
    UAV->altitude_msl = (int) UAS_data2->Location.AltitudeGeo;
    UAV->height_agl   = (int) UAS_data2->Location.Height;
    UAV->speed        = (int) UAS_data2->Location.SpeedHorizontal;
    UAV->heading      = (int) UAS_data2->Location.Direction;
  }

  if (UAS_data2->SystemValid) {
    UAV->base_lat_d  = UAS_data2->System.OperatorLatitude;
    UAV->base_long_d = UAS_data2->System.OperatorLongitude;
  }
}

static void parse_french_id(struct uav_data *UAV, uint8_t *payload) {
  memset(UAV->op_id, 0, sizeof(UAV->op_id));
  memset(UAV->uav_id, 0, sizeof(UAV->uav_id));

  int length = payload[1];
  int j = 6;

  union {int32_t i32; uint32_t u32;} uav_lat, uav_long, base_lat, base_long;
  union {int16_t i16; uint16_t u16;} alt, height;
  
  uav_lat.u32 = uav_long.u32 = base_lat.u32 = base_long.u32 = 0;
  alt.u16 = height.u16 = 0;

  while (j < length) {
    int t = payload[j];
    int l = payload[j + 1];
    uint8_t *v = &payload[j + 2];

    switch (t) {
      case 2: // Operator ID
        for (int i = 0; (i < (l - 6)) && (i < ODID_ID_SIZE); ++i) {
          UAV->op_id[i] = (char) v[i + 6];
        }
        break;
      case 3: // UAV ID
        for (int i = 0; (i < l) && (i < ODID_ID_SIZE); ++i) {
          UAV->uav_id[i] = (char) v[i];
        }
        break;
      case 4:
        for (int i = 0; i < 4; ++i) { uav_lat.u32 <<= 8; uav_lat.u32 |= v[i]; }
        break;
      case 5:
        for (int i = 0; i < 4; ++i) { uav_long.u32 <<= 8; uav_long.u32 |= v[i]; }
        break;
      case 6:
        alt.u16 = (((uint16_t) v[0]) << 8) | (uint16_t) v[1];
        break;
      case 7:
        height.u16 = (((uint16_t) v[0]) << 8) | (uint16_t) v[1];
        break;
      case 8:
        for (int i = 0; i < 4; ++i) { base_lat.u32 <<= 8; base_lat.u32 |= v[i]; }
        break;
      case 9:
        for (int i = 0; i < 4; ++i) { base_long.u32 <<= 8; base_long.u32 |= v[i]; }
        break;
      case 10:
        UAV->speed = v[0];
        break;
      case 11:
        UAV->heading = (((uint16_t) v[0]) << 8) | (uint16_t) v[1];
        break;
      default:
        break;
    }

    j += l + 2;
  }

  UAV->lat_d        = 1.0e-5 * (double) uav_lat.i32;
  UAV->long_d       = 1.0e-5 * (double) uav_long.i32;
  UAV->base_lat_d   = 1.0e-5 * (double) base_lat.i32;
  UAV->base_long_d  = 1.0e-5 * (double) base_long.i32;
  UAV->altitude_msl = alt.i16;
  UAV->height_agl   = height.i16;
}

static void store_mac(struct uav_data *uav, uint8_t *payload) {
  // Source MAC is at payload[10..15] for beacon
  // This might differ depending on frame type, but for mgmt frames it's usually consistent.
  memcpy(uav->mac, &payload[10], 6);
}

static void print_json(struct uav_data *UAV, int index) {
  int secs = 0; // Still unused, but we keep it for compatibility with the original format.

  char text1[16], text2[16], text3[16], text4[16];

  dtostrf(UAV->lat_d,     11, 6, text1);
  dtostrf(UAV->long_d,    11, 6, text2);
  dtostrf(UAV->base_lat_d,11, 6, text3);
  dtostrf(UAV->base_long_d,11,6, text4);

  char id[64];
  if (strlen(UAV->op_id) > 0) {
    snprintf(id, sizeof(id), "%s", UAV->op_id);
  } else {
    snprintf(id, sizeof(id), "%02x:%02x:%02x:%02x:%02x:%02x",
             UAV->mac[0], UAV->mac[1], UAV->mac[2],
             UAV->mac[3], UAV->mac[4], UAV->mac[5]);
  }

  char json[512];
  sprintf(json,
      "{ "
        "\"index\": %d, "
        "\"runtime\": %d, "
        "\"Basic ID\": { "
          "\"id\": \"%s\", "
          "\"id_type\": \"Serial Number (ANSI/CTA-2063-A)\" "
        "}, "
        "\"Location/Vector Message\": { "
          "\"latitude\": %s, "
          "\"longitude\": %s, "
          "\"speed\": %d, "
          "\"vert_speed\": 0, "
          "\"geodetic_altitude\": %d, "
          "\"height_agl\": %d "
        "}, "
        "\"Self-ID Message\": { "
          "\"text\": \"UAV %s operational\" "
        "}, "
        "\"System Message\": { "
          "\"latitude\": %s, "
          "\"longitude\": %s "
        "} "
      "}",
      index,
      secs,
      id,
      text1,
      text2,
      UAV->speed,
      UAV->altitude_msl,
      UAV->height_agl,
      id,
      text3,
      text4
  );

  Serial.print(json);
  Serial.print("\r\n");
}
