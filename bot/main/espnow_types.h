/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_TYPES_H
#define ESPNOW_TYPES_H

/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#if CONFIG_STATION_MODE
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif

#define ESPNOW_QUEUE_SIZE           6

#define IS_BROADCAST_ADDR(addr) (memcmp(addr, broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
  ESPNOW_SEND_CB,
  ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
  esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
  uint8_t mac_addr[ESP_NOW_ETH_ALEN];
  uint8_t *data;
  int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

/* When ESPNOW sending or receiving callback function is called, post event to ESPNOW task. */
typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

#define COMMAND_QUEUE_SIZE           256

typedef struct {
  uint8_t target;
  uint8_t command;
} foobot_command_event_t;

typedef enum
{
  PT_StationInfo,
  PT_Command,
} espnow_type_t;

/* User defined field of ESPNOW data in this example. */
typedef struct {
    uint8_t sender_mac[ESP_NOW_ETH_ALEN]; // MAC address of the sender
    uint16_t crc;                         // CRC16 hash of ESPNOW data
    uint16_t type;                        // Type of the encapsulated packet
    uint16_t payload_len;                 // Length of the payload data
    uint8_t payload[0];                   // Real payload of ESPNOW data.
} __attribute__((packed)) espnow_data_t;

// Station info packet
typedef enum
{
  RO_Station,
  RO_Bot,
} role_t;

typedef struct
{
  espnow_data_t header;
  role_t role;
  
} __attribute__((packed)) espnow_stationinfo_data_t;

// Foobot data packet
typedef struct
{
  uint8_t target;
  int8_t left_dir;
  int8_t left_en;
  int8_t right_dir;
  int8_t right_en;
} __attribute__((packed)) foobot_command_t;

#define NUM_BOTS 4

typedef struct
{
  espnow_data_t header;
  foobot_command_t command[NUM_BOTS];
} __attribute__((packed)) espnow_command_data_t;

/* Data about a packet waiting to be sent. */
typedef struct {
  int len;
  uint8_t *buffer;
} espnow_packet_param_t;

/* ESPNOW dataloger state. */
typedef struct {
    int len;                              // Maximum packet size
    uint8_t *buffer;                      // Buffer for building ESPNOW data.
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   // MAC address of destination device.
} espnow_state_t;

#endif // ESPNOW_TYPES_H

