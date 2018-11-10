/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
   This example shows how to use ESPNOW.
   Prepare two device, one for sending ESPNOW data and another for receiving
   ESPNOW data.
*/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "rom/ets_sys.h"
#include "rom/crc.h"

#include "manchester.h"
#include "espnow_types.h"

// IS_BASESTATION = 0 - read controllers, transmit data
// IS_BASESTATION = 1 - wait for packets, do stuff
#define IS_BASESTATION 0

// Motor pins
#define EN_A_PIN 23
#define EN_B_PIN 16

#define A1_PIN 26
#define A2_PIN 27

#define B1_PIN 21
#define B2_PIN 22

#define OUTPUT_MASK (\
  (1 << EN_A_PIN) | (1 << EN_B_PIN) | \
  (1 << A1_PIN) | (1 << A2_PIN) | \
  (1 << B1_PIN) | (1 << B2_PIN) \
)

// Controller data pin
#define ATAD_PIN 23

// Controller button mappings
#define BUTTON_A (1<<5)
#define BUTTON_B (1<<4)
#define BUTTON_U (1<<3)
#define BUTTON_D (1<<2)
#define BUTTON_L (1<<1)
#define BUTTON_R (1<<0)

static const char *TAG = "foobot";

static xQueueHandle espnow_queue;

static uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void espnow_deinit(espnow_state_t *state);

esp_err_t espnow_send_command_data(espnow_state_t* state, uint8_t bot, foobot_command_t command);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
  switch(event->event_id)
  {
  case SYSTEM_EVENT_STA_START:
    ESP_LOGI(TAG, "WiFi started");
    break;
  default:
    break;
  }
  return ESP_OK;
}

// Bot states
#define NUM_BOTS 4

typedef struct
{
  uint8_t id;
  uint8_t command;
  uint8_t dirty;
} bot_state_t;

volatile bot_state_t bots[NUM_BOTS] =
{
  { '1', 0xff, 0 },
  { '2', 0xff, 0 },
  { 'a', 0xff, 0 },
  { 'b', 0xff, 0 },
};

void update_bot(espnow_state_t* state, volatile bot_state_t* bot_state)
{
  if (!state) { return; }

  foobot_command_t command;

  if (bot_state->command & BUTTON_B) // Forwards
  {
    if ((bot_state->command & BUTTON_R) == 0)
    {
      command.left_dir = 1;
      command.left_en = 1;
    }
    else
    {
      command.left_dir = 0;
      command.left_en = 0;
    }
    if ((bot_state->command & BUTTON_L) == 0)
    {
      command.right_dir = 1;
      command.right_en = 1;
    }
    else
    {
      command.right_dir = 0;
      command.right_en = 0;
    }
  }
  else if (bot_state->command & BUTTON_A)  // Reverse
  {
    if ((bot_state->command & BUTTON_R) == 0)
    {
      command.left_dir = -1;
      command.left_en = 1;
    }
    if ((bot_state->command & BUTTON_L) == 0)
    {
      command.right_dir = -1;
      command.right_en = 1;
    }
  }
  else  // Spinning or braking
  {
    if (bot_state->command & BUTTON_L) // Spin left
    {
      command.left_dir = -1;
      command.left_en = 1;
      command.right_dir = 1;
      command.right_en = 1;
    }
    else if (bot_state->command & BUTTON_R)  // Spin right
    {
      command.left_dir = 1;
      command.left_en = 1;
      command.right_dir = -1;
      command.right_en = 1;
    }
    else  // Brake
    {
      command.left_en = 1;
      command.right_en = 1;
      command.left_dir = 0;
      command.right_dir = 0;
    }
  }

  // Queue the appropriate commands
  if (espnow_send_command_data(state, bot_state->id, command) != ESP_OK)
  {
    ESP_LOGE(TAG, "Send error");
  }
}

// Update the bots based on the current commands
void update_bots(espnow_state_t* state)
{
  for (int i = 0; i < NUM_BOTS; ++i)
  {
    if (bots[i].dirty)
    {
      update_bot(state, &bots[i]);
    }
  }
}

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
  tcpip_adapter_init();
  ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
  ESP_ERROR_CHECK( esp_wifi_start());

  /* In order to simplify example, channel is set after WiFi started.
   * This is not necessary in real application if the two devices have
   * been already on the same channel.
   */
  ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );
}

/* ESPNOW sending or receiving callback function is called in WiFi task.
 * Users should not do lengthy operations from this task. Instead, post
 * necessary data to a queue and handle it from a lower priority task. */
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  espnow_event_t evt;
  espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

  if (mac_addr == NULL)
  {
    ESP_LOGE(TAG, "Send cb arg error");
    return;
  }

  evt.id = ESPNOW_SEND_CB;
  memcpy(send_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  send_cb->status = status;
  if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
  {
    ESP_LOGW(TAG, "Send send queue fail");
  }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
  espnow_event_t evt;
  espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

  if (mac_addr == NULL || data == NULL || len <= 0)
  {
    ESP_LOGE(TAG, "Receive cb arg error");
    return;
  }

  evt.id = ESPNOW_RECV_CB;
  memcpy(recv_cb->mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
  recv_cb->data = malloc(len);
  if (recv_cb->data == NULL)
  {
    ESP_LOGE(TAG, "Malloc receive data fail");
    return;
  }
  memcpy(recv_cb->data, data, len);
  recv_cb->data_len = len;
  if (xQueueSend(espnow_queue, &evt, portMAX_DELAY) != pdTRUE)
  {
    ESP_LOGW(TAG, "Send receive queue fail");
    free(recv_cb->data);
  }
}

static char hex[] = "0123456789abcdef";

/* Dump packet to UART */
void data_dump(uint8_t* data, uint16_t data_len)
{
  uint16_t byte_index = 0;
  putchar('\r');
  putchar('\n');
  while (byte_index < data_len)
  {
    for (uint16_t line_char = 0; line_char < 16 && byte_index < data_len; ++line_char, ++byte_index)
    {
      putchar(hex[(data[byte_index] & 0xf0) >> 4]);
      putchar(hex[(data[byte_index] & 0x0f)     ]);
    }
  }
  putchar('\r');
  putchar('\n');
}

/* Parse received ESPNOW data. */
int espnow_data_parse(uint8_t *data, uint16_t data_len)
{
  espnow_data_t *buf = (espnow_data_t*)data;
  uint16_t crc, crc_cal = 0;

  // Check there is enough data to process the header
  if (data_len < sizeof(espnow_data_t))
  {
      ESP_LOGE(TAG, "Receive ESPNOW data too short, len:%d", data_len);
      return -1;
  }

  // Read packet size
  size_t packet_size = sizeof(espnow_data_t) + buf->payload_len;

  // Check CRC now we know the size of the packet
  crc = buf->crc;
  buf->crc = 0; // Zero CRC so we can recalculate correctly
  crc_cal = crc16_le(UINT16_MAX, (uint8_t const *)buf, packet_size);
  if (crc_cal == crc)
  {
      buf->crc = crc_cal; // Replace CRC for UART transmission

      // Invoke the appropriate handler for the packet
      switch (buf->type)
      {
        case PT_StationInfo:
        {
          assert(0 && "Station Info packets not yet implemented!");
        } break;
        case PT_Command:
        {
          // Just write out the data
          data_dump(data, packet_size);
        } break;
        default:
        {
          assert(0 && "Unknown packet type!");
        } break;
      }

      return 0;
  }

  return -1;
}

/* Prepare sensor data packet to be sent. */
espnow_packet_param_t espnow_build_command_data_packet(espnow_state_t* state, uint8_t bot, foobot_command_t command)
{
  espnow_command_data_t *buf = (espnow_command_data_t *)state->buffer;

  // Create packet params
  espnow_packet_param_t packet_params;
  packet_params.len = sizeof(espnow_data_t) + sizeof(espnow_command_data_t);
  packet_params.buffer = (uint8_t*)buf;

  // Fill buffer
  memset(buf, 0, packet_params.len);
  buf->header.type = PT_Command;
  buf->header.payload_len = sizeof(espnow_command_data_t) - sizeof(espnow_data_t);
  buf->target = bot;
  buf->command = command;

  assert(state->len >= sizeof(espnow_command_data_t));

  esp_read_mac(buf->header.sender_mac, ESP_MAC_WIFI_STA);
  buf->header.crc = 0;

  buf->header.crc = crc16_le(UINT16_MAX, (uint8_t const *)buf, sizeof(espnow_command_data_t));

  return packet_params;
}

esp_err_t espnow_send_command_data(espnow_state_t* state, uint8_t bot, foobot_command_t command)
{
  espnow_packet_param_t packet = espnow_build_command_data_packet(state, bot, command);

  return esp_now_send(state->dest_mac, packet.buffer, packet.len);
}

/*
extern uint8_t bits;
extern uint32_t counter;
extern uint32_t cycles;
extern uint32_t counter_buffer[32];
extern uint32_t state_buffer[32];
extern uint32_t timer_val;
*/

static void espnow_task(void *pvParameter)
{
  espnow_event_t evt;

  espnow_state_t *state = (espnow_state_t *)pvParameter;

#if !IS_BASESTATION
  /* If we're a sensor, start sending packets */
  /*
  ESP_LOGI(TAG, "Start sending broadcast data");
  if (espnow_send_command_data(state) != ESP_OK)
  {
      ESP_LOGE(TAG, "Send error");
      espnow_deinit(state);
      vTaskDelete(NULL);
  }
  */
#else
  ESP_LOGI(TAG, "Waiting for data");
#endif // !IS_BASESTATION

  while (xQueueReceive(espnow_queue, &evt, portMAX_DELAY) == pdTRUE)
  {
    switch (evt.id)
    {
      case ESPNOW_SEND_CB:
      {
        espnow_event_send_cb_t *send_cb = &evt.info.send_cb;

        /* Send more data now the previous data is sent. */
        /*
        ESP_LOGI(TAG, "send data to "MACSTR"", MAC2STR(send_cb->mac_addr));
        if (espnow_send_command_data(state) != ESP_OK)
        {
            ESP_LOGE(TAG, "Send error");
            espnow_deinit(state);
            vTaskDelete(NULL);
        }
        */

        /* Wait a bit */
        //vTaskDelay(1000 / portTICK_PERIOD_MS);

        break;
      }
      case ESPNOW_RECV_CB:
      {
        espnow_event_recv_cb_t *recv_cb = &evt.info.recv_cb;

        if (espnow_data_parse(recv_cb->data, recv_cb->data_len) < 0)
        {
          ESP_LOGI(TAG, "Receive error data from: "MACSTR"", MAC2STR(recv_cb->mac_addr));
        }
        free(recv_cb->data);
        break;
      }
      default:
        ESP_LOGE(TAG, "Callback type error: %d", evt.id);
        break;
    }

#if IS_BASESTATION == 0
    // Update bots and generate commands
    ESP_LOGI(TAG, "Command buffer:");
    data_dump((void*)&bots, sizeof(bots));
    update_bots(state);

    // Send out any pending commands

    /*
    ESP_LOGI(TAG, "Bits: %d", bits);
    ESP_LOGI(TAG, "Cycle: %d Counter: %d", cycles, counter);
    for (int i = 0; i < 32; ++i)
    {
      ESP_LOGI(TAG, "Counter: %d State: %d", counter_buffer[i], state_buffer[i]);
    }
    ESP_LOGI(TAG, "Timer: %d", timer_val);
    */
#endif

    // Give other tasks a chance
    vTaskDelay(1);
  }
}

static esp_err_t espnow_init(void)
{
    espnow_state_t *state;

    espnow_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(espnow_event_t));
    if (espnow_queue == NULL)
    {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL)
    {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    /* Initialize sending parameters. */
    state = malloc(sizeof(espnow_state_t));
    memset(state, 0, sizeof(espnow_state_t));
    if (state == NULL)
    {
        ESP_LOGE(TAG, "Malloc send parameter fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    state->len = CONFIG_ESPNOW_SEND_LEN;
    state->buffer = malloc(CONFIG_ESPNOW_SEND_LEN);
    if (state->buffer == NULL)
    {
        ESP_LOGE(TAG, "Malloc send buffer fail");
        free(state);
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }

    /* Start out by sending broadcast packets */
    memcpy(state->dest_mac, broadcast_mac, ESP_NOW_ETH_ALEN);

    /* Create the task to send and receive packets */
    xTaskCreate(espnow_task, "espnow_task", 2048, state, 4, NULL);

    return ESP_OK;
}

static void espnow_deinit(espnow_state_t *state)
{
    free(state->buffer);
    free(state);
    vSemaphoreDelete(espnow_queue);
    esp_now_deinit();
}

// Receive and process commands from the controller interface
void process_command(uint8_t target, uint8_t command)
{
  for (int i = 0; i < NUM_BOTS; ++i)
  {
    if (bots[i].id == target)
    {
      bots[i].command = command;
      bots[i].dirty = 1;
      break;
    }
  }
}

void app_main()
{
    // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
  {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

#if IS_BASESTATION
  // Configure outputs
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = OUTPUT_MASK;
  io_conf.pull_down_en = 1;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  // Disable motors
  gpio_set_level(EN_A_PIN, 0);
  gpio_set_level(EN_B_PIN, 0);
#else
  ESP_LOGI(TAG, "Manchester init...");
  // Initialise manchester receiver
  manchester_init(ATAD_PIN);
#endif

  // Initialise ESPNOW
  wifi_init();
  espnow_init();
}

