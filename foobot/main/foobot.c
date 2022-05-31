#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <esp_eth.h>

#include <cJSON.h>
#include <freertos/timers.h>

#include "wifi.h"
#include "led.h"
#include "motor.h"

#include "joystick.inc"
#include "virtualjoystick.inc"

static const char *TAG = "foobot_ws";

TimerHandle_t xFailsafeTimer = NULL;

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct async_resp_arg {
  httpd_handle_t hd;
  int fd;
};

/*
 * async send function, which we put into the httpd work queue
 */
static void ws_async_send(void *arg)
{
  static const char * data = "Async data";
  struct async_resp_arg *resp_arg = arg;
  httpd_handle_t hd = resp_arg->hd;
  int fd = resp_arg->fd;
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.payload = (uint8_t*)data;
  ws_pkt.len = strlen(data);
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  httpd_ws_send_frame_async(hd, fd, &ws_pkt);
  free(resp_arg);
}

static esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
  struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
  resp_arg->hd = req->handle;
  resp_arg->fd = httpd_req_to_sockfd(req);
  return httpd_queue_work(handle, ws_async_send, resp_arg);
}

/*
 * This handler echos back the received ws data
 * and triggers an async send if certain message received
 */
static esp_err_t echo_handler(httpd_req_t *req)
{
  if (req->method == HTTP_GET) {
    ESP_LOGI(TAG, "Handshake done, the new connection was opened");
    return ESP_OK;
  }
  httpd_ws_frame_t ws_pkt;
  uint8_t *buf = NULL;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  /* Set max_len = 0 to get the frame len */
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
    return ret;
  }
  ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
  if (ws_pkt.len) {
    /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
    buf = calloc(1, ws_pkt.len + 1);
    if (buf == NULL) {
      ESP_LOGE(TAG, "Failed to calloc memory for buf");
      return ESP_ERR_NO_MEM;
    }
    ws_pkt.payload = buf;
    /* Set max_len = ws_pkt.len to get the frame payload */
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
      free(buf);
      return ret;
    }
    ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
  }
  ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
  if (ws_pkt.type == HTTPD_WS_TYPE_TEXT &&
    strcmp((char*)ws_pkt.payload,"Trigger async") == 0) {
    free(buf);
    return trigger_async_send(req->handle, req);
  }

  // Process packet
  cJSON* root = cJSON_Parse((char*)ws_pkt.payload);
  cJSON* x_val = cJSON_GetObjectItem(root, "x");
  cJSON* y_val = cJSON_GetObjectItem(root, "y");

  if (x_val && y_val && x_val->type == cJSON_Number && y_val->type == cJSON_Number)
  {
    // Print the decoded joystick position
    ESP_LOGI(TAG, "X: %d Y: %d", x_val->valueint, y_val->valueint);

    // Turn on the LED to indicate that a message was received
    led_set(1);

    // Set motor speeds
    motor_set_all(y_val->valueint + x_val->valueint, y_val->valueint - x_val->valueint);
    ESP_LOGI(TAG, "A: %d B: %d", y_val->valueint + x_val->valueint, y_val->valueint - x_val->valueint);

    // Reset failsafe timer
    if (xTimerStart(xFailsafeTimer, 0) != pdPASS)
    {
      ESP_LOGW(TAG, "Failed to reset failsafe timer!");
    }
  }

  ret = httpd_ws_send_frame(req, &ws_pkt);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
  }
  free(buf);
  return ret;
}

static const httpd_uri_t ws = {
    .uri    = "/ws",
    .method   = HTTP_GET,
    .handler  = echo_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

typedef struct
{
  const unsigned char* data;
  unsigned int size;
} static_resource_t;

static_resource_t joystick_html_resource = {
  .data = main_joystick_html,
  .size = sizeof(main_joystick_html) / sizeof(char),
};

static_resource_t virtualjoystick_js_resource = {
  .data = virtualjoystick_js_virtualjoystick_js,
  .size = sizeof(virtualjoystick_js_virtualjoystick_js) / sizeof(char),
};

/* URI handler to GET a static resource */
esp_err_t get_handler_static(httpd_req_t *req)
{
  static_resource_t* resource = (static_resource_t*)(req->user_ctx);
  httpd_resp_send(req, (const char*)resource->data, resource->size);
  return ESP_OK;
}

/* URI handler structures for static GETs */
httpd_uri_t uri_get_joystick_html = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_handler_static,
    .user_ctx = &joystick_html_resource,
};
httpd_uri_t uri_get_virtualjoystick_js = {
    .uri      = "/virtualjoystick.js",
    .method   = HTTP_GET,
    .handler  = get_handler_static,
    .user_ctx = &virtualjoystick_js_resource,
};

static httpd_handle_t start_webserver(void)
{
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  // Start the httpd server
  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    // Registering the ws handler
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &uri_get_joystick_html);
    httpd_register_uri_handler(server, &uri_get_virtualjoystick_js);
    return server;
  }

  ESP_LOGI(TAG, "Error starting server!");
  return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
  // Stop the httpd server
  return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                 int32_t event_id, void* event_data)
{
  httpd_handle_t* server = (httpd_handle_t*) arg;
  if (*server) {
    ESP_LOGI(TAG, "Stopping webserver");
    if (stop_webserver(*server) == ESP_OK) {
      *server = NULL;
    } else {
      ESP_LOGE(TAG, "Failed to stop http server");
    }
  }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
              int32_t event_id, void* event_data)
{
  httpd_handle_t* server = (httpd_handle_t*) arg;
  if (*server == NULL) {
    ESP_LOGI(TAG, "Starting webserver");
    *server = start_webserver();
  }
}

// Failsafe has elapsed - stop motors and turn off the LED
void failsafe_elapsed(TimerHandle_t xTimer)
{
  led_set(0);
}

// Create the failsafe timer
void failsafe_init(void)
{
  xFailsafeTimer = xTimerCreate("Failsafe", pdMS_TO_TICKS(1000), pdTRUE, NULL, failsafe_elapsed);
  if (xTimerStart(xFailsafeTimer, 0) != pdPASS)
  {
    ESP_LOGE(TAG, "Failed to start failsafe timer!");
  }
}

void app_main(void)
{
  static httpd_handle_t server = NULL;

  //Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  wifi_init();

  led_init();
  motor_init();
  failsafe_init();

  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

  /* Start the server for the first time */
  server = start_webserver();
}
