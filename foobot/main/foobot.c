#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include <esp_netif.h>
#include <esp_eth.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>

#include <cJSON.h>
#include <freertos/timers.h>

#include "mount.h"
#include "wifi.h"
#include "led.h"
#include "motor.h"

/*
#include "joystick.inc"
#include "virtualjoystick.inc"
*/

static const char *TAG = "foobot_ws";

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

#define SCRATCH_BUFSIZE 8192

TimerHandle_t xFailsafeTimer = NULL;

#define KEY_LENGTH 8

static char master_key[KEY_LENGTH] = {};
static char user_key[KEY_LENGTH] = {};

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
  cJSON* code_val = cJSON_GetObjectItem(root, "code");

  if (x_val && y_val && code_val && x_val->type == cJSON_Number && y_val->type == cJSON_Number && code_val->type == cJSON_String)
  {
    int mode = 0;
    if (!strcmp(code_val->valuestring, master_key))
    {
      mode = 1;
    }
    else if (!strcmp(code_val->valuestring, user_key))
    {
      mode = 2;
    }

    // Print the decoded joystick position
    ESP_LOGI(TAG, "X: %d Y: %d Code: %s Mode: %d", x_val->valueint, y_val->valueint, code_val->valuestring, mode);
  
    if (mode > 0)
    {
      // Turn on the LED to indicate that a message was received
      led_set(1);
  
      // Set motor speeds
      motor_set_all(y_val->valueint + x_val->valueint, y_val->valueint - x_val->valueint);
      ESP_LOGI(TAG, "A: %d B: %d", y_val->valueint + x_val->valueint, y_val->valueint - x_val->valueint);
    }

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
    .is_websocket = true,
};

typedef struct
{
  /* Base path of file storage */
  char base_path[ESP_VFS_PATH_MAX + 1];

  /* Scratch buffer for temporary storage during file transfer */
  char scratch[SCRATCH_BUFSIZE];
} file_server_data;

file_server_data server_data;

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
  const size_t base_pathlen = strlen(base_path);
  size_t pathlen = strlen(uri);

  const char *quest = strchr(uri, '?');
  if (quest)
  {
    pathlen = MIN(pathlen, quest - uri);
  }
  const char *hash = strchr(uri, '#');
  if (hash)
  {
    pathlen = MIN(pathlen, hash - uri);
  }

  if (base_pathlen + pathlen + 1 > destsize)
  {
    /* Full path string won't fit into destination buffer */
    return NULL;
  }

  /* Construct full path (base + path) */
  strcpy(dest, base_path);
  strlcpy(dest + base_pathlen, uri, pathlen + 1);

  /* Return pointer to path, skipping the base */
  return dest + base_pathlen;
}

/* Send HTTP response with a run-time generated html consisting of
 * a list of all files and folders under the requested path.
 * In case of SPIFFS this returns empty list when path is any
 * string other than '/', since SPIFFS doesn't support directories */
static esp_err_t http_resp_dir_html(httpd_req_t *req, const char *dirpath)
{
  char entrypath[FILE_PATH_MAX];
  char entrysize[16];
  const char *entrytype;

  struct dirent *entry;
  struct stat entry_stat;

  DIR *dir = opendir(dirpath);
  const size_t dirpath_len = strlen(dirpath);

  /* Retrieve the base path of file storage to construct the full path */
  strlcpy(entrypath, dirpath, sizeof(entrypath));

  if (!dir)
  {
    ESP_LOGE(TAG, "Failed to stat dir : %s", dirpath);
    /* Respond with 404 Not Found */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Directory does not exist");
    return ESP_FAIL;
  }

  /* Send HTML file header */
  httpd_resp_sendstr_chunk(req, "<!DOCTYPE html><html><body>");

  /* Send file-list table definition and column labels */
  httpd_resp_sendstr_chunk(req,
    "<table class=\"fixed\" border=\"1\">"
    "<col width=\"800px\" /><col width=\"300px\" /><col width=\"300px\" />"
    "<thead><tr><th>Name</th><th>Type</th><th>Size (Bytes)</th></tr></thead>"
    "<tbody>");

  /* Iterate over all files / folders and fetch their names and sizes */
  while ((entry = readdir(dir)) != NULL)
  {
    entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

    strlcpy(entrypath + dirpath_len, entry->d_name, sizeof(entrypath) - dirpath_len);
    if (stat(entrypath, &entry_stat) == -1)
    {
      ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
      continue;
    }
    sprintf(entrysize, "%ld", entry_stat.st_size);
    ESP_LOGI(TAG, "Found %s : %s (%s bytes)", entrytype, entry->d_name, entrysize);

    /* Send chunk of HTML file containing table entries with file name and size */
    httpd_resp_sendstr_chunk(req, "<tr><td><a href=\"");
    httpd_resp_sendstr_chunk(req, req->uri);
    httpd_resp_sendstr_chunk(req, entry->d_name);
    if (entry->d_type == DT_DIR)
    {
      httpd_resp_sendstr_chunk(req, "/");
    }
    httpd_resp_sendstr_chunk(req, "\">");
    httpd_resp_sendstr_chunk(req, entry->d_name);
    httpd_resp_sendstr_chunk(req, "</a></td><td>");
    httpd_resp_sendstr_chunk(req, entrytype);
    httpd_resp_sendstr_chunk(req, "</td><td>");
    httpd_resp_sendstr_chunk(req, entrysize);
    httpd_resp_sendstr_chunk(req, "</td></tr>\n");
  }
  closedir(dir);

  /* Finish the file list table */
  httpd_resp_sendstr_chunk(req, "</tbody></table>");

  /* Send remaining chunk of HTML file to complete it */
  httpd_resp_sendstr_chunk(req, "</body></html>");

  /* Send empty chunk to signal HTTP response completion */
  httpd_resp_sendstr_chunk(req, NULL);
  return ESP_OK;
}

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/*
 * This handler serves files from SPIFFS
 * (or other mounted storage)
 */
static esp_err_t file_get_handler(httpd_req_t *req)
{
  char filepath[FILE_PATH_MAX];
  FILE *fd = NULL;
  struct stat file_stat;

  const char *filename = get_path_from_uri(filepath, ((file_server_data*)req->user_ctx)->base_path,
                                           req->uri, sizeof(filepath));
  if (!filename)
  {
    ESP_LOGE(TAG, "Filename is too long");
    /* Respond with 500 Internal Server Error */
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
    return ESP_FAIL;
  }

  /* If name has trailing '/', respond with directory contents */
  if (filename[strlen(filename) - 1] == '/')
  {
    return http_resp_dir_html(req, filepath);
  }

  if (stat(filepath, &file_stat) == -1)
  {
    /* If file not present on SPIFFS check if URI
     * corresponds to one of the hardcoded paths */
#if 0
    if (strcmp(filename, "/index.html") == 0)
    {
      return index_html_get_handler(req);
    }
    else if (strcmp(filename, "/favicon.ico") == 0)
    {
      return favicon_get_handler(req);
    }
#endif
    ESP_LOGE(TAG, "Failed to stat file : %s", filepath);

    /* Respond with 404 Not Found */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");

    return ESP_FAIL;
  }

  fd = fopen(filepath, "r");
  if (!fd)
  {
      ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
      /* Respond with 500 Internal Server Error */
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
      return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
  set_content_type_from_file(req, filename);

  /* Retrieve the pointer to scratch buffer for temporary storage */
  char *chunk = ((file_server_data*)req->user_ctx)->scratch;
  size_t chunksize;
  do
  {
    /* Read file in chunks into the scratch buffer */
    chunksize = fread(chunk, 1, SCRATCH_BUFSIZE, fd);

    if (chunksize > 0)
    {
      /* Send the buffer contents as HTTP response chunk */
      if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK)
      {
        fclose(fd);
        ESP_LOGE(TAG, "File sending failed!");
        /* Abort sending file */
        httpd_resp_sendstr_chunk(req, NULL);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
       return ESP_FAIL;
      }
    }

    /* Keep looping till the whole file is sent */
  }
  while (chunksize != 0);

  /* Close file after sending complete */
  fclose(fd);
  ESP_LOGI(TAG, "File sending complete");

  /* Respond with an empty chunk to signal HTTP response completion */
#if 1
    httpd_resp_set_hdr(req, "Connection", "close");
#endif
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t file_server = {
    .uri       = "/*",  // Match all URIs of type /path/to/file
    .method    = HTTP_GET,
    .handler   = file_get_handler,
    .user_ctx  = &server_data,
};

typedef struct
{
  const unsigned char* data;
  unsigned int size;
} static_resource_t;

/*
static_resource_t joystick_html_resource = {
  .data = main_joystick_html,
  .size = sizeof(main_joystick_html) / sizeof(char),
};

static_resource_t virtualjoystick_js_resource = {
  .data = virtualjoystick_js_virtualjoystick_js,
  .size = sizeof(virtualjoystick_js_virtualjoystick_js) / sizeof(char),
};
*/

/* URI handler to GET a static resource */
esp_err_t get_handler_static(httpd_req_t *req)
{
  static_resource_t* resource = (static_resource_t*)(req->user_ctx);
  httpd_resp_send(req, (const char*)resource->data, resource->size);
  return ESP_OK;
}

/* URI handler structures for static GETs */
/*
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
*/

static httpd_handle_t start_webserver(void)
{
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

 /* Use the URI wildcard matching function in order to
  * allow the same handler to respond to multiple different
  * target URIs which match the wildcard scheme */
 config.uri_match_fn = httpd_uri_match_wildcard;

  // Start the httpd server
  ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
  if (httpd_start(&server, &config) == ESP_OK) {
    // Registering the ws handler
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &file_server);
    //httpd_register_uri_handler(server, &uri_get_joystick_html);
    //httpd_register_uri_handler(server, &uri_get_virtualjoystick_js);
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

  // Set default codes
  strncpy(master_key, CONFIG_FOOBOT_MASTER_KEY, KEY_LENGTH);
  strncpy(user_key, CONFIG_FOOBOT_USER_KEY, KEY_LENGTH);

  const char* const base_path = "/data";
  ESP_ERROR_CHECK(mount_storage(base_path));
  strncpy(server_data.base_path, base_path, ESP_VFS_PATH_MAX + 1);

  wifi_init();

  led_init();
  motor_init();
  failsafe_init();

  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

  /* Start the server for the first time */
  server = start_webserver();
}

