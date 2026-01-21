#include "wifi_web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char *TAG = "wifi_web";

static httpd_handle_t s_server;
static app_config_t *s_cfg;
static bool s_started;

static bool parse_u16(const char *value, uint16_t *out) {
  if (!value || value[0] == '\0') {
    return false;
  }
  char *end = NULL;
  unsigned long v = strtoul(value, &end, 10);
  if (end == value || *end != '\0' || v > 1440) {
    return false;
  }
  *out = (uint16_t)v;
  return true;
}

static void url_decode(char *dst, const char *src, size_t dst_len) {
  size_t di = 0;
  for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; ++si) {
    if (src[si] == '+') {
      dst[di++] = ' ';
    } else if (src[si] == '%' && src[si + 1] && src[si + 2]) {
      char hex[3] = {src[si + 1], src[si + 2], '\0'};
      dst[di++] = (char)strtoul(hex, NULL, 16);
      si += 2;
    } else {
      dst[di++] = src[si];
    }
  }
  dst[di] = '\0';
}

static void form_get(const char *body, const char *key, char *out, size_t out_len) {
  const char *p = body;
  size_t key_len = strlen(key);
  while (p && *p) {
    const char *eq = strchr(p, '=');
    const char *amp = strchr(p, '&');
    if (!eq) {
      break;
    }
    size_t klen = (size_t)(eq - p);
    if (klen == key_len && strncmp(p, key, key_len) == 0) {
      size_t vlen = amp ? (size_t)(amp - eq - 1) : strlen(eq + 1);
      char tmp[CONFIG_STR_MAX_64];
      if (vlen >= sizeof(tmp)) {
        vlen = sizeof(tmp) - 1;
      }
      memcpy(tmp, eq + 1, vlen);
      tmp[vlen] = '\0';
      url_decode(out, tmp, out_len);
      return;
    }
    if (!amp) {
      break;
    }
    p = amp + 1;
  }
  out[0] = '\0';
}

static esp_err_t handle_root(httpd_req_t *req) {
  char page[2048];
  snprintf(page, sizeof(page),
           "<!doctype html><html><head><meta charset=\"utf-8\">"
           "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
           "<title>AlphaLoc Config</title>"
           "<style>body{font-family:Arial,sans-serif;margin:24px;max-width:560px;}label{display:block;margin:12px 0 4px;}input,select{width:100%%;padding:8px;margin-bottom:8px;}button{padding:10px 14px;}</style>"
           "</head><body><h2>AlphaLoc Config</h2>"
           "<form method=\"POST\" action=\"/save\">"
           "<label>Camera name prefix</label><input name=\"cam_name\" value=\"%s\">"
           "<label>Camera MAC prefix</label><input name=\"cam_mac\" value=\"%s\">"
           "<label>TZ offset (minutes)</label><input name=\"tz\" value=\"%u\">"
           "<label>DST offset (minutes)</label><input name=\"dst\" value=\"%u\">"
           "<label>WiFi mode</label>"
           "<select name=\"wifi_mode\">"
           "<option value=\"0\" %s>AP</option>"
           "<option value=\"1\" %s>STA</option>"
           "</select>"
           "<label>WiFi SSID (STA)</label><input name=\"wifi_ssid\" value=\"%s\">"
           "<label>WiFi pass (STA)</label><input name=\"wifi_pass\" value=\"%s\">"
           "<label>AP SSID</label><input name=\"ap_ssid\" value=\"%s\">"
           "<label>AP pass</label><input name=\"ap_pass\" value=\"%s\">"
           "<button type=\"submit\">Save</button>"
           "</form>"
           "<p>Reboot the device after saving to apply network changes.</p>"
           "</body></html>",
           s_cfg->camera_name_prefix,
           s_cfg->camera_mac_prefix,
           s_cfg->tz_offset_min,
           s_cfg->dst_offset_min,
           s_cfg->wifi_mode == APP_WIFI_MODE_AP ? "selected" : "",
           s_cfg->wifi_mode == APP_WIFI_MODE_STA ? "selected" : "",
           s_cfg->wifi_ssid,
           s_cfg->wifi_pass,
           s_cfg->ap_ssid,
           s_cfg->ap_pass);

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_save(httpd_req_t *req) {
  char body[512];
  int recv = httpd_req_recv(req, body, sizeof(body) - 1);
  if (recv <= 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
  }
  body[recv] = '\0';

  char value[CONFIG_STR_MAX_64];
  form_get(body, "cam_name", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->camera_name_prefix, value, sizeof(s_cfg->camera_name_prefix) - 1);
    s_cfg->camera_name_prefix[sizeof(s_cfg->camera_name_prefix) - 1] = '\0';
  }

  form_get(body, "cam_mac", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->camera_mac_prefix, value, sizeof(s_cfg->camera_mac_prefix) - 1);
    s_cfg->camera_mac_prefix[sizeof(s_cfg->camera_mac_prefix) - 1] = '\0';
  }

  form_get(body, "tz", value, sizeof(value));
  uint16_t tz = 0;
  if (parse_u16(value, &tz)) {
    s_cfg->tz_offset_min = tz;
  }

  form_get(body, "dst", value, sizeof(value));
  uint16_t dst = 0;
  if (parse_u16(value, &dst)) {
    s_cfg->dst_offset_min = dst;
  }

  form_get(body, "wifi_mode", value, sizeof(value));
  if (strcmp(value, "0") == 0) {
    s_cfg->wifi_mode = APP_WIFI_MODE_AP;
  } else if (strcmp(value, "1") == 0) {
    s_cfg->wifi_mode = APP_WIFI_MODE_STA;
  }

  form_get(body, "wifi_ssid", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->wifi_ssid, value, sizeof(s_cfg->wifi_ssid) - 1);
    s_cfg->wifi_ssid[sizeof(s_cfg->wifi_ssid) - 1] = '\0';
  }

  form_get(body, "wifi_pass", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->wifi_pass, value, sizeof(s_cfg->wifi_pass) - 1);
    s_cfg->wifi_pass[sizeof(s_cfg->wifi_pass) - 1] = '\0';
  }

  form_get(body, "ap_ssid", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->ap_ssid, value, sizeof(s_cfg->ap_ssid) - 1);
    s_cfg->ap_ssid[sizeof(s_cfg->ap_ssid) - 1] = '\0';
  }

  form_get(body, "ap_pass", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->ap_pass, value, sizeof(s_cfg->ap_pass) - 1);
    s_cfg->ap_pass[sizeof(s_cfg->ap_pass) - 1] = '\0';
  }

  config_save(s_cfg);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "Saved. Reboot to apply WiFi changes.\n");
}

void wifi_web_start(app_config_t *cfg) {
  if (s_started) {
    return;
  }
  s_cfg = cfg;

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_t *netif = NULL;
  if (s_cfg->wifi_mode == APP_WIFI_MODE_STA) {
    netif = esp_netif_create_default_wifi_sta();
  } else {
    netif = esp_netif_create_default_wifi_ap();
  }
  (void)netif;

  wifi_init_config_t cfg_init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg_init));

  if (s_cfg->wifi_mode == APP_WIFI_MODE_STA) {
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, s_cfg->wifi_ssid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, s_cfg->wifi_pass, sizeof(wifi_cfg.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  } else {
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.ap.ssid, s_cfg->ap_ssid, sizeof(wifi_cfg.ap.ssid) - 1);
    wifi_cfg.ap.ssid_len = strlen((char *)wifi_cfg.ap.ssid);
    strncpy((char *)wifi_cfg.ap.password, s_cfg->ap_pass, sizeof(wifi_cfg.ap.password) - 1);
    wifi_cfg.ap.authmode = strlen((char *)wifi_cfg.ap.password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    wifi_cfg.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
  }

  ESP_ERROR_CHECK(esp_wifi_start());
  if (s_cfg->wifi_mode == APP_WIFI_MODE_STA) {
    esp_wifi_connect();
  }

  httpd_config_t server_cfg = HTTPD_DEFAULT_CONFIG();
  server_cfg.uri_match_fn = httpd_uri_match_wildcard;
  httpd_start(&s_server, &server_cfg);

  httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = handle_root, .user_ctx = NULL};
  httpd_uri_t save = {.uri = "/save", .method = HTTP_POST, .handler = handle_save, .user_ctx = NULL};
  httpd_register_uri_handler(s_server, &root);
  httpd_register_uri_handler(s_server, &save);

  s_started = true;
  ESP_LOGI(TAG, "WiFi web started");
}

void wifi_web_stop(void) {
  if (!s_started) {
    return;
  }
  if (s_server) {
    httpd_stop(s_server);
    s_server = NULL;
  }
  esp_wifi_stop();
  esp_wifi_deinit();
  s_started = false;
  ESP_LOGI(TAG, "WiFi web stopped");
}
