#include "wifi_web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_client.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "gps.h"

#ifndef ALPHALOC_BATTERY_MONITOR
#define ALPHALOC_BATTERY_MONITOR 0
#endif

#if ALPHALOC_BATTERY_MONITOR
#include "battery.h"
#endif

static const char *TAG = "wifi_web";

static httpd_handle_t s_server;
static app_config_t *s_cfg;
static esp_netif_t *s_netif = NULL;
static const char *constellation_to_str(gps_constellation_t mask) {
  if (mask == (GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GLONASS)) {
    return "gps+glonass";
  }
  if (mask == GPS_CONSTELLATION_GPS) {
    return "gps";
  }
  if (mask == GPS_CONSTELLATION_GLONASS) {
    return "glonass";
  }
  return "none";
}
static bool s_started;
static bool s_wifi_handlers_registered = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *disc =
        (const wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW(TAG, "WiFi disconnected, reason=%u; retrying",
             disc ? (unsigned)disc->reason : 0);
    esp_wifi_connect();
  }
}

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

static void form_get(const char *body, const char *key, char *out,
                     size_t out_len) {
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
  char *page = malloc(4096);
  if (!page) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "No memory");
  }
  gps_status_t gps_status = {0};
  const char *gps_const_str = "n/a";
  if (gps_get_status(&gps_status)) {
    gps_const_str = constellation_to_str(gps_status.constellations);
  }
  bool cam_connected = ble_client_is_connected();
  bool cam_bonded = ble_client_is_bonded();
  const char *cam_dot_class =
      cam_connected ? (cam_bonded ? "dot-green" : "dot-blue") : "dot-red";
  const char *cam_conn_str = cam_connected ? "connected" : "disconnected";
  const char *cam_bond_str = cam_bonded ? "bonded" : "not bonded";
  const char *gps_dot_class = gps_status.has_lock ? "dot-green" : "dot-red";
  const char *gps_lock_str = gps_status.has_lock ? "lock" : "no lock";
#if ALPHALOC_BATTERY_MONITOR
  battery_status_t bat = {0};
  bool bat_ok = battery_get_status(&bat) && bat.valid;
  const char *bat_dot_class = "dot-gray";
  char bat_text[64] = "Battery: n/a";
  if (bat_ok) {
    if (bat.percent > 50.0f) {
      bat_dot_class = "dot-green";
    } else if (bat.percent > 30.0f) {
      bat_dot_class = "dot-yellow";
    } else {
      bat_dot_class = "dot-red";
    }
    snprintf(bat_text, sizeof(bat_text), "Battery: %.0f%% (%.2fV)",
             (double)bat.percent, (double)bat.voltage_v);
  }
#endif
  snprintf(
      page, 4096,
      "<!doctype html><html><head><meta charset=\"utf-8\">"
      "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      "<title>AlphaLoc Config</title>"
      "<style>body{font-family:Arial,sans-serif;margin:24px;max-width:560px;}"
      ".statusbar{display:flex;flex-wrap:wrap;gap:10px 14px;align-items:center;"
      "margin:10px 0 16px;padding:8px 10px;border:1px solid #ddd;border-radius:8px;"
      "background:#f6f8fb;font-size:14px;}"
      ".statuslabel{font-weight:600;margin-right:2px;}"
      ".statusitem{display:flex;align-items:center;gap:6px;white-space:nowrap;}"
      ".dot{width:10px;height:10px;border-radius:50%%;display:inline-block;}"
      ".dot-green{background:#2e9a44;}.dot-red{background:#d9534f;}.dot-blue{background:#2f6fdb;}"
      ".dot-yellow{background:#f0b429;}.dot-gray{background:#9aa3af;}"
      "label{display:block;margin:12px 0 "
      "4px;}input,select{width:100%%;padding:8px;margin-bottom:8px;}button{"
      "padding:10px 14px;}</style>"
      "</head><body><h2>AlphaLoc Config</h2>"
      "<div class=\"statusbar\">"
      "<span class=\"statuslabel\">Status</span>"
      "<div class=\"statusitem\"><span class=\"dot %s\"></span>"
      "<span>GPS: %s, %u sats, %s</span></div>"
      "<div class=\"statusitem\"><span class=\"dot %s\"></span>"
      "<span>Camera: %s, %s</span></div>"
#if ALPHALOC_BATTERY_MONITOR
      "<div class=\"statusitem\"><span class=\"dot %s\"></span>"
      "<span>%s</span></div>"
#endif
      "</div>"
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
      "<label>Max GPS age (seconds)</label><input name=\"max_age_s\" "
      "value=\"%u\">"
      "<button type=\"submit\">Save</button>"
      "</form>"
      "<p>Reboot the device after saving to apply network changes.</p>"
      "</body></html>",
      gps_dot_class, gps_lock_str, (unsigned)gps_status.satellites,
      gps_const_str, cam_dot_class, cam_conn_str, cam_bond_str,
#if ALPHALOC_BATTERY_MONITOR
      bat_dot_class, bat_text,
#endif
      s_cfg->camera_name_prefix, s_cfg->camera_mac_prefix, s_cfg->tz_offset_min,
      s_cfg->dst_offset_min,
      s_cfg->wifi_mode == APP_WIFI_MODE_AP ? "selected" : "",
      s_cfg->wifi_mode == APP_WIFI_MODE_STA ? "selected" : "", s_cfg->wifi_ssid,
      s_cfg->wifi_pass, s_cfg->ap_ssid, s_cfg->ap_pass,
      (unsigned)s_cfg->max_gps_age_s);

  httpd_resp_set_type(req, "text/html");
  esp_err_t res = httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
  free(page);
  return res;
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
    strncpy(s_cfg->camera_name_prefix, value,
            sizeof(s_cfg->camera_name_prefix) - 1);
    s_cfg->camera_name_prefix[sizeof(s_cfg->camera_name_prefix) - 1] = '\0';
  }

  form_get(body, "cam_mac", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(s_cfg->camera_mac_prefix, value,
            sizeof(s_cfg->camera_mac_prefix) - 1);
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
  }
  else if (strcmp(value, "1") == 0) {
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

  form_get(body, "max_age_s", value, sizeof(value));
  uint16_t max_age = 0;
  if (parse_u16(value, &max_age)) {
    s_cfg->max_gps_age_s = max_age;
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

  if (s_cfg->wifi_mode == APP_WIFI_MODE_STA) {
    ESP_ERROR_CHECK(
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                   &wifi_event_handler, NULL));
    s_wifi_handlers_registered = true;
    s_netif = esp_netif_create_default_wifi_sta();
  }
  else {
    s_netif = esp_netif_create_default_wifi_ap();
  }

  wifi_init_config_t cfg_init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg_init));

  if (s_cfg->wifi_mode == APP_WIFI_MODE_STA) {
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, s_cfg->wifi_ssid,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, s_cfg->wifi_pass,
            sizeof(wifi_cfg.sta.password) - 1);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  }
  else {
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.ap.ssid, s_cfg->ap_ssid,
            sizeof(wifi_cfg.ap.ssid) - 1);
    wifi_cfg.ap.ssid_len = strlen((char *)wifi_cfg.ap.ssid);
    strncpy((char *)wifi_cfg.ap.password, s_cfg->ap_pass,
            sizeof(wifi_cfg.ap.password) - 1);
    wifi_cfg.ap.authmode = strlen((char *)wifi_cfg.ap.password) == 0
                               ? WIFI_AUTH_OPEN
                               : WIFI_AUTH_WPA2_PSK;
    wifi_cfg.ap.max_connection = 4;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
  }

  ESP_ERROR_CHECK(esp_wifi_start());
  if (s_cfg->wifi_mode == APP_WIFI_MODE_STA) {
    // Enable WiFi power saving in STA mode (Issue 3.2)
    // esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    // ESP_LOGI(TAG, "WiFi power saving enabled");

    esp_wifi_connect();
  }

  httpd_config_t server_cfg = HTTPD_DEFAULT_CONFIG();
  server_cfg.uri_match_fn = httpd_uri_match_wildcard;
  server_cfg.stack_size = 8192;
  httpd_start(&s_server, &server_cfg);

  httpd_uri_t root = {
      .uri = "/", .method = HTTP_GET, .handler = handle_root, .user_ctx = NULL};
  httpd_uri_t save = {.uri = "/save",
                      .method = HTTP_POST,
                      .handler = handle_save,
                      .user_ctx = NULL};
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

  // Clean up network interface to prevent memory leak
  if (s_netif) {
    esp_netif_destroy(s_netif);
    s_netif = NULL;
  }

  if (s_wifi_handlers_registered) {
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                 &wifi_event_handler);
    s_wifi_handlers_registered = false;
  }

  // Clean up event loop
  esp_event_loop_delete_default();

  s_started = false;
  ESP_LOGI(TAG, "WiFi web stopped");
}
