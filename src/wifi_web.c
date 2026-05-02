#include "wifi_web.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble_client.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
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
static esp_netif_t *s_netif_sta = NULL;
static esp_netif_t *s_netif_ap = NULL;
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
static int64_t s_sta_start_us = 0;
static bool s_ap_active = false;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

#ifndef ALPHALOC_WIFI_STA_FALLBACK_TIMEOUT_MS
#define ALPHALOC_WIFI_STA_FALLBACK_TIMEOUT_MS 15000
#endif

#ifndef ALPHALOC_WIFI_RECONNECT_BACKOFF_MS
#define ALPHALOC_WIFI_RECONNECT_BACKOFF_MS 1500
#endif

static esp_timer_handle_t s_reconnect_timer = NULL;
static uint8_t s_sta_retry_count = 0;

static void wifi_cleanup_resources(void) {
  if (s_server) {
    httpd_stop(s_server);
    s_server = NULL;
  }
  if (s_reconnect_timer) {
    esp_timer_stop(s_reconnect_timer);
    esp_timer_delete(s_reconnect_timer);
    s_reconnect_timer = NULL;
  }
  esp_wifi_stop();
  esp_wifi_deinit();
  if (s_netif_sta) {
    esp_netif_destroy(s_netif_sta);
    s_netif_sta = NULL;
  }
  if (s_netif_ap) {
    esp_netif_destroy(s_netif_ap);
    s_netif_ap = NULL;
  }
  s_netif = NULL;
  if (s_wifi_handlers_registered) {
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                 &wifi_event_handler);
    s_wifi_handlers_registered = false;
  }
  esp_event_loop_delete_default();
  s_started = false;
  s_ap_active = false;
  s_sta_retry_count = 0;
}

static void wifi_reconnect_cb(void *arg) {
  (void)arg;
  if (s_started && !s_ap_active) {
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "WiFi reconnect start failed: %s", esp_err_to_name(err));
    }
  }
}

static bool start_softap(void) {
  app_config_t cfg;
  if (!config_get_snapshot(s_cfg, &cfg)) {
    cfg = *s_cfg;
    config_validate(&cfg);
  }
  if (!s_netif_ap) {
    s_netif_ap = esp_netif_create_default_wifi_ap();
  }
  wifi_config_t wifi_cfg = {0};
  strncpy((char *)wifi_cfg.ap.ssid, cfg.ap_ssid,
          sizeof(wifi_cfg.ap.ssid) - 1);
  wifi_cfg.ap.ssid_len = strlen((char *)wifi_cfg.ap.ssid);
  strncpy((char *)wifi_cfg.ap.password, cfg.ap_pass,
          sizeof(wifi_cfg.ap.password) - 1);
  wifi_cfg.ap.authmode = strlen((char *)wifi_cfg.ap.password) == 0
                             ? WIFI_AUTH_OPEN
                             : WIFI_AUTH_WPA2_PSK;
  wifi_cfg.ap.max_connection = 4;

  esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to switch to AP mode: %s", esp_err_to_name(err));
    return false;
  }
  err = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure AP: %s", esp_err_to_name(err));
    return false;
  }
  s_ap_active = true;
  ESP_LOGI(TAG, "Started AP fallback: ssid=%s", cfg.ap_ssid);
  return true;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base != WIFI_EVENT) {
    return;
  }

  if (event_id == WIFI_EVENT_STA_CONNECTED) {
    s_sta_retry_count = 0;
    ESP_LOGI(TAG, "WiFi STA connected");
    return;
  }

  if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *disc =
        (const wifi_event_sta_disconnected_t *)event_data;
    unsigned reason = disc ? (unsigned)disc->reason : 0;
    int64_t elapsed_ms = 0;
    if (s_sta_start_us > 0) {
      elapsed_ms = (esp_timer_get_time() - s_sta_start_us) / 1000;
    }

    ESP_LOGW(TAG, "WiFi disconnected, reason=%u", reason);

    if (elapsed_ms >= ALPHALOC_WIFI_STA_FALLBACK_TIMEOUT_MS && !s_ap_active) {
      ESP_LOGW(TAG, "STA connect timed out, falling back to AP");
      start_softap();
      return;
    }

    if (s_reconnect_timer) {
      uint64_t delay_us =
          (uint64_t)ALPHALOC_WIFI_RECONNECT_BACKOFF_MS * 1000ULL;
      if (s_sta_retry_count < 5) {
        delay_us *= (uint64_t)(s_sta_retry_count + 1);
        s_sta_retry_count++;
      }
      esp_timer_stop(s_reconnect_timer);
      esp_timer_start_once(s_reconnect_timer, delay_us);
    }
  }
}

static bool parse_i16_range(const char *value, int16_t min, int16_t max,
                            int16_t *out) {
  if (!value || value[0] == '\0') {
    return false;
  }
  char *end = NULL;
  long v = strtol(value, &end, 10);
  if (end == value || *end != '\0' || v < min || v > max) {
    return false;
  }
  *out = (int16_t)v;
  return true;
}

static bool parse_u32_range(const char *value, uint32_t max, uint32_t *out) {
  if (!value || value[0] == '\0') {
    return false;
  }
  char *end = NULL;
  unsigned long v = strtoul(value, &end, 10);
  if (end == value || *end != '\0' || v > max) {
    return false;
  }
  *out = (uint32_t)v;
  return true;
}

static void url_decode(char *dst, const char *src, size_t dst_len) {
  size_t di = 0;
  for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; ++si) {
    if (src[si] == '+') {
      dst[di++] = ' ';
    } else if (src[si] == '%' && src[si + 1] && src[si + 2]) {
      if (isxdigit((unsigned char)src[si + 1]) &&
          isxdigit((unsigned char)src[si + 2])) {
        char hex[3] = {src[si + 1], src[si + 2], '\0'};
        dst[di++] = (char)strtoul(hex, NULL, 16);
        si += 2;
      }
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

static void html_escape_attr(char *dst, size_t dst_len, const char *src) {
  size_t di = 0;
  if (!dst || dst_len == 0) {
    return;
  }
  if (!src) {
    dst[0] = '\0';
    return;
  }
  for (size_t si = 0; src[si] != '\0' && di + 1 < dst_len; ++si) {
    const char *rep = NULL;
    switch (src[si]) {
    case '&':
      rep = "&amp;";
      break;
    case '"':
      rep = "&quot;";
      break;
    case '<':
      rep = "&lt;";
      break;
    case '>':
      rep = "&gt;";
      break;
    default:
      dst[di++] = src[si];
      continue;
    }
    size_t rep_len = strlen(rep);
    if (di + rep_len >= dst_len) {
      break;
    }
    memcpy(dst + di, rep, rep_len);
    di += rep_len;
  }
  dst[di] = '\0';
}

static esp_err_t handle_root(httpd_req_t *req) {
  app_config_t cfg;
  if (!config_get_snapshot(s_cfg, &cfg)) {
    cfg = *s_cfg;
    config_validate(&cfg);
  }
  const size_t page_len = 6144;
  char *page = malloc(page_len);
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
  char camera_name[CONFIG_STR_MAX_32 * 6];
  char camera_mac[CONFIG_STR_MAX_18 * 6];
  char wifi_ssid[CONFIG_STR_MAX_32 * 6];
  char wifi_pass[CONFIG_STR_MAX_64 * 6];
  char ap_ssid[CONFIG_STR_MAX_32 * 6];
  char ap_pass[CONFIG_STR_MAX_64 * 6];
  html_escape_attr(camera_name, sizeof(camera_name), cfg.camera_name_prefix);
  html_escape_attr(camera_mac, sizeof(camera_mac), cfg.camera_mac_prefix);
  html_escape_attr(wifi_ssid, sizeof(wifi_ssid), cfg.wifi_ssid);
  html_escape_attr(wifi_pass, sizeof(wifi_pass), cfg.wifi_pass);
  html_escape_attr(ap_ssid, sizeof(ap_ssid), cfg.ap_ssid);
  html_escape_attr(ap_pass, sizeof(ap_pass), cfg.ap_pass);
  int written = snprintf(
      page, page_len,
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
      "<label>TZ offset (minutes)</label><input name=\"tz\" value=\"%d\">"
      "<label>DST offset (minutes)</label><input name=\"dst\" value=\"%d\">"
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
      camera_name, camera_mac, (int)cfg.tz_offset_min, (int)cfg.dst_offset_min,
      wifi_ssid, wifi_pass, ap_ssid, ap_pass, (unsigned)cfg.max_gps_age_s);
  if (written < 0 || (size_t)written >= page_len) {
    free(page);
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Page too large");
  }

  httpd_resp_set_type(req, "text/html");
  esp_err_t res = httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
  free(page);
  return res;
}

static esp_err_t handle_save(httpd_req_t *req) {
  char body[512];
  if (req->content_len >= sizeof(body)) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too large");
  }
  size_t total = 0;
  while (total < req->content_len) {
    int recv = httpd_req_recv(req, body + total, req->content_len - total);
    if (recv <= 0) {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
    }
    total += (size_t)recv;
  }
  if (total == 0) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
  }
  body[total] = '\0';

  app_config_t next;
  if (!config_get_snapshot(s_cfg, &next)) {
    next = *s_cfg;
    config_validate(&next);
  }

  char value[CONFIG_STR_MAX_64];
  form_get(body, "cam_name", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(next.camera_name_prefix, value, sizeof(next.camera_name_prefix) - 1);
    next.camera_name_prefix[sizeof(next.camera_name_prefix) - 1] = '\0';
  }

  form_get(body, "cam_mac", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(next.camera_mac_prefix, value, sizeof(next.camera_mac_prefix) - 1);
    next.camera_mac_prefix[sizeof(next.camera_mac_prefix) - 1] = '\0';
  }

  form_get(body, "tz", value, sizeof(value));
  int16_t tz = 0;
  if (parse_i16_range(value, -1440, 1440, &tz)) {
    next.tz_offset_min = tz;
  }

  form_get(body, "dst", value, sizeof(value));
  int16_t dst = 0;
  if (parse_i16_range(value, -1440, 1440, &dst)) {
    next.dst_offset_min = dst;
  }

  form_get(body, "wifi_ssid", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(next.wifi_ssid, value, sizeof(next.wifi_ssid) - 1);
    next.wifi_ssid[sizeof(next.wifi_ssid) - 1] = '\0';
  }

  form_get(body, "wifi_pass", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(next.wifi_pass, value, sizeof(next.wifi_pass) - 1);
    next.wifi_pass[sizeof(next.wifi_pass) - 1] = '\0';
  }

  form_get(body, "ap_ssid", value, sizeof(value));
  if (value[0] != '\0') {
    strncpy(next.ap_ssid, value, sizeof(next.ap_ssid) - 1);
    next.ap_ssid[sizeof(next.ap_ssid) - 1] = '\0';
  }

  form_get(body, "ap_pass", value, sizeof(value));
  if (value[0] != '\0' || strstr(body, "ap_pass=") != NULL) {
    strncpy(next.ap_pass, value, sizeof(next.ap_pass) - 1);
    next.ap_pass[sizeof(next.ap_pass) - 1] = '\0';
  }

  form_get(body, "max_age_s", value, sizeof(value));
  uint32_t max_age = 0;
  if (parse_u32_range(value, 86400, &max_age)) {
    next.max_gps_age_s = max_age;
  }

  if (!config_apply(s_cfg, &next)) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "Save failed");
  }
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
  ESP_ERROR_CHECK(
      esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                 &wifi_event_handler, NULL));
  s_wifi_handlers_registered = true;
  s_netif_sta = esp_netif_create_default_wifi_sta();
  s_netif = s_netif_sta;

  wifi_init_config_t cfg_init = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg_init));

  const esp_timer_create_args_t reconnect_timer_args = {
      .callback = wifi_reconnect_cb,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "wifi_reconnect",
      .skip_unhandled_events = true,
  };
  if (esp_timer_create(&reconnect_timer_args, &s_reconnect_timer) != ESP_OK) {
    ESP_LOGW(TAG, "Failed to create WiFi reconnect timer");
  }

  app_config_t cfg_snapshot;
  if (!config_get_snapshot(s_cfg, &cfg_snapshot)) {
    cfg_snapshot = *s_cfg;
    config_validate(&cfg_snapshot);
  }
  wifi_config_t wifi_cfg = {0};
  strncpy((char *)wifi_cfg.sta.ssid, cfg_snapshot.wifi_ssid,
          sizeof(wifi_cfg.sta.ssid) - 1);
  strncpy((char *)wifi_cfg.sta.password, cfg_snapshot.wifi_pass,
          sizeof(wifi_cfg.sta.password) - 1);
  esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set WiFi STA mode: %s", esp_err_to_name(err));
    wifi_cleanup_resources();
    return;
  }
  err = esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure WiFi STA: %s", esp_err_to_name(err));
    if (!start_softap()) {
      wifi_cleanup_resources();
      return;
    }
  }

  err = esp_wifi_start();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(err));
    wifi_cleanup_resources();
    return;
  }
  err = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to enable WiFi power save: %s", esp_err_to_name(err));
  }
  s_sta_start_us = esp_timer_get_time();
  if (!s_ap_active) {
    err = esp_wifi_connect();
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "WiFi connect start failed: %s", esp_err_to_name(err));
    }
  }

  httpd_config_t server_cfg = HTTPD_DEFAULT_CONFIG();
  server_cfg.uri_match_fn = httpd_uri_match_wildcard;
  server_cfg.stack_size = 8192;
  err = httpd_start(&s_server, &server_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(err));
    wifi_cleanup_resources();
    return;
  }

  httpd_uri_t root = {
      .uri = "/", .method = HTTP_GET, .handler = handle_root, .user_ctx = NULL};
  httpd_uri_t save = {.uri = "/save",
                      .method = HTTP_POST,
                      .handler = handle_save,
                      .user_ctx = NULL};
  err = httpd_register_uri_handler(s_server, &root);
  if (err == ESP_OK) {
    err = httpd_register_uri_handler(s_server, &save);
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "HTTP route registration failed: %s", esp_err_to_name(err));
    wifi_cleanup_resources();
    return;
  }

  s_started = true;
  ESP_LOGI(TAG, "WiFi web started");
}

void wifi_web_stop(void) {
  if (!s_started) {
    return;
  }
  wifi_cleanup_resources();
  ESP_LOGI(TAG, "WiFi web stopped");
}
