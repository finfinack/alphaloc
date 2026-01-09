// References:
// - https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_client/tutorial/Gatt_Client_Example_Walkthrough.md
// - https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_security_client/tutorial/Gatt_Security_Client_Example_Walkthrough.md

#include <stdio.h>
#include <string.h>

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "main.h"

bool uuid128_equals(const uint8_t a[ESP_UUID_LEN_128], const uint8_t b[ESP_UUID_LEN_128])
{
  for (int i = 0; i < ESP_UUID_LEN_128; i++)
  {
    if (a[i] != b[i])
    {
      return false;
    }
  }
  return true;
}

size_t buildLocationPayload(uint8_t *buf, uint32_t latitude, uint32_t longitude, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint16_t tzOffMin, uint16_t dstOffMin)
{
  const bool sendTzDst = (tzOffMin > 0 || dstOffMin > 0);
  const size_t totalLen = sendTzDst ? 95 : 91;

  putBE16(buf + 0, sendTzDst ? 0x5D : 0x59); // Length field

  // Fixed header
  buf[2] = 0x08;
  buf[3] = 0x02;
  buf[4] = 0xFC;

  // Timezone flag
  buf[5] = sendTzDst ? 0x03 : 0x00;

  // Fixed bytes
  buf[6] = 0x00;
  buf[7] = 0x00;
  buf[8] = 0x10;
  buf[9] = 0x10;
  buf[10] = 0x10;

  // Coordinates
  putBE32(buf + 11, latitude);
  putBE32(buf + 15, longitude);

  // Time
  putBE16(buf + 19, year);
  buf[21] = month;
  buf[22] = day;
  buf[23] = hour;
  buf[24] = minute;
  buf[25] = second;

  // Reserved area (always zero)
  for (size_t i = 26; i < (sendTzDst ? 95 : 91); i++)
  {
    buf[i] = 0x00;
  }

  // TZ / DST
  if (sendTzDst)
  {
    putBE16(buf + 91, tzOffMin);
    putBE16(buf + 93, dstOffMin);
  }

  return totalLen;
}

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static void rescan_timer_cb(void *arg)
{
  ESP_LOGI(TAG_BLE_SCAN, "Restarting BLE scan after delay");
  esp_ble_gap_start_scanning(BLE_SCAN_TIME);
}

static void disconnect_and_start_reconnect()
{
  connect = false;
  connected = false;
  get_location_service = false;
  get_remote_service = false;

  static esp_timer_handle_t timer;
  esp_timer_create_args_t timer_args = {
      .callback = rescan_timer_cb,
      .name = "rescan_timer"};

  // Start a timer to restart scanning after BLE_RESCAN_PERIOD_WHEN_NOT_CONNECTED ms
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_once(timer, BLE_RESCAN_PERIOD_WHEN_NOT_CONNECTED * 1000);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

  switch (event)
  {
  /* GATT Client registration */
  case ESP_GATTC_REG_EVT:
    /* Registered */
    ESP_LOGI(TAG_BLE_CONN, "GATT client register, status %u, app_id %u, gattc_if %d", p_data->reg.status, p_data->reg.app_id, gattc_if);
    esp_ble_gap_config_local_privacy(true);
    break;

  case ESP_GATTC_UNREG_EVT:
    /* Unregistered */
    ESP_LOGI(TAG_BLE_CONN, "UNREG_EVT");
    esp_err_t ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
      ESP_LOGE(TAG_BLE_CONN, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }
    break;

  /* GATT Client connection to server */
  case ESP_GATTC_CONNECT_EVT:
    /* GATT Client connected to server */
    ESP_LOGI(TAG_BLE_CONN, "Connected, conn_id %d, remote " ESP_BD_ADDR_STR "", p_data->connect.conn_id,
             ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
    break;

  case ESP_GATTC_DISCONNECT_EVT:
    ESP_LOGI(TAG_BLE_CONN, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
             ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);
    disconnect_and_start_reconnect();

    break;

  case ESP_GATTC_OPEN_EVT:
    /* Profile connection opened */
    if (param->open.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG_BLE_CONN, "Open failed, status %x", p_data->open.status);
      break;
    }
    ESP_LOGI(TAG_BLE_CONN, "Open successfully, MTU %d", p_data->open.mtu);
    gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
    memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
    esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
    if (mtu_ret)
    {
      ESP_LOGE(TAG_BLE_CONN, "config MTU error, error code = %x", mtu_ret);
    }
    break;

  case ESP_GATTC_CLOSE_EVT:
    /* Profile connection closed */
    if (param->close.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG_BLE_CONN, "Close failed, status %d", p_data->close.status);
    }
    else
    {
      ESP_LOGI(TAG_BLE_CONN, "Close success");
    }

    disconnect_and_start_reconnect();
    break;

  case ESP_GATTC_CFG_MTU_EVT:
    /* MTU has been set */
    ESP_LOGI(TAG_BLE_CONN, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
    esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &CAM_LOCATION_SERVICE);
    break;

  case ESP_GATTC_SEARCH_RES_EVT:
  {
    /* Search result is in */
    esp_gatt_id_t *srvc_id = &p_data->search_res.srvc_id;
    uint16_t conn_id = p_data->search_res.conn_id;

    ESP_LOGI(TAG_BLE_CONN, "Service search result, conn_id = %x", conn_id);

    if (srvc_id->uuid.len == ESP_UUID_LEN_128)
    {
      if (uuid128_equals(srvc_id->uuid.uuid.uuid128, CAM_LOCATION_SERVICE.uuid.uuid128))
      {
        ESP_LOGI(TAG_BLE_CONN, "Found location service");
        gl_profile_tab[PROFILE_A_APP_ID].location_service_start_handle = p_data->search_res.start_handle;
        gl_profile_tab[PROFILE_A_APP_ID].location_service_end_handle = p_data->search_res.end_handle;
        get_location_service = true;
        break;
      }
      else if (uuid128_equals(srvc_id->uuid.uuid.uuid128, CAM_REMOTE_SERVICE.uuid.uuid128))
      {
        ESP_LOGI(TAG_BLE_CONN, "Found remote service");
        gl_profile_tab[PROFILE_A_APP_ID].remote_service_start_handle = p_data->search_res.start_handle;
        gl_profile_tab[PROFILE_A_APP_ID].remote_service_end_handle = p_data->search_res.end_handle;
        // get_remote_service = true;
        break;
      }
      else
      {
        ESP_LOGD(TAG_BLE_CONN, "Service is not a known service");
      }
    }
    else
    {
      ESP_LOGD(TAG_BLE_CONN, "Service is not based on a UUID128");
      break;
    }

    break;
  }

  case ESP_GATTC_SEARCH_CMPL_EVT:
    /* Search is complete */
    if (p_data->search_cmpl.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG_BLE_CONN, "Service search failed, status %x", p_data->search_cmpl.status);
      break;
    }
    if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
    {
      ESP_LOGD(TAG_BLE_CONN, "Get service information from remote device");
    }
    else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
    {
      ESP_LOGD(TAG_BLE_CONN, "Get service information from flash");
    }
    else
    {
      ESP_LOGD(TAG_BLE_CONN, "unknown service source");
    }

    ESP_LOGI(TAG_BLE_CONN, "Service search complete");
    if (get_location_service)
    {
      ESP_LOGD(TAG_BLE_CONN, "Reading Location Service attr");

      uint16_t count = 0;
      uint16_t offset = 0;
      esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                  ESP_GATT_DB_CHARACTERISTIC,
                                                                  gl_profile_tab[PROFILE_A_APP_ID].location_service_start_handle,
                                                                  gl_profile_tab[PROFILE_A_APP_ID].location_service_end_handle,
                                                                  INVALID_HANDLE,
                                                                  &count);
      if (ret_status != ESP_GATT_OK)
      {
        ESP_LOGE(TAG_BLE_CONN, "esp_ble_gattc_get_attr_count error, %d", __LINE__);
        break;
      }

      if (count > 0)
      {
        char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (!char_elem_result)
        {
          ESP_LOGE(TAG_BLE_CONN, "gattc no mem");
          break;
        }
        else
        {
          ESP_LOGD(TAG_BLE_CONN, "Reading Location Service attr chars");
          ret_status = esp_ble_gattc_get_all_char(gattc_if,
                                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                  gl_profile_tab[PROFILE_A_APP_ID].location_service_start_handle,
                                                  gl_profile_tab[PROFILE_A_APP_ID].location_service_end_handle,
                                                  char_elem_result,
                                                  &count,
                                                  offset);
          if (ret_status != ESP_GATT_OK)
          {
            ESP_LOGE(TAG_BLE_CONN, "esp_ble_gattc_get_all_char error, %d", __LINE__);
            free(char_elem_result);
            char_elem_result = NULL;
            break;
          }
          if (count > 0)
          {
            for (int i = 0; i < count; ++i)
            {
              ESP_LOGD(TAG_BLE_CONN, "Iterating over chars: %d", i);
              if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && char_elem_result[i].uuid.uuid.uuid16 == CAM_LOCATION_SERVICE_FEATURE_CHAR_UUID && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_READ))
              {
                /* Read characteristic with encryption requirement
                 * GATT layer will automatically trigger encryption if needed */
                ESP_LOGI(TAG_BLE_CONN, "Found Location Service Feature Char. Reading... (triggers secure connection)");
                gl_profile_tab[PROFILE_A_APP_ID].feature_char_handle = char_elem_result[i].char_handle;
                esp_ble_gattc_read_char(gattc_if,
                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                        gl_profile_tab[PROFILE_A_APP_ID].feature_char_handle,
                                        ESP_GATT_AUTH_REQ_NO_MITM);
              }
              else if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && char_elem_result[i].uuid.uuid.uuid16 == CAM_LOCATION_SERVICE_INFO_CHAR_UUID && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE))
              {
                ESP_LOGI(TAG_BLE_CONN, "Found Location Info Lock Char");
                gl_profile_tab[PROFILE_A_APP_ID].location_char_handle = char_elem_result[i].char_handle;
              }
              else if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && char_elem_result[i].uuid.uuid.uuid16 == CAM_LOCATION_SERVICE_LOCK_CHAR_UUID && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE))
              {
                ESP_LOGI(TAG_BLE_CONN, "Found Location Service Lock Char");
                gl_profile_tab[PROFILE_A_APP_ID].lock_char_handle = char_elem_result[i].char_handle;
              }
              else if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && char_elem_result[i].uuid.uuid.uuid16 == CAM_LOCATION_SERVICE_UPDATE_CHAR_UUID && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE))
              {
                ESP_LOGI(TAG_BLE_CONN, "Found Location Service Update Char");
                gl_profile_tab[PROFILE_A_APP_ID].update_char_handle = char_elem_result[i].char_handle;
              }
            }
          }
        }
        free(char_elem_result);
        char_elem_result = NULL;
      }
      else
      {
        ESP_LOGI(TAG_BLE_CONN, "Reading attr count: No results");
      }
    }
    else if (get_remote_service)
    {
      ESP_LOGD(TAG_BLE_CONN, "Reading Remote Service attr");

      uint16_t count = 0;
      uint16_t offset = 0;
      esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                  ESP_GATT_DB_CHARACTERISTIC,
                                                                  gl_profile_tab[PROFILE_A_APP_ID].remote_service_start_handle,
                                                                  gl_profile_tab[PROFILE_A_APP_ID].remote_service_end_handle,
                                                                  INVALID_HANDLE,
                                                                  &count);
      if (ret_status != ESP_GATT_OK)
      {
        ESP_LOGE(TAG_BLE_CONN, "esp_ble_gattc_get_attr_count error, %d", __LINE__);
        break;
      }

      if (count > 0)
      {
        char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
        if (!char_elem_result)
        {
          ESP_LOGE(TAG_BLE_CONN, "gattc no mem");
          break;
        }
        else
        {
          ESP_LOGD(TAG_BLE_CONN, "Reading Remote Service attr chars");
          ret_status = esp_ble_gattc_get_all_char(gattc_if,
                                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                  gl_profile_tab[PROFILE_A_APP_ID].remote_service_start_handle,
                                                  gl_profile_tab[PROFILE_A_APP_ID].remote_service_end_handle,
                                                  char_elem_result,
                                                  &count,
                                                  offset);
          if (ret_status != ESP_GATT_OK)
          {
            ESP_LOGE(TAG_BLE_CONN, "esp_ble_gattc_get_all_char error, %d", __LINE__);
            free(char_elem_result);
            char_elem_result = NULL;
            break;
          }
          if (count > 0)
          {
            for (int i = 0; i < count; ++i)
            {
              ESP_LOGD(TAG_BLE_CONN, "Iterating over chars: %d", i);
              if (char_elem_result[i].uuid.len == ESP_UUID_LEN_16 && char_elem_result[i].uuid.uuid.uuid16 == CAM_REMOTE_SERVICE_NOTIFY_CHAR_UUID && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
              {
                ESP_LOGI(TAG_BLE_CONN, "Found Remote Service Notify Char. Registering...");
                gl_profile_tab[PROFILE_A_APP_ID].notify_char_handle = char_elem_result[i].char_handle;
                esp_ble_gattc_register_for_notify(gattc_if,
                                                  gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                                  gl_profile_tab[PROFILE_A_APP_ID].notify_char_handle);
              }
            }
          }
        }
        free(char_elem_result);
        char_elem_result = NULL;
      }
      else
      {
        ESP_LOGI(TAG_BLE_CONN, "Reading attr count: No results");
      }
    }

    break;

  case ESP_GATTC_REG_FOR_NOTIFY_EVT:
  {
    if (p_data->reg_for_notify.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG_BLE_CONN, "Notification register failed, status %x", p_data->reg_for_notify.status);
      break;
    }
    else
    {
      ESP_LOGI(TAG_BLE_CONN, "Notification register successfully");
    }

    // uint16_t count = 0;
    // uint16_t offset = 0;
    // uint16_t notify_en = 1;
    // esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
    //                                                             gl_profile_tab[PROFILE_A_APP_ID].conn_id,
    //                                                             ESP_GATT_DB_DESCRIPTOR,
    //                                                             gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
    //                                                             gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
    //                                                             p_data->reg_for_notify.handle,
    //                                                             &count);
    // if (ret_status != ESP_GATT_OK)
    // {
    //   ESP_LOGE(TAG_BLE_CONN, "esp_ble_gattc_get_attr_count error, %d", __LINE__);
    //   break;
    // }

    // if (count > 0)
    // {
    //   descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
    //   if (!descr_elem_result)
    //   {
    //     ESP_LOGE(TAG_BLE_CONN, "malloc error, gattc no mem");
    //     break;
    //   }
    //   else
    //   {
    //     ret_status = esp_ble_gattc_get_all_descr(gattc_if,
    //                                              gl_profile_tab[PROFILE_A_APP_ID].conn_id,
    //                                              p_data->reg_for_notify.handle,
    //                                              descr_elem_result,
    //                                              &count,
    //                                              offset);
    //     if (ret_status != ESP_GATT_OK)
    //     {
    //       ESP_LOGE(TAG_BLE_CONN, "esp_ble_gattc_get_all_descr error, %d", __LINE__);
    //       free(descr_elem_result);
    //       descr_elem_result = NULL;
    //       break;
    //     }

    //     for (int i = 0; i < count; ++i)
    //     {
    //       if (descr_elem_result[i].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
    //       {
    //         esp_ble_gattc_write_char_descr(gattc_if,
    //                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
    //                                        descr_elem_result[i].handle,
    //                                        sizeof(notify_en),
    //                                        (uint8_t *)&notify_en,
    //                                        ESP_GATT_WRITE_TYPE_RSP,
    //                                        ESP_GATT_AUTH_REQ_NONE);

    //         break;
    //       }
    //     }
    //   }
    //   free(descr_elem_result);
    //   descr_elem_result = NULL;
    // }

    break;
  }

  case ESP_GATTC_NOTIFY_EVT:
    ESP_LOGI(TAG_BLE_CONN, "Notification received, value ");
    ESP_LOG_BUFFER_HEX(TAG_BLE_CONN, p_data->notify.value, p_data->notify.value_len);
    break;

  case ESP_GATTC_WRITE_DESCR_EVT:
    if (p_data->write.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG_BLE_CONN, "Descriptor write failed, status %x", p_data->write.status);
      break;
    }
    ESP_LOGI(TAG_BLE_CONN, "Descriptor write successfully");
    break;

  case ESP_GATTC_SRVC_CHG_EVT:
    esp_bd_addr_t bda;
    memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG_BLE_CONN, "Service change from " ESP_BD_ADDR_STR "", ESP_BD_ADDR_HEX(bda));
    break;

  case ESP_GATTC_READ_CHAR_EVT:
  {
    if (param->read.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG, "Read failed: %d", param->read.status);
      break;
    }

    ESP_LOGD(TAG_BLE_CONN,
             "Read complete: conn_id=%d handle=0x%04x len=%d",
             param->read.conn_id,
             param->read.handle,
             param->read.value_len);

    if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].feature_char_handle)
    {
      esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &CAM_REMOTE_SERVICE);

      esp_ble_gattc_write_char(
          gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
          gl_profile_tab[PROFILE_A_APP_ID].conn_id,
          gl_profile_tab[PROFILE_A_APP_ID].lock_char_handle, 1, &unlock_value,
          ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NO_MITM);

      esp_ble_gattc_write_char(
          gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
          gl_profile_tab[PROFILE_A_APP_ID].conn_id,
          gl_profile_tab[PROFILE_A_APP_ID].update_char_handle, 1, &unlock_value,
          ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NO_MITM);

      break;
    }
    else
    {
      ESP_LOGW(TAG_BLE_CONN, "Read unexpected char");
      break;
    }

    break;
  }

  case ESP_GATTC_WRITE_CHAR_EVT:
  {
    if (p_data->write.status != ESP_GATT_OK)
    {
      ESP_LOGE(TAG_BLE_CONN, "Characteristic write failed, status %x", p_data->write.status);
      break;
    }

    if (p_data->write.handle == gl_profile_tab[PROFILE_A_APP_ID].location_char_handle)
    {
      ESP_LOGI(TAG_BLE_CONN, "Location char written.");
      break;
    }
    else if (p_data->write.handle == gl_profile_tab[PROFILE_A_APP_ID].lock_char_handle)
    {
      ESP_LOGI(TAG_BLE_CONN, "Lock char written. Fully connected and ready to send location");
      connected = true;
      break;
    }
    else if (p_data->write.handle == gl_profile_tab[PROFILE_A_APP_ID].update_char_handle)
    {
      ESP_LOGI(TAG_BLE_CONN, "Update char written. Fully connected and ready to send location");
      connected = true;
      break;
    }

    break;
  }

  default:
    break;
  }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  switch (event)
  {
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(TAG_BLE_INIT, "Local privacy config failed, status %x", param->local_privacy_cmpl.status);
      break;
    }
    ESP_LOGI(TAG_BLE_INIT, "Local privacy config successfully");
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (scan_ret)
    {
      ESP_LOGE(TAG_BLE_SCAN, "set scan params error, error code = %x", scan_ret);
    }
    break;

  case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
  {
    esp_ble_gap_start_scanning(BLE_SCAN_TIME);
    break;
  }

  case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    // scan start complete event to indicate scan start successfully or failed
    if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(TAG_BLE_SCAN, "Scanning start failed, status %x", param->scan_start_cmpl.status);
      disconnect_and_start_reconnect();
      break;
    }
    ESP_LOGI(TAG_BLE_SCAN, "Scanning start successfully");
    break;

  case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
    /* Call the following function to input the passkey which is displayed on the remote device */
    // esp_ble_passkey_reply(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, true, 0x00);
    ESP_LOGI(TAG_BLE_BOND, "Passkey request");
    break;

  case ESP_GAP_BLE_OOB_REQ_EVT:
    ESP_LOGI(TAG_BLE_BOND, "OOB request");
    uint8_t tk[16] = {1}; // If you paired with OOB, both devices need to use the same tk
    esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
    break;

  case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
    ESP_LOGI(TAG_BLE_BOND, "Local identity root");
    break;

  case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
    ESP_LOGI(TAG_BLE_BOND, "Local encryption root");
    break;

  case ESP_GAP_BLE_SEC_REQ_EVT:
    /* send the positive(true) security response to the peer device to accept the security request.
    If not accept the security request, should send the security response with negative(false) accept value*/
    esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
    break;

  case ESP_GAP_BLE_NC_REQ_EVT:
    /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
    show the passkey number to the user to confirm it with the number displayed by peer device. */
    esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
    ESP_LOGI(TAG_BLE_BOND, "Numeric Comparison request, passkey %" PRIu32, param->ble_security.key_notif.passkey);
    break;

  case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: /// the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
    /// show the passkey number to the user to input it in the peer device.
    ESP_LOGI(TAG_BLE_BOND, "Passkey notify, passkey %" PRIu32, param->ble_security.key_notif.passkey);
    break;

  case ESP_GAP_BLE_KEY_EVT:
    // shows the ble key info share with peer device to the user.
    ESP_LOGI(TAG_BLE_BOND, "Key exchanged, key_type %d", param->ble_security.ble_key.key_type);
    break;

  case ESP_GAP_BLE_AUTH_CMPL_EVT:
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
    ESP_LOGI(TAG_BLE_BOND, "Authentication complete, addr_type %d, addr " ESP_BD_ADDR_STR "",
             param->ble_security.auth_cmpl.addr_type, ESP_BD_ADDR_HEX(bd_addr));
    if (!param->ble_security.auth_cmpl.success)
    {
      ESP_LOGI(TAG_BLE_BOND, "Pairing failed, reason 0x%x", param->ble_security.auth_cmpl.fail_reason);
    }
    else
    {
      ESP_LOGI(TAG_BLE_BOND, "Pairing successfully, auth mode %d", param->ble_security.auth_cmpl.auth_mode);
    }
    break;

  case ESP_GAP_BLE_SCAN_RESULT_EVT:
  {
    esp_ble_gap_cb_param_t *scan = param;
    if (scan->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT)
    {
      ESP_LOGD(TAG_BLE_SCAN, "Scan result, device " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(scan->scan_rst.bda));

      // Manufacturer data
      uint8_t manu_len = 0;
      uint8_t *manu = esp_ble_resolve_adv_data(
          scan->scan_rst.ble_adv,
          ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE,
          &manu_len);

      if (!manu || manu_len < 4)
        break; // no valid manufacturer data

      uint16_t company_id = manu[0] | (manu[1] << 8);
      uint16_t product_id = manu[2] | (manu[3] << 8);

      if (company_id != SONY_COMPANY_ID || product_id != SONY_CAMERA_PRODUCT)
      {
        ESP_LOGD(TAG_BLE_SCAN, "Skipping non-Sony camera product");
        break; // not a Sony camera
      }

      // Optional device name filter
      if (strlen(BLE_CAM_NAME) > 0)
      {
        uint8_t adv_name_len = 0;
        uint8_t *adv_name = esp_ble_resolve_adv_data(
            scan->scan_rst.ble_adv,
            ESP_BLE_AD_TYPE_NAME_CMPL,
            &adv_name_len);

        if (adv_name == NULL)
        {
          // Name check is active but remote device didn't send any, skip
          break;
        }
        if (strlen(BLE_CAM_NAME) != adv_name_len ||
            strncmp((char *)adv_name, BLE_CAM_NAME, adv_name_len) != 0)
        {
          // Name is non-empty but doesn't match, skip
          break;
        }
        ESP_LOGI(TAG_BLE_SCAN, "Device name matches: %s", BLE_CAM_NAME);
      }

      // Optional MAC prefix filter
      if (strlen(BLE_CAM_MAC_PREFIX_DYNAMIC) > 0)
      {
        char addr_str[18];
        snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 scan->scan_rst.bda[0], scan->scan_rst.bda[1], scan->scan_rst.bda[2],
                 scan->scan_rst.bda[3], scan->scan_rst.bda[4], scan->scan_rst.bda[5]);

        if (strncmp(addr_str, BLE_CAM_MAC_PREFIX_DYNAMIC, strlen(BLE_CAM_MAC_PREFIX_DYNAMIC)) != 0)
        {
          break; // skip if prefix doesn't match
        }
        ESP_LOGI(TAG_BLE_SCAN, "Device name matches: %s", BLE_CAM_NAME);
      }

      // Found a matching Sony camera
      ESP_LOGI(TAG_BLE_SCAN, "Sony Camera found! Address: %02X:%02X:%02X:%02X:%02X:%02X",
               scan->scan_rst.bda[0], scan->scan_rst.bda[1], scan->scan_rst.bda[2],
               scan->scan_rst.bda[3], scan->scan_rst.bda[4], scan->scan_rst.bda[5]);

      if (!connect)
      {
        connect = true;
        esp_ble_gap_stop_scanning();

        ESP_LOGI(TAG_BLE_CONN, "Connecting to the camera...");
        esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
        memcpy(&creat_conn_params.remote_bda, scan->scan_rst.bda, ESP_BD_ADDR_LEN);
        creat_conn_params.remote_addr_type = scan->scan_rst.ble_addr_type;
        creat_conn_params.own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC;
        creat_conn_params.is_direct = true;
        creat_conn_params.is_aux = false;
        creat_conn_params.phy_mask = 0x0;
        esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                               &creat_conn_params);
      }
    }
    else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT)
    {
      ESP_LOGI(TAG_BLE_SCAN, "Scan completed");
      disconnect_and_start_reconnect();
    }
    break;
  }

  case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
    {
      ESP_LOGE(TAG_BLE_SCAN, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
    }
    else
    {
      ESP_LOGI(TAG_BLE_SCAN, "Scanning stop successfully");
    }
    if (!connect)
    {
      esp_ble_gap_start_scanning(BLE_SCAN_TIME);
    }
    break;

  default:
    break;
  }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  ESP_LOGI(TAG_BLE_INIT, "EVT %d, gattc if %d", event, gattc_if);

  /* If event is register event, store the gattc_if for each profile */
  if (event == ESP_GATTC_REG_EVT)
  {
    if (param->reg.status == ESP_GATT_OK)
    {
      gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
    }
    else
    {
      ESP_LOGI(TAG_BLE_INIT, "Reg app failed, app_id %04x, status %d",
               param->reg.app_id,
               param->reg.status);
      return;
    }
  }

  /* If the gattc_if equal to profile A, call profile A cb handler,
   * so here call each profile's callback */
  do
  {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++)
    {
      if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
          gattc_if == gl_profile_tab[idx].gattc_if)
      {
        if (gl_profile_tab[idx].gattc_cb)
        {
          gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
        }
      }
    }
  } while (0);
}

static void location_sender_task(void *arg)
{
  uint8_t payload[95];

  while (1)
  {
    if (connected && gl_profile_tab[PROFILE_A_APP_ID].gattc_if != ESP_GATT_IF_NONE && gl_profile_tab[PROFILE_A_APP_ID].location_char_handle != INVALID_HANDLE)
    {
      // TODO: Get actual GPS data :)
      size_t len = buildLocationPayload(
          payload,
          473000000, // latitude * 1e7
          854000000, // longitude * 1e7
          2026, 1, 9, 12, 0, 0,
          60, 60);

      ESP_LOGI(TAG, "Sending location update");

      esp_err_t err = esp_ble_gattc_write_char(
          gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
          gl_profile_tab[PROFILE_A_APP_ID].conn_id,
          gl_profile_tab[PROFILE_A_APP_ID].location_char_handle,
          len,
          payload,
          ESP_GATT_WRITE_TYPE_RSP,
          ESP_GATT_AUTH_REQ_NONE);

      if (err != ESP_OK)
      {
        ESP_LOGE(TAG, "Location write failed: %s", esp_err_to_name(err));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(BLE_LOCATION_RETRANSMIT));
  }
}

void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_INFO);
  ESP_LOGI(TAG, "Starting AlphaLoc");
  esp_err_t ret;

  // Initialize NVS.
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // BLE Setup
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT)); // We need BLE only

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  // register the  callback function to the gap module
  ret = esp_ble_gap_register_callback(esp_gap_cb);
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s gap register error, error code = %x", __func__, ret);
    return;
  }

  // register the callback function to the gattc module
  ret = esp_ble_gattc_register_callback(esp_gattc_cb);
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s gattc register error, error code = %x", __func__, ret);
    return;
  }

  ret = esp_ble_gattc_app_register(0);
  if (ret)
  {
    ESP_LOGE(TAG_BLE_INIT, "%s gattc app register error, error code = %x", __func__, ret);
  }

  esp_ble_gap_set_device_name(BLE_NAME);

  /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
  /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
  and the response key means which key you can distribute to the Master;
  If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
  and the init key means which key you can distribute to the slave. */
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

  xTaskCreate(location_sender_task, "location_sender", 4096, NULL, 5, NULL);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
