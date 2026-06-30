/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef _ESP_HID_GAP_H_
#define _ESP_HID_GAP_H_

#define HIDD_IDLE_MODE 0x00
#define HIDD_BLE_MODE 0x01
#define HIDD_BT_MODE 0x02
#define HIDD_BTDM_MODE 0x03

#if CONFIG_BT_HID_DEVICE_ENABLED
#if CONFIG_BT_BLE_ENABLED
#define HID_DEV_MODE HIDD_BTDM_MODE
#else
#define HID_DEV_MODE HIDD_BT_MODE
#endif
#elif CONFIG_BT_BLE_ENABLED
#define HID_DEV_MODE HIDD_BLE_MODE
#elif CONFIG_BT_NIMBLE_ENABLED
#define HID_DEV_MODE HIDD_BLE_MODE
#else
#define HID_DEV_MODE HIDD_IDLE_MODE
#endif

#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#if !CONFIG_BT_NIMBLE_ENABLED
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#endif
#include "esp_hid_common.h"
#if CONFIG_BT_BLE_ENABLED
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#endif

/**
 * @brief External BLE GAP event callback
 *        Called when specific GAP events occur (e.g. RSSI read complete)
 */
typedef void (*esp_hid_gap_ble_ext_cb_t)(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/**
 * @brief Register an external BLE GAP event callback.
 *        Used to receive forwarded GAP events like ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT.
 * @param cb  Callback function, or NULL to unregister.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_hid_gap_ble_register_ext_cb(esp_hid_gap_ble_ext_cb_t cb);

#ifdef __cplusplus
extern "C" {
#endif

#if !CONFIG_BT_NIMBLE_ENABLED
typedef struct esp_hidh_scan_result_s {
    struct esp_hidh_scan_result_s *next;

    esp_bd_addr_t bda;
    const char *name;
    int8_t rssi;
    esp_hid_usage_t usage;
    esp_hid_transport_t transport; //BT, BLE or USB
    union {
        struct {
            esp_bt_cod_t cod;
            esp_bt_uuid_t uuid;
        } bt;
        struct {
            esp_ble_addr_type_t addr_type;
            uint16_t appearance;
        } ble;
    };
} esp_hid_scan_result_t;

esp_err_t esp_hid_scan(uint32_t seconds, size_t *num_results, esp_hid_scan_result_t **results);
void esp_hid_scan_results_free(esp_hid_scan_result_t *results);
const char *ble_addr_type_str(esp_ble_addr_type_t ble_addr_type);
void print_uuid(esp_bt_uuid_t *uuid);
#endif

esp_err_t esp_hid_gap_init(uint8_t mode);
esp_err_t esp_hid_gap_deinit(void);

esp_err_t esp_hid_ble_gap_adv_init(uint16_t appearance, const char *device_name);
esp_err_t esp_hid_ble_gap_adv_start(void);
esp_err_t esp_hid_ble_gap_adv_stop(void);

/**
 * @brief Start directed advertising to a specific peer device.
 *        Only the target device can connect.
 * @param target_bda  Peer BDA to direct advertising to.
 * @param peer_addr_type  Peer address type (BLE_ADDR_TYPE_PUBLIC or BLE_ADDR_TYPE_RANDOM).
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_hid_ble_gap_adv_start_directed(esp_bd_addr_t target_bda, esp_ble_addr_type_t peer_addr_type);

/**
 * @brief Start BLE scanning for nearby devices.
 * @param seconds  Scan duration in seconds (0 = continuous).
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_hid_ble_gap_scan_start(uint32_t seconds);

/**
 * @brief Stop BLE scanning.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_hid_ble_gap_scan_stop(void);

/** Raw BLE scan result callback: called for EVERY scan result (no UUID filter) */
typedef void (*esp_hid_ble_scan_raw_cb_t)(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, int8_t rssi);

/**
 * @brief Register a raw scan result callback.
 *        Called for each BLE scan result regardless of UUID/service data.
 * @param cb  Callback function
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_hid_ble_gap_register_scan_raw_cb(esp_hid_ble_scan_raw_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* _ESP_HIDH_GAP_H_ */
