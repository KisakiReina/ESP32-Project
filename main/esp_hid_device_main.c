/*
 * Proximity Unlock: BLE Peripheral — scan-based unlock, no connections after pairing.
 * Unlock: bonded device + not in scan cache + RSSI > -40 → servo open (5s), then relock.
 * Indoor detection: devices seen during 10s observation window (and in cache) are "indoor", ignored.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_hidd.h"
#include "esp_hidd_gatts.h"
#include "esp_hid_gap.h"
#include "servo_control.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"

/* Set servo GPIO pin directly here */
#define SERVO_GPIO_PIN              2
#define BUTTON_GPIO                 4
#define BUTTON_ACTIVE_LEVEL         0
#define BUTTON_DEBOUNCE_MS          50
#define PAIRING_MODE_TIMEOUT_S      60
#define UNLOCK_HOLD_TIME_US         5000000
#define SERVO_ANGLE_UNLOCK          30
#define SERVO_ANGLE_LOCK            180
#define SERVO_MOVE_TIME_MS          800
#define BLE_DEVICE_NAME             "Proximity Unlock"
#define RSSI_THRESHOLD_NEAR         -50
#define LED_GPIO                    8
#define LED_BLINK_INTERVAL_US       200000  /* 200ms */
#define SCAN_CACHE_SIZE             8
#define SCAN_CACHE_TTL_US           30000000
#define SCAN_GRACE_PERIOD_US        10000000

static const char *TAG = "PROX_UNLOCK";

/* ========== State ========== */
static esp_hidd_dev_t *s_hid_dev = NULL;

typedef enum { PROX_STATE_LOCKED, PROX_STATE_UNLOCKING, PROX_STATE_UNLOCKED, PROX_STATE_LOCKING } proximity_state_t;
static proximity_state_t s_prox_state = PROX_STATE_LOCKED;
static bool s_pairing_mode_allowed = false;

/* Scan cache */
static struct { esp_bd_addr_t bda; int64_t last_seen_us; } s_scan_cache[SCAN_CACHE_SIZE];
static bool s_is_scanning_mode = false;
static int64_t s_scan_mode_enter_us = 0;

static esp_timer_handle_t s_hold_timer = NULL;
static esp_timer_handle_t s_led_blink_timer = NULL;
static rmt_channel_handle_t s_rmt_chan = NULL;

static void do_lock(void);
static void do_unlock(void);
static bool is_bonded_device(esp_bd_addr_t bda);

/* ========== Hold Timer ========== */
static void hold_timer_cb(void *arg)
{
    ESP_LOGI(TAG, "Unlock hold time expired — locking");
    do_lock();
}

/* ========== WS2812 RGB LED via RMT ========== */
/* RMT resolution: 10MHz = 100ns/tick */
#define RMT_RESOLUTION_HZ       10000000
/* WS2812 bit timings (ticks at 10MHz):
   Bit 0: high 4 ticks (0.4µs), low 9 ticks (0.9µs)
   Bit 1: high 8 ticks (0.8µs), low 5 ticks (0.5µs) */
#define WS2812_T0H_TICKS        4
#define WS2812_T0L_TICKS        9
#define WS2812_T1H_TICKS        8
#define WS2812_T1L_TICKS        5

static rmt_encoder_handle_t s_bytes_encoder = NULL;

static void ws2812_set_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    /* WS2812 color order: GRB */
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    rmt_transmit(s_rmt_chan, s_bytes_encoder, grb, sizeof(grb), &tx_cfg);
    /* Need to wait for transmit to complete before next refresh */
    rmt_tx_wait_all_done(s_rmt_chan, portMAX_DELAY);
}

static void ws2812_clear(void)
{
    ws2812_set_pixel(0, 0, 0);
}

static uint8_t s_blink_r = 0, s_blink_g = 0, s_blink_b = 0;

static void led_blink_timer_cb(void *arg)
{
    static bool on = false;
    on = !on;
    if (on) {
        ws2812_set_pixel(s_blink_r, s_blink_g, s_blink_b);
    } else {
        ws2812_clear();
    }
}

static void led_blink_start(uint8_t r, uint8_t g, uint8_t b)
{
    s_blink_r = r; s_blink_g = g; s_blink_b = b;
    ws2812_set_pixel(r, g, b);
    esp_timer_start_periodic(s_led_blink_timer, LED_BLINK_INTERVAL_US);
}

static void led_blink_stop(void)
{
    esp_timer_stop(s_led_blink_timer);
    ws2812_clear();
}

/* ========== Scan Cache Helpers ========== */
static int scan_cache_find(esp_bd_addr_t bda)
{
    for (int i = 0; i < SCAN_CACHE_SIZE; i++)
        if (s_scan_cache[i].last_seen_us && !memcmp(s_scan_cache[i].bda, bda, ESP_BD_ADDR_LEN))
            return i;
    return -1;
}

static bool scan_cache_is_known(esp_bd_addr_t bda)
{
    int idx = scan_cache_find(bda);
    return (idx >= 0) && ((esp_timer_get_time() - s_scan_cache[idx].last_seen_us) < SCAN_CACHE_TTL_US);
}

static void scan_cache_seen(esp_bd_addr_t bda)
{
    int64_t now = esp_timer_get_time();
    int idx = scan_cache_find(bda);
    if (idx >= 0) { s_scan_cache[idx].last_seen_us = now; return; }
    for (int i = 0; i < SCAN_CACHE_SIZE; i++) {
        if (!s_scan_cache[i].last_seen_us) {
            memcpy(s_scan_cache[i].bda, bda, ESP_BD_ADDR_LEN);
            s_scan_cache[i].last_seen_us = now; return;
        }
    }
    int oldest = 0;
    for (int i = 1; i < SCAN_CACHE_SIZE; i++)
        if (s_scan_cache[i].last_seen_us < s_scan_cache[oldest].last_seen_us) oldest = i;
    memcpy(s_scan_cache[oldest].bda, bda, ESP_BD_ADDR_LEN);
    s_scan_cache[oldest].last_seen_us = now;
}

/* ========== Core: Scan Callback → Unlock Decision ========== */
static void scan_raw_cb(esp_bd_addr_t bda, esp_ble_addr_type_t addr_type, int8_t rssi)
{
    if (!s_is_scanning_mode) return;

    if (!is_bonded_device(bda)) {
        /* Log non-bonded devices at DEBUG level for diagnostics */
        ESP_LOGD(TAG, "Scan non-bonded: " ESP_BD_ADDR_STR " type=%d RSSI=%d",
                 ESP_BD_ADDR_HEX(bda), addr_type, rssi);
        return;
    }

    ESP_LOGD(TAG, "Scan bonded: " ESP_BD_ADDR_STR " type=%d RSSI=%d",
             ESP_BD_ADDR_HEX(bda), addr_type, rssi);

    if (scan_cache_is_known(bda)) { scan_cache_seen(bda); return; }

    if ((esp_timer_get_time() - s_scan_mode_enter_us) < SCAN_GRACE_PERIOD_US) {
        scan_cache_seen(bda);
        ESP_LOGD(TAG, "Grace: " ESP_BD_ADDR_STR " → cache (indoor)", ESP_BD_ADDR_HEX(bda));
        return;
    }

    if (rssi > RSSI_THRESHOLD_NEAR && s_prox_state == PROX_STATE_LOCKED) {
        ESP_LOGI(TAG, "NEW arrival " ESP_BD_ADDR_STR " RSSI=%d → UNLOCK", ESP_BD_ADDR_HEX(bda), rssi);
        scan_cache_seen(bda);
        do_unlock();
    } else if (s_prox_state == PROX_STATE_LOCKED) {
        ESP_LOGD(TAG, "RSSI %d <= threshold %d, waiting for closer signal", rssi, RSSI_THRESHOLD_NEAR);
    }
}

/* ========== Enter Scan Mode ========== */
static void enter_scan_mode(void)
{
    if (s_is_scanning_mode) return;
    s_is_scanning_mode = true;
    s_scan_mode_enter_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Scan mode — %ds observation window", (int)(SCAN_GRACE_PERIOD_US/1000000));
    /* Stop advertising so bonded phones won't auto-reconnect */
    esp_hid_ble_gap_adv_stop();
    vTaskDelay(pdMS_TO_TICKS(50));
    /* Stop any ongoing scan first, then start fresh */
    esp_hid_ble_gap_scan_stop();
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_hid_ble_gap_scan_start(0);
}

/* ========== Servo Helpers ========== */
static void do_unlock(void)
{
    ESP_LOGI(TAG, "Unlocking...");
    s_prox_state = PROX_STATE_UNLOCKING;
    led_blink_start(255, 0, 0);  /* red blink */
    servo_set_angle(SERVO_ANGLE_UNLOCK, SERVO_MOVE_TIME_MS);
    s_prox_state = PROX_STATE_UNLOCKED;
    ESP_LOGI(TAG, "Unlocked — relock in %ds", (int)(UNLOCK_HOLD_TIME_US/1000000));
    if (s_hold_timer) { esp_timer_stop(s_hold_timer); esp_timer_start_once(s_hold_timer, UNLOCK_HOLD_TIME_US); }
}

static void do_lock(void)
{
    if (s_prox_state == PROX_STATE_LOCKED) return;
    ESP_LOGI(TAG, "Locking...");
    s_prox_state = PROX_STATE_LOCKING;
    led_blink_stop();
    servo_set_angle(SERVO_ANGLE_LOCK, SERVO_MOVE_TIME_MS);
    s_prox_state = PROX_STATE_LOCKED;
    ESP_LOGI(TAG, "Locked");
    if (s_hold_timer) esp_timer_stop(s_hold_timer);
}

/* ========== is_bonded_device ========== */
static bool is_bonded_device(esp_bd_addr_t bda)
{
    int n = esp_ble_get_bond_device_num();
    if (n <= 0) return false;
    esp_ble_bond_dev_t *list = malloc(n * sizeof(esp_ble_bond_dev_t));
    if (!list) return false;
    memset(list, 0, n * sizeof(esp_ble_bond_dev_t));
    if (esp_ble_get_bond_device_list(&n, list) != ESP_OK) { free(list); return false; }
    for (int i = 0; i < n; i++)
        if (!memcmp(list[i].bd_addr, bda, ESP_BD_ADDR_LEN)) { free(list); return true; }
    free(list);
    return false;
}

/* ========== Button & Pairing ========== */
static void button_init(void)
{
    gpio_config_t c = { .pin_bit_mask = 1ULL<<BUTTON_GPIO, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&c);
    ESP_LOGI(TAG, "Button on GPIO%d", BUTTON_GPIO);
}

static void button_task(void *arg)
{
    bool prev = true;
    int64_t ps = 0;
    while (1) {
        bool cur = gpio_get_level(BUTTON_GPIO);
        if (prev && !cur) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(BUTTON_GPIO) == BUTTON_ACTIVE_LEVEL) {
                while (gpio_get_level(BUTTON_GPIO) == BUTTON_ACTIVE_LEVEL) vTaskDelay(pdMS_TO_TICKS(20));
                vTaskDelay(pdMS_TO_TICKS(20));
                s_pairing_mode_allowed = !s_pairing_mode_allowed;
                if (s_pairing_mode_allowed) {
                    ps = esp_timer_get_time();
                    ESP_LOGI(TAG, "*** PAIRING ON (%ds) ***", PAIRING_MODE_TIMEOUT_S);
                    led_blink_start(0, 0, 255);  /* blue blink */
                    s_is_scanning_mode = false;
                    esp_hid_ble_gap_scan_stop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_hid_ble_gap_adv_start();
                } else {
                    ps = 0;
                    ESP_LOGI(TAG, "*** PAIRING OFF ***");
                    led_blink_stop();
                    enter_scan_mode();
                }
            }
        }
        prev = gpio_get_level(BUTTON_GPIO);
        if (s_pairing_mode_allowed && ps && (esp_timer_get_time()-ps)/1000000 >= PAIRING_MODE_TIMEOUT_S) {
            s_pairing_mode_allowed = false; ps = 0;
            ESP_LOGI(TAG, "*** Pairing timeout ***");
            led_blink_stop();
            enter_scan_mode();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ========== GATTS (minimal forward) ========== */
static void gatts_event_handler(esp_gatts_cb_event_t e, esp_gatt_if_t g, esp_ble_gatts_cb_param_t *p)
{ esp_hidd_gatts_event_handler(e, g, p); }

/* ========== HIDD Events ========== */
static void ble_hidd_event_callback(void *ha, esp_event_base_t b, int32_t id, void *ed)
{
    (void)ha; (void)b; (void)ed;
    switch ((esp_hidd_event_t)id) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "BLE HID started");
        if (esp_ble_get_bond_device_num() <= 0) {
            ESP_LOGI(TAG, "No bonded devices → advertising for initial pair");
            esp_hid_ble_gap_adv_start();
        } else enter_scan_mode();
        break;
    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "Device connected (pairing)");
        s_is_scanning_mode = false;
        break;
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "Device disconnected");
        do_lock();
        if (s_pairing_mode_allowed) esp_hid_ble_gap_adv_start();
        else enter_scan_mode();
        break;
    default: break;
    }
}

/* ========== HID Report Map ========== */
static const unsigned char hid_report_map[] = {
    0x06,0x00,0xFF, 0x09,0x01, 0xA1,0x01, 0x85,0x01,
    0x15,0x00, 0x26,0xFF,0x00, 0x75,0x08, 0x95,0x01,
    0x09,0x01, 0x81,0x02, 0xC0
};
static esp_hid_raw_report_map_t ble_maps[] = {{ .data = hid_report_map, .len = sizeof(hid_report_map) }};
static esp_hid_device_config_t ble_cfg = {
    .vendor_id = 0x16C0, .product_id = 0x05DF, .version = 0x0100,
    .device_name = BLE_DEVICE_NAME, .manufacturer_name = "Espressif",
    .serial_number = "1234567890", .report_maps = ble_maps, .report_maps_len = 1
};

/* ========== app_main ========== */
void app_main(void)
{
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); r = nvs_flash_init(); }
    ESP_ERROR_CHECK(r);

    ESP_LOGI(TAG, "Proximity Unlock Starting...");
    ESP_ERROR_CHECK(esp_hid_gap_init(HIDD_BLE_MODE));
    ESP_ERROR_CHECK(esp_ble_gap_config_local_privacy(true));
    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, BLE_DEVICE_NAME));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_cfg, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &s_hid_dev));

    esp_timer_create_args_t ta = { .callback = hold_timer_cb, .name = "hold", .skip_unhandled_events = true };
    ESP_ERROR_CHECK(esp_timer_create(&ta, &s_hold_timer));

    esp_timer_create_args_t lta = { .callback = led_blink_timer_cb, .name = "led_blink", .skip_unhandled_events = true };
    ESP_ERROR_CHECK(esp_timer_create(&lta, &s_led_blink_timer));

    rmt_tx_channel_config_t rmt_chan_cfg = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_chan_cfg, &s_rmt_chan));

    rmt_bytes_encoder_config_t bytes_encoder_cfg = {
        .bit0 = {
            .duration0 = WS2812_T0H_TICKS,
            .level0 = 1,
            .duration1 = WS2812_T0L_TICKS,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = WS2812_T1H_TICKS,
            .level0 = 1,
            .duration1 = WS2812_T1L_TICKS,
            .level1 = 0,
        },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_cfg, &s_bytes_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));

    esp_hid_ble_gap_register_scan_raw_cb(scan_raw_cb);

    ESP_LOGI(TAG, "Servo on GPIO%d", SERVO_GPIO_PIN);
    if (servo_init(SERVO_GPIO_PIN) == ESP_OK) { servo_set_angle(SERVO_ANGLE_LOCK, 500); ESP_LOGI(TAG, "Servo OK"); }
    else ESP_LOGE(TAG, "Servo init failed");

    button_init();
    xTaskCreate(button_task, "btn", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Ready. Button GPIO%d (%ds). RSSI>%d → unlock, %ds hold.",
             BUTTON_GPIO, PAIRING_MODE_TIMEOUT_S, RSSI_THRESHOLD_NEAR, (int)(UNLOCK_HOLD_TIME_US/1000000));
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}
