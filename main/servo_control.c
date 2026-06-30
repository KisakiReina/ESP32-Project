/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "servo_control.h"

static const char *TAG = "SERVO";

/* SG90 servo specs: 50Hz (20ms period), 0.5~2.5ms pulse => 0~180 degrees */
#define SERVO_FREQ              50      /* 50 Hz (20ms period) */
#define SERVO_TIMER             LEDC_TIMER_0
#define SERVO_MODE              LEDC_LOW_SPEED_MODE
#define SERVO_DUTY_RES          LEDC_TIMER_13_BIT  /* 13-bit: 0~8191 */
#define SERVO_CHANNEL           LEDC_CHANNEL_0

/* Pulse width in microseconds for 0° and 180° */
#define SERVO_PULSE_MIN_US      500     /* 0.5ms => 0 degrees */
#define SERVO_PULSE_MAX_US      2500    /* 2.5ms => 180 degrees */

/* Convert pulse width (us) to LEDC duty at 50Hz with 13-bit resolution */
/* Duty = pulse_us * (2^13) / (1000000 / 50) = pulse_us * 8192 / 20000 */
#define PULSE_TO_DUTY(us)       ((uint32_t)(((uint64_t)(us) * 8192) / 20000))

static int s_servo_gpio = -1;

esp_err_t servo_init(int gpio_pin)
{
    s_servo_gpio = gpio_pin;

    /* Configure LEDC timer */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_DUTY_RES,
        .freq_hz         = SERVO_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* Configure LEDC channel */
    ledc_channel_config_t chan_cfg = {
        .gpio_num       = gpio_pin,
        .speed_mode     = SERVO_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = SERVO_TIMER,
        .duty           = PULSE_TO_DUTY(SERVO_PULSE_MIN_US), /* start at 0° */
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));

    ESP_LOGI(TAG, "Servo initialized on GPIO %d", gpio_pin);
    return ESP_OK;
}

esp_err_t servo_set_angle(uint16_t angle, uint32_t wait_ms)
{
    if (s_servo_gpio < 0) {
        ESP_LOGE(TAG, "Servo not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (angle > 180) {
        angle = 180;
    }

    /* Map angle [0, 180] to pulse width [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US] */
    uint32_t pulse_us = SERVO_PULSE_MIN_US +
                        (uint32_t)((uint64_t)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * angle / 180);
    uint32_t duty = PULSE_TO_DUTY(pulse_us);

    ESP_LOGD(TAG, "Set angle %u° => pulse %lu us => duty %lu", angle, pulse_us, duty);

    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);

    if (wait_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }

    return ESP_OK;
}

esp_err_t servo_deinit(void)
{
    if (s_servo_gpio < 0) {
        return ESP_OK;
    }

    /* Stop PWM output */
    ledc_stop(SERVO_MODE, SERVO_CHANNEL, 0);
    s_servo_gpio = -1;
    ESP_LOGI(TAG, "Servo de-initialized");
    return ESP_OK;
}
