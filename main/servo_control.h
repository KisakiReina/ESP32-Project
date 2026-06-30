/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize servo motor with LEDC PWM
 *        Default: 50Hz PWM, 0.5ms~2.5ms pulse (0~180 degrees)
 *
 * @param gpio_pin  GPIO pin connected to servo signal wire
 * @return esp_err_t ESP_OK on success
 */
esp_err_t servo_init(int gpio_pin);

/**
 * @brief Set servo angle
 *
 * @param angle  Target angle in degrees (0~180)
 * @param wait_ms  Wait time for servo to reach position (ms)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t servo_set_angle(uint16_t angle, uint32_t wait_ms);

/**
 * @brief De-initialize servo and free LEDC resources
 * @return esp_err_t ESP_OK on success
 */
esp_err_t servo_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* _SERVO_CONTROL_H_ */
