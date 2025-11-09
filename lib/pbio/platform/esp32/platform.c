// SPDX-License-Identifier: MIT
// Copyright (c) 2025 The Pybricks Authors

// Minimal ESP32 platform initialization

#include <string.h>

#include <pbdrv/config.h>
#include <pbdrv/gpio.h>
#include <pbio/button.h>

#include "pbdrvconfig.h"

// TODO: Include ESP-IDF headers when we integrate with build system
// #include "esp_system.h"
// #include "driver/gpio.h"

// Button configuration (can use boot button on most ESP32 dev boards)
#if PBDRV_CONFIG_BUTTON_GPIO
#include "../../drv/button/button_gpio.h"

const pbdrv_button_gpio_platform_t pbdrv_button_gpio_platform[PBDRV_CONFIG_BUTTON_GPIO_NUM_BUTTON] = {
    [0] = {
        .gpio = { .bank = NULL, .pin = 0 },  // GPIO0 is boot button on most boards
        .pull = PBDRV_GPIO_PULL_UP,
        .button = PBIO_BUTTON_CENTER,
        .active_low = true,
    }
};
#endif

// LED configuration (can use built-in LED)
#if PBDRV_CONFIG_LED_PWM
#include "../../drv/led/led_pwm.h"

// Placeholder LED - will need proper PWM configuration
const pbdrv_led_pwm_platform_data_t pbdrv_led_pwm_platform_data[PBDRV_CONFIG_LED_PWM_NUM_DEV] = {
    {
        .id = 0,
        .r_id = 0,  // Will map to GPIO pin via PWM driver
        .g_id = 0,
        .b_id = 0,
        .scale_factor = 1,
    }
};
#endif

// Bluetooth - ESP32 uses NimBLE
#if PBDRV_CONFIG_BLUETOOTH_NIMBLE
// Platform data will be configured through ESP-IDF
// NimBLE is initialized through ESP-IDF's stack
#endif

// USB configuration
#if PBDRV_CONFIG_USB
// ESP32-S2/S3/C3 have native USB support via TinyUSB
// Configuration is handled through ESP-IDF
#endif

/**
 * Initializes the platform-specific hardware.
 *
 * For ESP32, most initialization is handled by ESP-IDF before this is called.
 * This function handles any additional Pybricks-specific setup.
 */
void pbdrv_platform_init(void) {
    // ESP-IDF has already initialized:
    // - System clocks
    // - FreeRTOS
    // - Flash/PSRAM
    // - Basic peripherals

    // TODO: Add any ESP32-specific initialization here
    // For now, minimal setup since drivers will initialize themselves
}

// Periodic platform update (called from main loop if needed)
void pbdrv_platform_update(void) {
    // Nothing to do here for now
}
