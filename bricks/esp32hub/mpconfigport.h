// SPDX-License-Identifier: MIT
// Copyright (c) 2025 The Pybricks Authors

// MicroPython configuration for ESP32-based Pybricks hub

#define MICROPY_HW_BOARD_NAME                   "Pybricks ESP32 Hub"
#define MICROPY_HW_MCU_NAME                     "ESP32-S3"  // Can be changed for other variants

#define PYBRICKS_HUB_NAME                       "esp32hub"
#define PYBRICKS_HUB_CLASS_NAME                 (MP_QSTR_ESP32Hub)

#define PYBRICKS_HUB_ESP32HUB                   (1)

// Pybricks modules - minimal configuration for initial testing
#define PYBRICKS_PY_COMMON                      (1)
#define PYBRICKS_PY_COMMON_BLE                  (1)
#define PYBRICKS_PY_COMMON_CHARGER              (0)  // No charger initially
#define PYBRICKS_PY_COMMON_COLOR_LIGHT          (0)  // No color light initially
#define PYBRICKS_PY_COMMON_CONTROL              (1)
#define PYBRICKS_PY_COMMON_IMU                  (0)  // Can add later with MPU6050/LSM6DS3
#define PYBRICKS_PY_COMMON_KEYPAD               (1)
#define PYBRICKS_PY_COMMON_KEYPAD_HUB_BUTTONS   (1)  // Just one button initially
#define PYBRICKS_PY_COMMON_LIGHT_ARRAY          (0)
#define PYBRICKS_PY_COMMON_LIGHT_MATRIX         (0)
#define PYBRICKS_PY_COMMON_LOGGER               (0)  // Can enable later
#define PYBRICKS_PY_COMMON_MOTOR_MODEL          (0)  // No motors initially
#define PYBRICKS_PY_COMMON_MOTORS               (0)  // No motors initially
#define PYBRICKS_PY_COMMON_SPEAKER              (0)  // Can add later with DAC
#define PYBRICKS_PY_COMMON_SYSTEM               (1)
#define PYBRICKS_PY_EV3DEVICES                  (0)
#define PYBRICKS_PY_EXPERIMENTAL                (1)
#define PYBRICKS_PY_HUBS                        (1)
#define PYBRICKS_PY_IODEVICES                   (0)  // No IO devices initially
#define PYBRICKS_PY_NXTDEVICES                  (0)
#define PYBRICKS_PY_PARAMETERS                  (1)
#define PYBRICKS_PY_PARAMETERS_BUTTON           (1)
#define PYBRICKS_PY_PARAMETERS_ICON             (0)
#define PYBRICKS_PY_DEVICES                     (0)  // No devices initially
#define PYBRICKS_PY_PUPDEVICES                  (0)  // No PUP devices initially
#define PYBRICKS_PY_ROBOTICS                    (0)  // No robotics initially
#define PYBRICKS_PY_TOOLS                       (1)

// Pybricks options
#define PYBRICKS_OPT_COMPILER                   (1)
#define PYBRICKS_OPT_TERSE_ERR                  (0)
#define PYBRICKS_OPT_EXTRA_MOD                  (1)
#define PYBRICKS_OPT_CUSTOM_IMPORT              (1)

// Include ESP32 MicroPython port configuration
// This provides base ESP32 features (WiFi, BLE, etc.)
#include "../../micropython/ports/esp32/mpconfigport.h"

// Override some ESP32 defaults for Pybricks
#undef MICROPY_HW_BOARD_NAME
#define MICROPY_HW_BOARD_NAME "Pybricks ESP32 Hub"

// Ensure BLE is enabled for Pybricks communication
#ifndef MICROPY_PY_BLUETOOTH
#define MICROPY_PY_BLUETOOTH (1)
#endif

// Include common Pybricks configuration
#include "../_common/mpconfigport.h"
