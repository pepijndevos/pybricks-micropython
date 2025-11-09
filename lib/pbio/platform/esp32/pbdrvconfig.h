// SPDX-License-Identifier: MIT
// Copyright (c) 2025 The Pybricks Authors

// platform-specific configuration for ESP32

// Basic system drivers
#define PBDRV_CONFIG_BATTERY                        (1)
#define PBDRV_CONFIG_BATTERY_TEST                   (1)

#define PBDRV_CONFIG_BLOCK_DEVICE                   (1)
#define PBDRV_CONFIG_BLOCK_DEVICE_RAM_SIZE          (50 * 1024)
#define PBDRV_CONFIG_BLOCK_DEVICE_TEST              (1)

#define PBDRV_CONFIG_BLUETOOTH                      (1)
#define PBDRV_CONFIG_BLUETOOTH_NIMBLE_ESP32         (1)

#define PBDRV_CONFIG_BUTTON                         (1)
#define PBDRV_CONFIG_BUTTON_GPIO                    (1)
#define PBDRV_CONFIG_BUTTON_GPIO_NUM_BUTTON         (1)

#define PBDRV_CONFIG_CLOCK                          (1)
#define PBDRV_CONFIG_CLOCK_LINUX                    (1)  // Use Linux clock for now (FreeRTOS compatible)

#define PBDRV_CONFIG_GPIO                           (1)
#define PBDRV_CONFIG_GPIO_ESP32                     (1)

// Minimal port support for now (can be expanded later)
#define PBDRV_CONFIG_IOPORT                         (0)
#define PBDRV_CONFIG_IOPORT_NUM_DEV                 (0)

// LED support
#define PBDRV_CONFIG_LED                            (1)
#define PBDRV_CONFIG_LED_NUM_DEV                    (1)
#define PBDRV_CONFIG_LED_PWM                        (1)
#define PBDRV_CONFIG_LED_PWM_NUM_DEV                (1)

// Motor support disabled for minimal implementation
#define PBDRV_CONFIG_MOTOR_DRIVER                   (0)
#define PBDRV_CONFIG_MOTOR_DRIVER_NUM_DEV           (0)

#define PBDRV_CONFIG_PWM                            (0)
#define PBDRV_CONFIG_PWM_NUM_DEV                    (0)

#define PBDRV_CONFIG_RESET                          (0)  // Disable for now

#define PBDRV_CONFIG_UART                           (0)  // Disable for minimal build

// USB support - disable for now, will implement later
#define PBDRV_CONFIG_USB                            (0)

// Watchdog - disable for now
#define PBDRV_CONFIG_WATCHDOG                       (0)

// No ports initially
#define PBDRV_CONFIG_HAS_PORT_A                     (0)
#define PBDRV_CONFIG_HAS_PORT_B                     (0)
#define PBDRV_CONFIG_HAS_PORT_C                     (0)
#define PBDRV_CONFIG_HAS_PORT_D                     (0)

// ESP32 system clock varies by model, typically 160-240 MHz
#define PBDRV_CONFIG_SYS_CLOCK_RATE                 240000000
#define PBDRV_CONFIG_INIT_ENABLE_INTERRUPTS_FREERTOS (1)
