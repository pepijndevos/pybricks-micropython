# SPDX-License-Identifier: MIT
# Copyright (c) 2025 The Pybricks Authors

# Board configuration for Pybricks ESP32-S3

set(IDF_TARGET esp32s3)

# Use 4MB flash partition table
set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.usb
    boards/sdkconfig.ble
    boards/sdkconfig.240mhz
)
