# Pybricks ESP32 Hub

This directory contains the Pybricks port for ESP32-based microcontrollers.

## Status

**⚠️ WORK IN PROGRESS - Early Development Stage**

This is a minimal implementation to explore ESP32 support for Pybricks. Current status:

### ✅ Completed
- Platform configuration files (`lib/pbio/platform/esp32/`)
- Basic brick configuration (`bricks/esp32hub/`)
- Build system integration with ESP-IDF
- Custom board definition for ESP32-S3

### 🚧 In Progress
- Driver implementations (GPIO, BLE, USB, etc.)
- Build system testing
- Basic code execution

### 📋 Planned
- USB upload support (for ESP32-S2/S3/C3)
- BLE upload support (all ESP32 variants)
- Motor drivers
- Sensor support
- IMU support (MPU6050/LSM6DS3)

## Hardware Requirements

### Recommended Board
- **ESP32-S3** development board (e.g., ESP32-S3-DevKitC-1)
  - Native USB support via TinyUSB
  - Dual-core 240MHz
  - 512KB SRAM
  - BLE 5.0 support

### Alternative Boards
- ESP32-S2 (single core, native USB, no Bluetooth Classic)
- ESP32-C3 (RISC-V, native USB, BLE only)
- ESP32 (classic, no native USB, requires USB-to-Serial adapter)

## Prerequisites

### 1. ESP-IDF Installation

Pybricks ESP32 requires ESP-IDF v5.2 or later.

```bash
# Clone ESP-IDF
cd ~
git clone -b v5.4.2 --recursive https://github.com/espressif/esp-idf.git

# Install ESP-IDF
cd ~/esp-idf
./install.sh esp32,esp32s2,esp32s3,esp32c3

# Source environment variables (do this in each new terminal)
source ~/esp-idf/export.sh
```

### 2. Build Tools

```bash
# Install build essentials
sudo apt-get install git make cmake python3 python3-pip

# Verify ESP-IDF is available
idf.py --version
```

## Building

### 1. Clone Repository

```bash
git clone https://github.com/pybricks/pybricks-micropython.git
cd pybricks-micropython
git checkout claude/pybricks-esp32-target-011CUxLoAQCZEkfvYUd5dQgn
```

### 2. Initialize Submodules

```bash
git submodule update --init --recursive micropython
```

### 3. Build mpy-cross

```bash
make -C micropython/mpy-cross
```

### 4. Build Pybricks for ESP32

```bash
cd bricks/esp32hub
make
```

This will build the firmware in `build-PYBRICKS_ESP32_S3/`.

## Flashing

### Erase Flash (first time only)

```bash
make erase PORT=/dev/ttyUSB0
```

### Flash Firmware

```bash
make deploy PORT=/dev/ttyUSB0
```

### Monitor Serial Output

```bash
make monitor PORT=/dev/ttyUSB0
```

Press `Ctrl+]` to exit monitor.

## Usage

### Upload Code via USB (Planned)

Once USB upload is implemented, you'll be able to upload Python code using `pybricksdev`:

```bash
pybricksdev run ble your_program.py
```

### REPL Access

Currently, you can access the MicroPython REPL via USB CDC:

```bash
make monitor PORT=/dev/ttyUSB0
```

## Architecture

### Directory Structure

```
bricks/esp32hub/              # ESP32 brick configuration
├── Makefile                  # Build wrapper for ESP-IDF
├── mpconfigport.h            # MicroPython + Pybricks config
├── manifest.py               # Frozen modules
├── boards/
│   └── PYBRICKS_ESP32_S3/   # Board-specific config
└── README.md                 # This file

lib/pbio/platform/esp32/      # ESP32 hardware abstraction
├── pbdrvconfig.h             # Driver configuration
├── pbioconfig.h              # I/O configuration
├── pbsysconfig.h             # System configuration
├── pbio_os_config.h          # FreeRTOS config
└── platform.c                # Platform initialization
```

### Build System

Unlike STM32-based Pybricks ports which use bare-metal ARM builds, the ESP32 port uses:
- **ESP-IDF**: Espressif's official IoT Development Framework
- **FreeRTOS**: Real-time operating system (included in ESP-IDF)
- **CMake**: Build system (wrapped by Makefile for convenience)

The build process:
1. `make` in `bricks/esp32hub/` calls `idf.py build`
2. ESP-IDF compiles MicroPython + Pybricks as an ESP-IDF component
3. Links against ESP-IDF libraries (FreeRTOS, drivers, NimBLE, etc.)
4. Produces `firmware.bin` ready to flash

### Drivers

Driver implementations needed:
- `lib/pbio/drv/clock/clock_freertos.c` - FreeRTOS-based timing
- `lib/pbio/drv/gpio/gpio_esp32.c` - ESP32 GPIO driver
- `lib/pbio/drv/bluetooth/bluetooth_nimble_esp32.c` - NimBLE integration
- `lib/pbio/drv/usb/usb_esp32.c` - TinyUSB integration
- `lib/pbio/drv/reset/reset_esp32.c` - ESP32 reset handling

## Development Notes

### Differences from STM32 Ports

1. **RTOS vs Bare Metal**: ESP32 runs FreeRTOS, not bare metal
2. **Build System**: Uses ESP-IDF (CMake) instead of GNU Make
3. **Drivers**: Uses ESP-IDF HAL instead of STM32 HAL
4. **Bluetooth**: Uses NimBLE stack instead of BlueNRG/CC2640
5. **USB**: Uses TinyUSB instead of STM32 USB library

### ESP-IDF Integration

The ESP-IDF provides:
- Hardware abstraction layer (HAL) for all peripherals
- FreeRTOS operating system
- WiFi and Bluetooth stacks
- USB device stack (TinyUSB)
- Flash and partition management
- Build system and toolchain

### Memory Layout

ESP32-S3 typical memory:
- **SRAM**: 512KB (runtime)
- **Flash**: 4-16MB (firmware + storage)
  - Bootloader: ~32KB
  - Partition table: 4KB
  - MicroPython app: ~2MB
  - User programs: ~1MB+
  - NVS storage: 64KB

## Troubleshooting

### "idf.py: command not found"

Make sure ESP-IDF is sourced:
```bash
source ~/esp-idf/export.sh
```

### "USB device not found"

1. Check USB cable (must be data cable, not power-only)
2. Check device permissions: `sudo usermod -a -G dialout $USER`
3. Try different USB port
4. For ESP32-S3, hold BOOT button while connecting

### Build Errors

1. Make sure all submodules are initialized:
   ```bash
   git submodule update --init --recursive
   ```

2. Clean build directory:
   ```bash
   make clean
   ```

3. Check ESP-IDF version:
   ```bash
   idf.py --version  # Should be v5.2 or later
   ```

## Contributing

This is experimental code. Contributions welcome!

Areas that need work:
- [ ] Clock driver for FreeRTOS
- [ ] GPIO driver for ESP32
- [ ] NimBLE integration
- [ ] TinyUSB integration
- [ ] Motor drivers
- [ ] Sensor support
- [ ] Testing and validation

## References

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/)
- [MicroPython ESP32 Port](https://github.com/micropython/micropython/tree/master/ports/esp32)
- [Pybricks Documentation](https://pybricks.com)
- [ESP32-S3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)

## License

SPDX-License-Identifier: MIT

Copyright (c) 2025 The Pybricks Authors
