# SPDX-License-Identifier: MIT
# Copyright (c) 2025 The Pybricks Authors

# Makefile for Pybricks ESP32 Hub
#
# This wraps the ESP-IDF build system (idf.py/cmake)
# Similar to micropython/ports/esp32/Makefile

PBIO_PLATFORM = esp32
PBTOP = ../..

# ESP32 variant - using custom Pybricks board
BOARD ?= PYBRICKS_ESP32_S3

# Build directory
BUILD ?= build-$(BOARD)

# Device serial settings
ifneq ($(PORT),)
PORT_ARG := -p $(PORT)
endif

ifneq ($(BAUD),)
BAUD_ARG := -b $(BAUD)
endif

DEVICE += $(PORT_ARG) $(BAUD_ARG)

PYTHON ?= python3

.PHONY: all clean deploy erase monitor

# Add Pybricks modules as USER_C_MODULES
CMAKE_ARGS = -DUSER_C_MODULES=$(PBTOP)

# Point to Pybricks platform config
IDFPY_FLAGS += -D PBIO_PLATFORM=$(PBIO_PLATFORM)
IDFPY_FLAGS += -D MICROPY_BOARD=$(BOARD)
IDFPY_FLAGS += -D MICROPY_BOARD_DIR="$(PBTOP)/bricks/esp32hub/boards/$(BOARD)"
IDFPY_FLAGS += $(CMAKE_ARGS)

# Set frozen manifest
IDFPY_FLAGS += -D MICROPY_FROZEN_MANIFEST="$(PBTOP)/bricks/esp32hub/manifest.py"

define RUN_IDF_PY
	$(Q)cd $(PBTOP)/micropython/ports/esp32 && idf.py $(IDFPY_FLAGS) -B $(PBTOP)/bricks/esp32hub/$(BUILD) $(1)
endef

all:
	@echo "Building Pybricks ESP32 Hub..."
	$(call RUN_IDF_PY,build)

clean:
	$(call RUN_IDF_PY,fullclean)

deploy:
	$(call RUN_IDF_PY,$(DEVICE) flash)

erase:
	$(call RUN_IDF_PY,$(DEVICE) erase-flash)

monitor:
	$(call RUN_IDF_PY,$(DEVICE) monitor)

size:
	$(call RUN_IDF_PY,size)

# Build mpy-cross compiler (needed for frozen modules)
mpy-cross:
	$(MAKE) -C $(PBTOP)/micropython/mpy-cross

.PHONY: mpy-cross
