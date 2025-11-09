# Pybricks BLE Broadcast Protocol

This document explains how the Pybricks BLE broadcast protocol works and how to implement it using ESP32's native BLE stack (NimBLE via ESP-IDF).

## Protocol Overview

Pybricks uses **BLE advertising packets** (non-connectable advertisements) to broadcast data between hubs. This is a lightweight, connectionless protocol perfect for simple data sharing.

### Key Features
- **One-way broadcast**: No connection needed, just advertise data
- **Low latency**: Direct advertising, no connection overhead
- **Multiple channels**: Broadcast on channel 0-255, observe multiple channels simultaneously
- **Type-safe**: Supports Python types (bool, int, float, str, bytes)
- **LEGO vendor ID**: Uses official LEGO Company ID (0x0397) in manufacturer-specific data

## Advertisement Packet Format

### BLE Advertisement Structure

```
┌──────────────────────────────────────────────┐
│           BLE Advertisement Packet            │
├──────────┬───────────────────────────────────┤
│  Length  │  Type  │         Data             │
├──────────┼────────┼──────────────────────────┤
│  1 byte  │ 1 byte │  Variable (0-29 bytes)   │
└──────────┴────────┴──────────────────────────┘
```

### Pybricks Manufacturer-Specific Data Format

```
Advertisement Data (up to 31 bytes total):
┌────────┬──────┬─────────┬────────┬──────────┐
│ Length │ 0xFF │ LEGO_ID │Channel │ Payload  │
├────────┼──────┼─────────┼────────┼──────────┤
│ N+4    │  MFG │  0x97   │  0-255 │  N bytes │
│ 1 byte │ 1 B  │  0x03   │ 1 byte │ N bytes  │
└────────┴──────┴─────────┴────────┴──────────┘
                 ^─────────^
                 Little-endian
```

**Fields:**
- **Length**: Total length of this AD structure (N + 4)
- **0xFF**: Manufacturer-specific data type
- **LEGO_ID**: `0x0397` (little-endian: `0x97 0x03`)
- **Channel**: Broadcast channel number (1-255, user-selectable)
- **Payload**: Encoded Python objects (up to 26 bytes)

### Payload Encoding

Each value in the payload is encoded as:

```
┌──────────┬───────────────────┐
│  Header  │  Data (optional)  │
├──────────┼───────────────────┤
│  1 byte  │  0-N bytes        │
└──────────┴───────────────────┘
     │
     └─► [Type: 3 bits][Size: 5 bits]
```

**Type Codes** (3 bits, stored in upper 3 bits):
- `0`: SINGLE_OBJECT marker (not a tuple)
- `1`: `True`
- `2`: `False`
- `3`: `int` (1, 2, or 4 bytes, little-endian, signed)
- `4`: `float` (4 bytes, IEEE 754 single-precision)
- `5`: `str` (UTF-8 encoded)
- `6`: `bytes` (raw binary)

**Size** (5 bits, stored in lower 5 bits):
- For booleans: always 0
- For int/float/str/bytes: number of data bytes (0-31 max)

## Python API

### Broadcast Example

```python
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

hub = PrimeHub(broadcast_channel=1)

# Broadcast single value
hub.ble.broadcast(42)

# Broadcast multiple values (tuple)
hub.ble.broadcast([100, 3.14, "hello"])

# Stop broadcasting
hub.ble.broadcast(None)
```

### Observe Example

```python
from pybricks.hubs import PrimeHub
from pybricks.tools import wait

hub = PrimeHub(observe_channels=[2, 3])

while True:
    # Get data from channel 2
    data = hub.ble.observe(2)
    if data:
        print("Channel 2:", data)

    wait(100)
```

## Implementation for ESP32

### Architecture

ESP32 uses **NimBLE** (Apache Mynewt's BLE stack) which is already integrated into ESP-IDF. The implementation needs:

1. **Advertising API**: Set advertisement data and start non-connectable advertising
2. **Scanning API**: Scan for advertisements and filter by manufacturer data
3. **Data encoding/decoding**: Same as Pybricks implementation

### ESP-IDF Code Structure

```
lib/pbio/drv/bluetooth/
├── bluetooth_nimble_esp32.c      ← New file (main implementation)
├── bluetooth_nimble_esp32.h      ← New file (platform-specific headers)
└── bluetooth.c                     ← Common code (already exists)
```

### Required Functions

Based on the driver interface, you need to implement:

```c
// Initialize BLE subsystem
void pbdrv_bluetooth_init(void);

// Start broadcasting with custom data
pbio_error_t pbdrv_bluetooth_start_broadcasting_func(
    pbio_os_state_t *state,
    void *context
);

// Start observing advertisements
pbio_error_t pbdrv_bluetooth_start_observing_func(
    pbio_os_state_t *state,
    void *context
);

// Stop observing
pbio_error_t pbdrv_bluetooth_stop_observing_func(
    pbio_os_state_t *state,
    void *context
);
```

### ESP32 NimBLE Implementation

Here's how to implement broadcasting with ESP32's NimBLE stack:

#### 1. Initialize NimBLE

```c
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

void pbdrv_bluetooth_init(void) {
    // Initialize NimBLE host stack
    nimble_port_init();

    // Configure host stack
    ble_hs_cfg.sync_cb = ble_sync_callback;

    // Start NimBLE host task
    nimble_port_freertos_init(nimble_host_task);
}
```

#### 2. Set Advertisement Data

```c
#include "host/ble_gap.h"

// Storage for broadcast data (from Pybricks API)
extern uint8_t pbdrv_bluetooth_broadcast_data[31];
extern uint8_t pbdrv_bluetooth_broadcast_data_size;

static int set_adv_data(void) {
    struct ble_hs_adv_fields fields = {0};

    // Set manufacturer-specific data
    fields.mfg_data = pbdrv_bluetooth_broadcast_data;
    fields.mfg_data_len = pbdrv_bluetooth_broadcast_data_size;

    // Set advertisement data
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
```

#### 3. Start Non-Connectable Advertising

```c
pbio_error_t pbdrv_bluetooth_start_broadcasting_func(
    pbio_os_state_t *state,
    void *context
) {
    PBIO_OS_ASYNC_BEGIN(state);

    // Set advertisement data (from Pybricks broadcast() call)
    if (set_adv_data() != 0) {
        PBIO_OS_ASYNC_END(PBIO_ERROR_FAILED);
    }

    // Configure non-connectable advertising
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;  // Non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;  // Non-discoverable
    adv_params.itvl_min = 0x00A0;  // 100ms interval
    adv_params.itvl_max = 0x00A0;

    // Start advertising
    int rc = ble_gap_adv_start(
        BLE_OWN_ADDR_PUBLIC,    // Use public address
        NULL,                    // No directed advertising
        BLE_HS_FOREVER,         // Advertise forever
        &adv_params,
        NULL,                    // No callback needed
        NULL
    );

    if (rc != 0 && rc != BLE_HS_EALREADY) {
        PBIO_OS_ASYNC_END(PBIO_ERROR_FAILED);
    }

    pbdrv_bluetooth_advertising_state = PBDRV_BLUETOOTH_ADVERTISING_STATE_BROADCASTING;

    PBIO_OS_ASYNC_END(PBIO_SUCCESS);
}
```

#### 4. Start Scanning/Observing

```c
// Callback for scan results
static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            // Extract advertisement data
            struct ble_hs_adv_fields fields;
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);

            // Check if it's LEGO manufacturer data
            if (fields.mfg_data && fields.mfg_data_len >= 5) {
                uint16_t company_id = (fields.mfg_data[1] << 8) | fields.mfg_data[0];

                if (company_id == 0x0397) {  // LEGO Company ID
                    // Call Pybricks observe callback
                    if (pbdrv_bluetooth_observe_callback) {
                        pbdrv_bluetooth_observe_callback(
                            PBDRV_BLUETOOTH_AD_TYPE_ADV_NONCONN_IND,
                            fields.mfg_data,
                            fields.mfg_data_len,
                            event->disc.rssi
                        );
                    }
                }
            }
            break;
        }
        default:
            break;
    }
    return 0;
}

pbio_error_t pbdrv_bluetooth_start_observing_func(
    pbio_os_state_t *state,
    void *context
) {
    PBIO_OS_ASYNC_BEGIN(state);

    // Configure scan parameters
    struct ble_gap_disc_params disc_params = {0};
    disc_params.filter_duplicates = 0;  // Don't filter duplicates (need fresh RSSI)
    disc_params.passive = 1;            // Passive scanning
    disc_params.itvl = 0x0010;          // 10ms scan interval
    disc_params.window = 0x0010;        // 10ms scan window

    // Start scanning
    int rc = ble_gap_disc(
        BLE_OWN_ADDR_PUBLIC,
        BLE_HS_FOREVER,
        &disc_params,
        gap_event_handler,
        NULL
    );

    if (rc != 0) {
        PBIO_OS_ASYNC_END(PBIO_ERROR_FAILED);
    }

    pbdrv_bluetooth_is_observing = true;

    PBIO_OS_ASYNC_END(PBIO_SUCCESS);
}
```

#### 5. Stop Observing

```c
pbio_error_t pbdrv_bluetooth_stop_observing_func(
    pbio_os_state_t *state,
    void *context
) {
    PBIO_OS_ASYNC_BEGIN(state);

    // Stop scanning
    int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        PBIO_OS_ASYNC_END(PBIO_ERROR_FAILED);
    }

    pbdrv_bluetooth_is_observing = false;

    PBIO_OS_ASYNC_END(PBIO_SUCCESS);
}
```

## Complete Example: Minimal BLE Broadcast

Here's a minimal standalone ESP32 example showing just the broadcast protocol:

```c
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

#define LEGO_CID 0x0397
#define BROADCAST_CHANNEL 1

// Encode an integer into Pybricks format
static size_t encode_int(uint8_t *buf, int32_t value) {
    uint8_t type = 3;  // INT type
    uint8_t size;

    if (value >= INT8_MIN && value <= INT8_MAX) {
        size = 1;
        buf[0] = (type << 5) | size;
        buf[1] = (int8_t)value;
        return 2;
    } else if (value >= INT16_MIN && value <= INT16_MAX) {
        size = 2;
        buf[0] = (type << 5) | size;
        *(int16_t*)&buf[1] = (int16_t)value;
        return 3;
    } else {
        size = 4;
        buf[0] = (type << 5) | size;
        *(int32_t*)&buf[1] = value;
        return 5;
    }
}

void broadcast_value(int32_t value) {
    uint8_t adv_data[31];
    size_t index = 0;

    // Build manufacturer-specific data
    adv_data[index++] = 0xFF;  // Type: Manufacturer Specific
    adv_data[index++] = 0x97;  // LEGO CID low byte
    adv_data[index++] = 0x03;  // LEGO CID high byte
    adv_data[index++] = BROADCAST_CHANNEL;

    // Encode the value
    index += encode_int(&adv_data[index], value);

    // Calculate length
    adv_data[0] = index - 1;

    // Set advertisement data
    struct ble_hs_adv_fields fields = {0};
    fields.mfg_data = adv_data;
    fields.mfg_data_len = index;

    ble_gap_adv_set_fields(&fields);

    // Start non-connectable advertising
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;

    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
}

void app_main(void) {
    // Initialize NimBLE
    esp_nimble_hci_init();
    nimble_port_init();

    // Broadcast a value
    broadcast_value(42);

    // Keep broadcasting
    vTaskDelay(portMAX_DELAY);
}
```

## Testing

### With nRF Connect (Mobile App)

1. Install "nRF Connect" on your phone
2. Scan for devices
3. Look for manufacturer data with ID `0x0397`
4. View the raw data to see your encoded values

### With Another Pybricks Hub

```python
# Broadcaster (Hub A)
hub = PrimeHub(broadcast_channel=1)
hub.ble.broadcast(42)

# Observer (Hub B)
hub = PrimeHub(observe_channels=[1])
while True:
    data = hub.ble.observe(1)
    if data:
        print(data)  # Should print: 42
    wait(100)
```

## Key Differences: BTStack vs NimBLE

| Feature | BTStack (Pybricks) | NimBLE (ESP32) |
|---------|-------------------|----------------|
| API Style | Command queue | Direct function calls |
| Advertisement | `gap_advertisements_set_data()` | `ble_gap_adv_set_fields()` |
| Scanning | `gap_start_scan()` | `ble_gap_disc()` |
| Callbacks | Event packets | Event structs |
| Threading | Single-threaded | FreeRTOS task |

## Summary

The Pybricks BLE broadcast protocol is:
1. **Simple**: Just manufacturer-specific advertisement data
2. **Standard BLE**: Works with any BLE stack
3. **Type-safe**: Custom encoding for Python types
4. **Low overhead**: No connections, just advertisements

ESP32 implementation is straightforward using NimBLE APIs that map directly to the BTStack functions Pybricks uses on other platforms.

## Next Steps

To integrate this into the ESP32 platform:
1. Create `bluetooth_nimble_esp32.c` with the functions shown above
2. Update `pbdrvconfig.h` to enable `PBDRV_CONFIG_BLUETOOTH_NIMBLE_ESP32`
3. Add ESP-IDF NimBLE components to build system
4. Test with basic broadcast/observe examples

The encoding/decoding logic in `pybricks/common/pb_type_ble.c` can be reused as-is since it's platform-independent.
