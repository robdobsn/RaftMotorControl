# MotorControl Binary Data Publishing Implementation

## Overview

This document describes the implementation required to add binary data publishing support to the MotorControl device. Binary data publishing is more efficient than JSON for high-frequency status updates, particularly over constrained communication channels like BLE or WebSocket connections.

## Current State

The MotorControl device currently publishes data via:
- `getStatusJSON()` → delegates to `getDataJSON(DEVICE_JSON_LEVEL_PUBLISH)` → returns JSON with operational state

JSON output structure (from `MotionController::getDataJSON(DEVICE_JSON_LEVEL_PUBLISH)`):
```json
{
  "pos": [x, y, z],
  "steps": [step0, step1, step2],
  "busy": true/false,
  "paused": true/false,
  "pattern": "patternName"
}
```

The `DeviceManager` calls `getStatusBinary()` on all devices to aggregate binary data, but `MotorControl` currently returns an empty vector (default `RaftDevice` implementation).

## Binary Data Infrastructure

The RaftCore framework provides the following infrastructure for binary data publishing:

### 1. `RaftDevice::getStatusBinary()`
Virtual method that returns binary data for publishing. Default returns empty vector.

### 2. `RaftDevice::genBinaryDataMsg()`
Static helper that wraps device data with a standard header:
```
[2 bytes] Message length (big-endian, excluding these 2 bytes)
[1 byte]  Connection mode | (isOnline ? 0x80 : 0x00)
[4 bytes] Address (big-endian)
[2 bytes] Device type index (big-endian)
[N bytes] Device-specific data
```

### 3. `DeviceTypeRecordDynamic`
Provides schema information for external decoders. The schema JSON uses Python struct format specifiers.

### 4. `RaftDevice::getDeviceTypeRecord()`
Virtual method that returns the device type record including the binary schema.

## Data Types

From `AxesValues.h`:
- `AxisPosDataType` = `float` (4 bytes)
- `AxisStepsDataType` = `int32_t` (4 bytes)
- `AXIS_VALUES_MAX_AXES` = 3

## Proposed Binary Format

### Data Layout (Fixed Size: 31 bytes)

| Offset | Size | Type | Name | Description |
|--------|------|------|------|-------------|
| 0 | 2 | uint16_t (BE) | timestamp | Lower 16 bits of milliseconds timestamp |
| 2 | 4 | float (BE) | pos0 | Position axis 0 (mm or degrees) |
| 6 | 4 | float (BE) | pos1 | Position axis 1 (mm or degrees) |
| 10 | 4 | float (BE) | pos2 | Position axis 2 (mm or degrees) |
| 14 | 4 | int32_t (BE) | steps0 | Step count axis 0 |
| 18 | 4 | int32_t (BE) | steps1 | Step count axis 1 |
| 22 | 4 | int32_t (BE) | steps2 | Step count axis 2 |
| 26 | 1 | uint8_t | flags | Bit 0: busy, Bit 1: paused |
| 27 | 4 | uint8_t[4] | patternHash | First 4 bytes of pattern name (or hash) |

**Total: 31 bytes** (excluding wrapper header from `genBinaryDataMsg`)

### Alternative: Variable-length Pattern Name

If pattern name must be preserved exactly:

| Offset | Size | Type | Name | Description |
|--------|------|------|------|-------------|
| 0 | 2 | uint16_t (BE) | timestamp | Lower 16 bits of ms timestamp |
| 2 | 12 | float[3] (BE) | pos | Position values for 3 axes |
| 14 | 12 | int32_t[3] (BE) | steps | Step counts for 3 axes |
| 26 | 1 | uint8_t | flags | Bit 0: busy, Bit 1: paused |
| 27 | 1 | uint8_t | patternLen | Length of pattern name (0-255) |
| 28 | N | char[N] | pattern | Pattern name string (no null terminator) |

## Schema JSON

The schema follows the format used by other devices in RaftCore:

```json
{
  "name": "Motor Controller",
  "desc": "Multi-axis Motor Controller Status",
  "manu": "Robotical",
  "type": "MotorControl",
  "resp": {
    "b": 31,
    "a": [
      {"n": "pos0", "t": ">f", "u": "mm", "r": [-1000, 1000], "d": 1, "f": ".2f", "o": "float"},
      {"n": "pos1", "t": ">f", "u": "mm", "r": [-1000, 1000], "d": 1, "f": ".2f", "o": "float"},
      {"n": "pos2", "t": ">f", "u": "mm", "r": [-1000, 1000], "d": 1, "f": ".2f", "o": "float"},
      {"n": "steps0", "t": ">i", "u": "steps", "r": [-2147483648, 2147483647], "d": 1, "f": "d", "o": "int"},
      {"n": "steps1", "t": ">i", "u": "steps", "r": [-2147483648, 2147483647], "d": 1, "f": "d", "o": "int"},
      {"n": "steps2", "t": ">i", "u": "steps", "r": [-2147483648, 2147483647], "d": 1, "f": "d", "o": "int"},
      {"n": "busy", "t": ">B", "u": "", "r": [0, 1], "d": 1, "f": "d", "o": "bool", "m": "0x01"},
      {"n": "paused", "t": ">B", "u": "", "r": [0, 1], "d": 1, "f": "d", "o": "bool", "m": "0x02", "s": true}
    ]
  }
}
```

### Hex Values in JSON

The bitmask values (`"m": "0x01"`) are specified as hex strings rather than numeric literals. This is valid because:

1. **JSON Compliance**: JSON only supports decimal number literals; `0x01` would be invalid JSON syntax
2. **RaftJson Support**: The `RaftJson::getLongIm()` function uses `strtol(pJsonDocPos, NULL, 0)` which automatically detects the base from the string prefix:
   - `0x` or `0X` → hexadecimal
   - `0` prefix → octal  
   - Otherwise → decimal
3. **String Handling**: When `RAFT_JSON_TREAT_STRINGS_AS_NUMBERS` is enabled, RaftJson skips the opening quote before parsing, so `"0x01"` is correctly interpreted as hexadecimal 1

### Schema Field Definitions

| Field | Description |
|-------|-------------|
| `n` | Name of the field |
| `t` | Python struct format: `>` = big-endian, `f` = float, `i` = int32, `h` = int16, `H` = uint16, `B` = uint8 |
| `u` | Unit (e.g., "mm", "steps", "deg/s") |
| `r` | Range [min, max] |
| `d` | Divisor for scaling (1 = no scaling) |
| `f` | Format string for display |
| `o` | Output type for decoded value |
| `m` | Bitmask for extracting value from byte (for flags) |
| `s` | Skip byte read (share byte with previous field) |

## Implementation Plan

### 1. Add to MotionController

**File: `components/MotorControl/Controller/MotionController.h`**
```cpp
// Add method declaration
void formBinaryDataResponse(std::vector<uint8_t>& data) const;
```

**File: `components/MotorControl/Controller/MotionController.cpp`**
```cpp
void MotionController::formBinaryDataResponse(std::vector<uint8_t>& data) const
{
    // Timestamp (16-bit)
    uint16_t timeVal = (uint16_t)(millis() & 0xFFFF);
    data.push_back((timeVal >> 8) & 0xFF);
    data.push_back(timeVal & 0xFF);
    
    // Position values (3 x float, big-endian)
    AxesValues<AxisPosDataType> pos = getLastMonitoredPos();
    for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        float val = pos.getVal(i);
        uint32_t floatBits;
        memcpy(&floatBits, &val, sizeof(float));
        data.push_back((floatBits >> 24) & 0xFF);
        data.push_back((floatBits >> 16) & 0xFF);
        data.push_back((floatBits >> 8) & 0xFF);
        data.push_back(floatBits & 0xFF);
    }
    
    // Step counts (3 x int32, big-endian)
    AxesValues<AxisStepsDataType> steps = getAxisTotalSteps();
    for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        int32_t val = steps.getVal(i);
        data.push_back((val >> 24) & 0xFF);
        data.push_back((val >> 16) & 0xFF);
        data.push_back((val >> 8) & 0xFF);
        data.push_back(val & 0xFF);
    }
    
    // Flags byte
    uint8_t flags = 0;
    if (isBusy()) flags |= 0x01;
    if (isPaused()) flags |= 0x02;
    data.push_back(flags);
    
    // Pattern name (first 4 bytes as identifier)
    const String& pattern = getCurrentMotionPatternName();
    for (uint32_t i = 0; i < 4; i++)
    {
        data.push_back(i < pattern.length() ? pattern[i] : 0);
    }
}
```

### 2. Add to MotorControl

**File: `components/MotorControl/MotorControl.h`**
```cpp
// Add method declarations
virtual std::vector<uint8_t> getStatusBinary() const override final;
virtual bool getDeviceTypeRecord(DeviceTypeRecordDynamic& devTypeRec) const override final;
```

**File: `components/MotorControl/MotorControl.cpp`**
```cpp
std::vector<uint8_t> MotorControl::getStatusBinary() const
{
    // Get device data from motion controller
    std::vector<uint8_t> data;
    _motionController.formBinaryDataResponse(data);
    
    // Wrap with standard header
    std::vector<uint8_t> binBuf;
    RaftDevice::genBinaryDataMsg(binBuf, DeviceManager::DEVICE_CONN_MODE_DIRECT, 
                                  0, getDeviceTypeIndex(), true, data);
    return binBuf;
}

bool MotorControl::getDeviceTypeRecord(DeviceTypeRecordDynamic& devTypeRec) const
{
    static const char* devInfoJson = R"~({"name":"Motor Controller","desc":"Multi-axis Motor Controller","manu":"Robotical","type":"MotorControl")~"
        R"~(,"resp":{"b":31,"a":[)~"
        R"~({"n":"pos0","t":">f","u":"mm","r":[-1000,1000],"d":1,"f":".2f","o":"float"},)~"
        R"~({"n":"pos1","t":">f","u":"mm","r":[-1000,1000],"d":1,"f":".2f","o":"float"},)~"
        R"~({"n":"pos2","t":">f","u":"mm","r":[-1000,1000],"d":1,"f":".2f","o":"float"},)~"
        R"~({"n":"steps0","t":">i","u":"steps","r":[-2147483648,2147483647],"d":1,"f":"d","o":"int"},)~"
        R"~({"n":"steps1","t":">i","u":"steps","r":[-2147483648,2147483647],"d":1,"f":"d","o":"int"},)~"
        R"~({"n":"steps2","t":">i","u":"steps","r":[-2147483648,2147483647],"d":1,"f":"d","o":"int"},)~"
        R"~({"n":"flags","t":">B","u":"","r":[0,255],"d":1,"f":"d","o":"uint8"})~"
        R"~(]}})~";
    
    devTypeRec = DeviceTypeRecordDynamic(
        getPublishDeviceType().c_str(),
        "",     // addresses
        "",     // detectionValues
        "",     // initValues
        "",     // pollInfo
        31,     // pollDataSizeBytes
        devInfoJson,
        nullptr // pollResultDecodeFn
    );
    
    return true;
}
```

### 3. Required Includes

Add to `MotorControl.cpp`:
```cpp
#include "DeviceManager.h"
#include "DeviceTypeRecordDynamic.h"
```

## Considerations

### Dynamic Number of Axes

The current design assumes exactly 3 axes (`AXIS_VALUES_MAX_AXES = 3`). If the number of axes needs to be dynamic:

1. **Option A**: Include axis count in the binary format and use variable-length data
2. **Option B**: Always transmit 3 axes but mark unused axes with sentinel values (e.g., `NaN` for floats)
3. **Option C**: Define separate device types for different axis configurations

### Pattern Name Handling

The pattern name is currently truncated to 4 bytes. Alternatives:

1. **Hash**: Use a 4-byte hash of the pattern name
2. **Index**: Use a predefined index for known patterns
3. **Variable Length**: Include length prefix and full string (increases message size)

For most use cases, 4 characters is sufficient to identify common patterns like "home", "idle", "move".

### Endianness

All multi-byte values use big-endian (network byte order) format, consistent with the Python struct format specifiers (`>` prefix) used in the schema.

## Testing

1. Verify binary data matches JSON data by decoding both and comparing
2. Test with external decoder using the schema
3. Verify `DeviceManager::getDevicesDataBinary()` includes MotorControl data
4. Test change detection via `getDeviceStateHash()` triggers binary updates

## Future Enhancements

1. **Compression**: For high-frequency updates, consider delta encoding
2. **Configurable Fields**: Allow selecting which fields to include in binary output
3. **Binary Commands**: Add `sendCmdBinary()` for efficient command input
