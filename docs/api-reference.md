# RaftMotorControl API Reference

## Overview

The RaftMotorControl API provides JSON-based command interface for controlling stepper motors through the `sendCmdJSON()` method. This document details all supported commands, parameters, and response formats.

## Command Interface

### Primary Method: sendCmdJSON()

**Location**: `MotorControl::sendCmdJSON()` in `components/MotorControl/MotorControl.cpp:199-236`

**Signature**:
```cpp
RaftRetCode sendCmdJSON(const char* cmdJSON)
```

**Return Value**: `RaftRetCode` indicating success or error

## Supported Commands

### 1. Motion Control Commands

#### Basic Motion Command
```json
{
  "cmd": "motion",
  "rel": 0,           // Movement type: 0=absolute, 1=relative
  "nosplit": 0,       // Move splitting: 0=allow, 1=prevent
  "speed": 10,        // Target speed value
  "speedOk": 1,       // Speed validity: 1=use speed, 0=ignore
  "pos": [            // Position array
    {"a": 0, "p": 100}, // Axis 0 to position 100
    {"a": 1, "p": 50}   // Axis 1 to position 50
  ],
  "clearQ": 0,        // Queue management: 1=clear queue, 0=append
  "stop": 0,          // Emergency stop: 1=stop, 0=continue
  "en": 1,            // Motor enable: 1=enable, 0=disable
  "constrain": 0      // Boundary checking: 1=enforce, 0=ignore
}
```

#### Motion Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `cmd` | string | Command type, must be "motion" | Required |
| `rel` | int | 0=absolute positioning, 1=relative movement | 0 |
| `nosplit` | int | 0=allow move splitting, 1=execute as single block | 0 |
| `speed` | float | Target movement speed | 0 |
| `speedOk` | int | 1=use speed parameter, 0=use default speed | 0 |
| `pos` | array | Array of axis position objects | [] |
| `clearQ` | int | 1=clear motion queue before executing | 0 |
| `stop` | int | 1=stop all motion immediately | 0 |
| `en` | int | 1=enable motors, 0=disable motors | 1 |
| `constrain` | int | 1=enforce motion boundaries, 0=ignore limits | 0 |

#### Position Object Format
```json
{
  "a": 0,     // Axis index (0, 1, 2, ...)
  "p": 100.5  // Position value (float)
}
```

#### Advanced Motion Parameters

The MotionArgs class supports additional parameters accessible through the JSON interface:

| Parameter | Type | Description |
|-----------|------|-------------|
| `steps` | int | 1=units are steps, 0=units are axis units |
| `ramped` | int | 1=use ramped motion (acceleration/deceleration), 0=flat motion |
| `feedrate` | float | Feedrate percentage or units per minute |
| `feedrateUnitsPerMin` | int | 1=feedrate in units/min, 0=feedrate as percentage |
| `extrude` | float | Extrusion distance (for 3D printing applications) |
| `extrudeValid` | int | 1=use extrude parameter |
| `moreMovesComing` | int | 1=hint that more moves follow (optimization) |
| `motionTrackingIdx` | int | Motion tracking identifier |

### 2. Motor Configuration Commands

#### Set Maximum Current
```json
{
  "cmd": "maxCurrent",
  "axisIdx": 0,        // Axis index (0, 1, 2, ...)
  "maxCurrentA": 0.5   // Maximum current in amperes
}
```

#### Set Motor Auto-Off Time
```json
{
  "cmd": "offAfter",
  "offAfterS": 10.0    // Seconds to keep motors on after movement
}
```

### 3. Motion Pattern Commands

#### Start Motion Pattern
```json
{
  "cmd": "startPattern",
  "pattern": "homing",  // Pattern name ("homing" is built-in)
  "forMs": 30000       // Runtime in milliseconds (0 = run forever)
}
```

#### Stop Motion Pattern
```json
{
  "cmd": "stopPattern"
}
```

## Data Query Interface

### Named Value Queries

**Method**: `getNamedValue(const char* param, bool& isFresh)`

#### Axis Position Queries
| Query | Description | Return Value |
|-------|-------------|--------------|
| `"0pos"` | Axis 0 position | Current position (float) |
| `"1pos"` | Axis 1 position | Current position (float) |
| `"2pos"` | Axis 2 position | Current position (float) |

#### End-Stop State Queries
| Query | Description | Return Value |
|-------|-------------|--------------|
| `"0min"` | Axis 0 min end-stop | 1.0=triggered, 0.0=not triggered |
| `"0max"` | Axis 0 max end-stop | 1.0=triggered, 0.0=not triggered |
| `"1min"` | Axis 1 min end-stop | 1.0=triggered, 0.0=not triggered |
| `"1max"` | Axis 1 max end-stop | 1.0=triggered, 0.0=not triggered |

#### Step Count Queries
| Query | Description | Return Value |
|-------|-------------|--------------|
| `"0steps"` | Axis 0 total steps | Step count (float) |
| `"1steps"` | Axis 1 total steps | Step count (float) |
| `"2steps"` | Axis 2 total steps | Step count (float) |

#### Legacy Position Queries
| Query | Description | Return Value |
|-------|-------------|--------------|
| `"x"` | X-axis position (axis 0) | Current position (float) |
| `"y"` | Y-axis position (axis 1) | Current position (float) |
| `"z"` | Z-axis position (axis 2) | Current position (float) |

#### System Status Queries
| Query | Description | Return Value |
|-------|-------------|--------------|
| `"b"` | Busy state | 1.0=busy, 0.0=idle |

### JSON Data Output

**Method**: `getDataJSON(RaftDeviceJSONLevel level)`

**Levels**:
- `DEVICE_JSON_LEVEL_MIN`: Minimal status information
- `DEVICE_JSON_LEVEL_MED`: Standard status information  
- `DEVICE_JSON_LEVEL_MAX`: Comprehensive diagnostics

**Sample Output**:
```json
{
  "axes": {
    "0": {"pos": 100.5, "steps": 2010, "enabled": true},
    "1": {"pos": 50.0, "steps": 1000, "enabled": true}
  },
  "motion": {
    "busy": false,
    "paused": false,
    "queueSlots": 10
  },
  "endstops": {
    "0": {"min": false, "max": false},
    "1": {"min": false, "max": false}
  }
}
```

## Capabilities

**Method**: `hasCapability(const char* capabilityStr)`

### Supported Capabilities
| Capability | Description |
|------------|-------------|
| `"s"` | Streaming outbound data capability |

## Error Handling

### Return Codes
All commands return `RaftRetCode` values:
- `RAFT_OK`: Command executed successfully
- `RAFT_INVALID_DATA`: Invalid command format or parameters
- `RAFT_NOT_IMPLEMENTED`: Command not implemented
- `RAFT_BUSY`: System busy, command rejected

### Command Validation
- JSON parsing errors result in silent failure
- Unknown commands are ignored
- Invalid parameters use default values where possible
- Motion commands validate axis indices and position ranges

## Implementation Notes

### Command Processing Flow
1. **JSON Parsing**: Commands parsed using RaftJson class
2. **Command Routing**: Based on `cmd` parameter in `sendCmdJSON()`
3. **Parameter Extraction**: Type-safe parameter extraction with defaults
4. **Execution**: Forwarded to appropriate controller methods
5. **Response**: Return code indicates success/failure

### Thread Safety
- Commands processed in main thread context
- Motion controller handles concurrent access internally
- End-stop monitoring runs in interrupt context

### Performance Considerations
- JSON parsing overhead on each command
- Motion commands queued for smooth execution
- Real-time constraints maintained through hardware timers
- Memory allocation minimized during operation

## Examples

### Complete Motion Sequence
```json
// Clear queue and move to origin
{"cmd":"motion","clearQ":1,"rel":0,"pos":[{"a":0,"p":0},{"a":1,"p":0}]}

// Relative move with controlled speed
{"cmd":"motion","rel":1,"speed":50,"speedOk":1,"pos":[{"a":0,"p":10}]}

// Absolute positioning with boundary checking
{"cmd":"motion","rel":0,"constrain":1,"pos":[{"a":0,"p":100},{"a":1,"p":50}]}
```

### Homing Sequence
```json
// Start homing pattern
{"cmd":"startPattern","pattern":"homing","forMs":30000}

// (Wait for completion, monitor via getNamedValue("b"))

// Set origin after homing
{"cmd":"motion","rel":0,"pos":[{"a":0,"p":0},{"a":1,"p":0}]}
```

### Motor Configuration
```json
// Set motor current limits
{"cmd":"maxCurrent","axisIdx":0,"maxCurrentA":1.0}
{"cmd":"maxCurrent","axisIdx":1,"maxCurrentA":0.8}

// Configure auto-off
{"cmd":"offAfter","offAfterS":5.0}
```