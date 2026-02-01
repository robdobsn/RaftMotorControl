# MotorController State Publishing Implementation Plan

## Overview

This document outlines the implementation plan for adding state publishing capabilities to the MotorController component. This will enable real-time streaming of motor positions and step counts to connected clients via WebSocket, BLE, or serial connections.

## Background

The RaftCore framework provides a StatePublisher SysMod that manages publication of state data from any module. The system uses:
- **Dynamic registration**: No pre-configuration needed - topics are created when a data source registers
- **Explicit subscription**: Clients must subscribe via REST API to receive data
- **Change detection**: Uses hash-based state change detection to minimize unnecessary transmissions
- **Multiple formats**: Supports both JSON and binary data formats

## Requirements

### Data to Publish

The MotorController should publish:
1. **Axis positions** (last commanded and last monitored)
   - Per-axis position values in configured units (mm, degrees, etc.)
2. **Step counts** (total accumulated steps per axis)
   - Raw step counter values from ramp generator
3. **Motion status**
   - Busy state (motion in progress)
   - Paused state
   - Current pattern name (if any)
4. **End-stop states** (optional)
   - Min/max end-stop triggered status per axis

### Topic Name

- **Primary topic**: `motorsjson` - JSON format for easy consumption without decoding
- **Future consideration**: `motorsbin` - Binary format for higher efficiency

## Implementation Architecture

### 1. Registration Location

The registration should occur in **MotorControl.cpp** (the RaftDevice wrapper), NOT in MotionController.cpp, because:
- MotorControl is the RaftDevice that has access to `getSysManager()`
- MotorControl already aggregates data from MotionController
- MotorControl is the public interface for the motor control subsystem

### 2. Data Source Registration

Registration should happen in `MotorControl::setup()` after motion controller initialization:

```cpp
void MotorControl::setup()
{
    // Existing setup code...
    _motionController.setup(deviceConfig);
    
    // Register as state publisher
    registerStatePublisher();
}
```

### 3. Required Components

#### A. Message Generation Callback (`msgGenCB`)

This callback generates the JSON payload when a publication is triggered:

```cpp
// Callback signature: 
// bool msgGenCB(const char* messageName, CommsChannelMsg& msg)

[this](const char* messageName, CommsChannelMsg& msg) {
    String statusJSON = getMotorStatusJSON();
    msg.setFromBuffer((uint8_t*)statusJSON.c_str(), statusJSON.length());
    return true;
}
```

The `getMotorStatusJSON()` method should format data like:
```json
{
  "pos": [100.5, 45.2, 0.0],
  "steps": [10050, 4520, 0],
  "busy": true,
  "paused": false,
  "pattern": ""
}
```

#### B. State Hash Callback (`stateDetectCB`)

This callback computes a hash of current state to detect changes:

```cpp
// Callback signature:
// bool stateDetectCB(const char* messageName, std::vector<uint8_t>& stateHash)

[this](const char* messageName, std::vector<uint8_t>& stateHash) {
    return getMotorStateHash(stateHash);
}
```

The hash should include bytes representing:
- Position values (convert float/double to bytes)
- Step counts (convert int32_t/int64_t to bytes)
- Status flags (busy, paused bits)

### 4. Data Access Methods

The MotorControl class already has access to MotionController methods:
- `_motionController.getLastCommandedPos()` - Returns AxesValues<AxisPosDataType>
- `_motionController.getLastMonitoredPos()` - Returns AxesValues<AxisPosDataType>
- `_motionController.getAxisTotalSteps()` - Returns AxesValues<AxisStepsDataType>
- `_motionController.isBusy()` - Returns bool
- `_motionController.isPaused()` - Returns bool
- `_motionController.getCurrentMotionPatternName()` - Returns const String&

## Detailed Implementation Steps

### Step 1: Add Helper Methods to MotorControl Class

**File**: `components/MotorControl/MotorControl.h`

Add private method declarations:
```cpp
private:
    // State publishing
    void registerStatePublisher();
    String getMotorStatusJSON() const;
    bool getMotorStateHash(std::vector<uint8_t>& stateHash) const;
```

### Step 2: Implement Registration Method

**File**: `components/MotorControl/MotorControl.cpp`

```cpp
void MotorControl::registerStatePublisher()
{
    // Register as a data source for motor state publishing
    getSysManager()->registerDataSource(
        deviceClassName.c_str(),  // SysMod name (or use "MotorControl")
        "motorsjson",              // Publication topic
        // Message generation callback
        [this](const char* messageName, CommsChannelMsg& msg) {
            String statusJSON = getMotorStatusJSON();
            msg.setFromBuffer((uint8_t*)statusJSON.c_str(), statusJSON.length());
            return true;
        },
        // State hash callback
        [this](const char* messageName, std::vector<uint8_t>& stateHash) {
            return getMotorStateHash(stateHash);
        }
    );
    
    LOG_I(MODULE_PREFIX, "Registered state publisher topic: motorsjson");
}
```

### Step 3: Implement JSON Generation Method

**File**: `components/MotorControl/MotorControl.cpp`

```cpp
String MotorControl::getMotorStatusJSON() const
{
    // Get current state from motion controller
    AxesValues<AxisPosDataType> cmdPos = _motionController.getLastCommandedPos();
    AxesValues<AxisPosDataType> monPos = _motionController.getLastMonitoredPos();
    AxesValues<AxisStepsDataType> steps = _motionController.getAxisTotalSteps();
    bool busy = _motionController.isBusy();
    bool paused = _motionController.isPaused();
    String pattern = _motionController.getCurrentMotionPatternName();
    
    // Build JSON string
    // Using monitored position as the primary position feedback
    String json = "{";
    
    // Position array
    json += "\"pos\":[";
    for (uint32_t i = 0; i < monPos.numValues(); i++)
    {
        if (i > 0) json += ",";
        json += String(monPos.getVal(i), 2);  // 2 decimal places
    }
    json += "],";
    
    // Step counts array
    json += "\"steps\":[";
    for (uint32_t i = 0; i < steps.numValues(); i++)
    {
        if (i > 0) json += ",";
        json += String(steps.getVal(i));
    }
    json += "],";
    
    // Status fields
    json += "\"busy\":";
    json += busy ? "true" : "false";
    json += ",\"paused\":";
    json += paused ? "true" : "false";
    json += ",\"pattern\":\"";
    json += pattern;
    json += "\"}";
    
    return json;
}
```

### Step 4: Implement State Hash Method

**File**: `components/MotorControl/MotorControl.cpp`

```cpp
bool MotorControl::getMotorStateHash(std::vector<uint8_t>& stateHash) const
{
    // Get current state
    AxesValues<AxisStepsDataType> steps = _motionController.getAxisTotalSteps();
    bool busy = _motionController.isBusy();
    bool paused = _motionController.isPaused();
    
    // Add step counts to hash
    // Note: Position is not included because it is derived from step counts
    // (through kinematics transformation), so if steps haven't changed, position can't either
    for (uint32_t i = 0; i < steps.numValues(); i++)
    {
        AxisStepsDataType stepVal = steps.getVal(i);
        stateHash.push_back(stepVal & 0xFF);
        stateHash.push_back((stepVal >> 8) & 0xFF);
        stateHash.push_back((stepVal >> 16) & 0xFF);
        stateHash.push_back((stepVal >> 24) & 0xFF);
        // If AxisStepsDataType is 64-bit, add more bytes
        if (sizeof(AxisStepsDataType) > 4)
        {
            stateHash.push_back((stepVal >> 32) & 0xFF);
            stateHash.push_back((stepVal >> 40) & 0xFF);
            stateHash.push_back((stepVal >> 48) & 0xFF);
            stateHash.push_back((stepVal >> 56) & 0xFF);
        }
    }
    
    // Add status flags
    uint8_t statusByte = (busy ? 0x01 : 0x00) | (paused ? 0x02 : 0x00);
    stateHash.push_back(statusByte);
    
    return true;
}
```

### Step 5: Call Registration in Setup

**File**: `components/MotorControl/MotorControl.cpp`

Modify the `setup()` method:
```cpp
void MotorControl::setup()
{
    // Setup motion controller
    _motionController.setup(deviceConfig);

    // Setup serial bus
    String serialBusName = deviceConfig.getString("bus", "");
    _pMotorSerialBus = raftBusSystem.getBusByName(serialBusName);
    _motionController.setupSerialBus(_pMotorSerialBus, false);

    // Register motion patterns
    _motionController.addMotionPattern("homing", HomingPattern::create);

    // Register state publisher (NEW)
    registerStatePublisher();

    // Debug
    LOG_I(MODULE_PREFIX, "setup type %s serialBusName %s%s",
            deviceClassName.c_str(), serialBusName.c_str(),
            _pMotorSerialBus ? "" : " (BUS INVALID)");
}
```

## Configuration

### System Configuration (SysTypes.json)

Enable the StatePublisher SysMod (should already be in the config):
```json
{
  "SysManager": {
    "SysModules": [
      {
        "name": "Publish",
        "type": "Publish",
        "enable": 1
      }
    ]
  }
}
```

No additional configuration is needed - the `motorsjson` topic is created automatically when MotorControl registers.

## Client Subscription

### WebSocket Subscription

Clients must explicitly subscribe to receive motor data:

**Subscription Request** (via WebSocket):
```json
{
  "cmdName": "subscription",
  "action": "update",
  "pubRecs": [
    {
      "topic": "motorsjson",
      "rateHz": 10.0,
      "trigger": "timeorchange",
      "minTimeBetweenMs": 50
    }
  ]
}
```

**Parameters**:
- `topic`: `"motorsjson"` - must match registered topic name
- `rateHz`: Publication rate (e.g., 10 Hz = 10 updates/second)
- `trigger`: `"timeorchange"` - publish on timer OR when state changes
  - `"time"` - only timer-based
  - `"change"` - only on state change
  - `"timeorchange"` - either condition triggers publication
- `minTimeBetweenMs`: Minimum interval between publications (throttling)

### REST API Subscription

```
GET /subscription?action=update&pubRecs=[{"topic":"motorsjson","rateHz":10,"trigger":"timeorchange"}]
```

## Testing Plan

### 1. Basic Functionality
- Verify registration occurs without errors
- Check logs for "Registered state publisher topic: motorsjson"
- Verify topic appears in StatePublisher's list

### 2. Subscription
- Subscribe from WebSocket client
- Verify subscription acknowledgment
- Confirm data begins streaming

### 3. Data Validation
- Move motors and verify position updates
- Check step count increments
- Verify busy/paused states reflect actual status
- Test pattern name field during pattern execution

### 4. Change Detection
- With `trigger: "change"`, verify no publications when stationary
- Verify publications occur when motion starts
- Test that hash mechanism detects position changes

### 5. Rate Control
- Test various `rateHz` values (1, 5, 10, 20 Hz)
- Verify `minTimeBetweenMs` throttling works
- Confirm no flooding during rapid state changes

### 6. Multiple Clients
- Connect multiple WebSocket clients
- Verify each receives independent stream
- Test disconnection/reconnection

## Performance Considerations

### Hash Computation Overhead
- Hash is computed on StatePublisher's timer (typically every 10-50ms)
- Keep hash computation fast - avoid complex calculations
- Current implementation is O(n) where n = number of axes

### JSON Generation Overhead
- JSON is only generated when hash changes
- String concatenation is reasonably efficient for small payloads
- Consider pre-allocating String capacity if needed

### Publication Frequency
- Recommend 10-20 Hz for smooth UI updates
- Higher rates (>50 Hz) may strain WebSocket connection
- Use `minTimeBetweenMs` to prevent flooding

### Network Bandwidth
Estimated JSON payload size for 3-axis system:
```json
{"pos":[100.50,45.20,0.00],"steps":[10050,4520,0],"busy":true,"paused":false,"pattern":""}
```
- Approximate size: ~100 bytes
- At 10 Hz: ~1 KB/s per client
- At 20 Hz: ~2 KB/s per client

This is very manageable for WebSocket connections.

## Future Enhancements

### 1. Binary Format Topic (`motorsbin`)
- Implement `msgGenCB` for binary format
- Use flatbuffers or simple struct packing
- Reduce payload size by ~50%
- Better for BLE connections (bandwidth constrained)

### 2. Configurable Data Selection
- Add JSON config to select which fields to publish
- Allow per-client customization via subscription parameters
- Example: `{"topic":"motorsjson","fields":["pos","busy"]}`

### 3. End-Stop State Publishing
- Add end-stop states to JSON payload
- Useful for UI to show limit switch status
- Add to hash computation

### 4. Velocity Publishing
- Calculate and publish instantaneous velocity
- Useful for motion profiling and tuning
- Requires delta-position / delta-time calculation

### 5. Error/Fault Reporting
- Publish motor driver errors
- Report motion planning failures
- Enable proactive UI alerts

## References

- [RaftStatePublisherSysMod Wiki](https://github.com/robdobsn/RaftCore/wiki/RaftStatePublisherSysMod)
- [Registering as a Data Source Wiki](https://github.com/robdobsn/RaftCore/wiki/RegisteringAsADataSource)
- [DeviceDataPublishing Wiki](https://github.com/robdobsn/RaftCore/wiki/DeviceDataPublishing)

## Example WebUI Integration

### Subscribe to Motor Data
```typescript
// In ConnManager or SystemTypeTwoStepper
const subscribeToMotors = async () => {
  const subscribeCmd = {
    cmdName: "subscription",
    action: "update",
    pubRecs: [
      {
        topic: "motorsjson",
        rateHz: 10,
        trigger: "timeorchange",
        minTimeBetweenMs: 50
      }
    ]
  };
  
  await connector.sendRICRESTMsg(JSON.stringify(subscribeCmd), {});
};
```

### Handle Published Data
```typescript
// Register publish event handler
connector.onRaftEvent('publish', (eventType, eventEnum, eventName, data) => {
  if (eventName === 'motorsjson') {
    const motorData = JSON.parse(data as string);
    // Update UI with motorData.pos, motorData.steps, etc.
    updateMotorPositionDisplay(motorData.pos);
    updateStepCountDisplay(motorData.steps);
    updateStatusIndicators(motorData.busy, motorData.paused);
  }
});
```

## Notes

- The implementation is non-intrusive - it adds publishing capability without modifying MotionController internals
- Publishing is opt-in via subscription - no overhead if not used
- The JSON format is self-documenting and easy to consume
- State hash mechanism minimizes unnecessary network traffic
- Multiple clients can subscribe independently with different rates

## Summary

This implementation provides a clean, efficient way to stream motor position and status data to connected clients. The approach follows RaftCore patterns, is minimally invasive to existing code, and provides flexibility for future enhancements.
