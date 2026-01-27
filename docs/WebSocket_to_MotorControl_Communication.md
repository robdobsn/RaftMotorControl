# WebSocket to MotorControl Communication Flow

## Overview

This document explains how JSON commands sent over WebSockets are processed and ultimately executed by the MotorControl component. The architecture provides a flexible, protocol-agnostic system that allows multiple communication channels to control motor operations.

## High-Level Architecture

```
┌─────────────────┐
│ WebSocket Client│
└────────┬────────┘
         │ JSON over WebSocket
         ▼
┌─────────────────┐
│   WebServer     │ Manages HTTP/WebSocket connections
└────────┬────────┘
         │ CommsChannelMsg
         ▼
┌─────────────────┐
│ProtocolExchange │ Protocol decoding (RICSerial/RICFrame/RICJSON)
└────────┬────────┘
         │ RICRESTMsg
         ▼
┌──────────────────────┐
│RestAPIEndpointManager│ Routes requests to registered endpoints
└────────┬─────────────┘
         │ REST API Call
         ▼
┌─────────────────┐
│   MainSysMod    │ Translates REST API to device commands
└────────┬────────┘
         │ Device JSON Command
         ▼
┌─────────────────┐
│ DeviceManager   │ Routes commands to specific devices
└────────┬────────┘
         │ sendCmdJSON()
         ▼
┌─────────────────┐
│  MotorControl   │ Executes motion commands
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│MotionController │ Controls stepper motors
└─────────────────┘
```

## Detailed Component Flow

### 1. WebSocket Configuration

**Location**: `test_rigs/TwoStepperMagRotTestRig/systypes/SysTypeMain/SysTypes.json`

The WebServer is configured with WebSocket support:

```json
"WebServer": {
  "enable": 1,
  "webServerPort": 80,
  "websockets": [{
    "maxConn": 4,
    "pcol": "RICSerial",
    "pfix": "ws",
    "pingMs": 1000,
    "txQueueMax": 20
  }]
}
```

**Key Parameters:**
- `pcol`: Protocol used for encoding messages (RICSerial)
- `pfix`: WebSocket endpoint prefix (`ws`)
- `maxConn`: Maximum simultaneous WebSocket connections
- `txQueueMax`: Maximum outbound message queue length

### 2. Protocol Exchange

**Location**: `test_rigs/TwoStepperMagRotTestRig/build/SysTypeMain/raft/RaftCore/components/comms/ProtocolExchange/`

ProtocolExchange acts as the central hub for all protocol messages. It supports three protocol types:

#### Supported Protocols

1. **RICSerial** - Serial-like protocol with framing
   - Frame boundary character: `0xE7`
   - Control escape character: `0xD7`
   - Used by WebSockets and serial console

2. **RICFrame** - Binary frame protocol
   - For efficient binary data transfer

3. **RICJSON** - Direct JSON protocol
   - For simple JSON message passing

#### Message Processing

```cpp
bool ProtocolExchange::processEndpointMsg(CommsChannelMsg &cmdMsg)
{
    if (protocol == MSG_PROTOCOL_RICREST)
    {
        RICRESTMsg ricRESTReqMsg;
        ricRESTReqMsg.decode(cmdMsg.getBuf(), cmdMsg.getBufLen());
        
        switch(ricRESTReqMsg.getElemCode())
        {
            case RICREST_ELEM_CODE_URL:
                // Process REST URL (e.g., "motors/setPos?a0=100")
                processRICRESTURL(ricRESTReqMsg, respMsg, sourceInfo);
                break;
                
            case RICREST_ELEM_CODE_COMMAND_FRAME:
                // Process JSON command frame
                processRICRESTCmdFrame(ricRESTReqMsg, respMsg, cmdMsg);
                break;
                
            case RICREST_ELEM_CODE_FILEBLOCK:
                // Process file upload blocks
                processRICRESTFileStreamBlock(ricRESTReqMsg, respMsg, cmdMsg);
                break;
        }
    }
}
```

### 3. REST API Endpoint Manager

**Location**: `test_rigs/TwoStepperMagRotTestRig/build/SysTypeMain/raft/RaftCore/components/core/RestAPIEndpoints/`

The RestAPIEndpointManager maintains a registry of all REST API endpoints and routes incoming requests to the appropriate handler.

#### Endpoint Registration

Each SysMod can register endpoints during the `addRestAPIEndpoints()` phase:

```cpp
void MainSysMod::addRestAPIEndpoints(RestAPIEndpointManager &endpointManager)
{
    endpointManager.addEndpoint(
        "motors",                              // Endpoint name
        RestAPIEndpoint::ENDPOINT_CALLBACK,    // Type
        RestAPIEndpoint::ENDPOINT_GET,         // HTTP method
        std::bind(&MainSysMod::apiControl, this, 
                  std::placeholders::_1,
                  std::placeholders::_2,
                  std::placeholders::_3),
        "Control Motors"                       // Description
    );
}
```

#### Request Routing

When a URL request arrives, the manager:
1. Parses the endpoint name from the URL
2. Finds the matching registered endpoint
3. Calls the endpoint's callback function
4. Returns the response

### 4. MainSysMod - Command Translation Layer

**Location**: `test_rigs/TwoStepperMagRotTestRig/components/MainSysMod/`

MainSysMod acts as an application-specific translation layer between REST API calls and device-specific JSON commands.

#### API to Command Translation

**Example: Position Command**

REST API Request:
```
motors/setPos?a0=1000&a1=-1000&speedUps=100&unitsAreSteps=true
```

Gets translated to MotorControl JSON:
```json
{
  "cmd": "motion",
  "mode": "pos-abs-steps",
  "imm": 0,
  "nosplit": 1,
  "speed": "6000upm",
  "pos": [1000, -1000]
}
```

**Implementation:**

```cpp
RaftRetCode MainSysMod::apiControl(const String &reqStr, 
                                   String &respStr, 
                                   const APISourceInfo& sourceInfo)
{
    // Parse REST parameters
    RaftJson requestAsJSON = RestAPIEndpointManager::getJSONFromRESTRequest(reqStr.c_str());
    String command = requestAsJSON.getString("path[1]", "");
    
    if (command.equalsIgnoreCase("setPos"))
    {
        double axis0Pos = requestAsJSON.getDouble("params/a0", 0);
        double axis1Pos = requestAsJSON.getDouble("params/a1", 0);
        double speedUps = requestAsJSON.getDouble("params/speedUps", 100);
        bool unitsAreSteps = requestAsJSON.getBool("params/unitsAreSteps", false);
        
        // Build MotorControl JSON command
        const char* mode = unitsAreSteps ? "pos-abs-steps" : "abs";
        snprintf(jsonBuffer, sizeof(jsonBuffer),
            "{\"cmd\":\"motion\",\"mode\":\"%s\",\"speed\":\"%.2fupm\",\"pos\":[%.2f,%.2f]}",
            mode, speedUps * 60.0, axis0Pos, axis1Pos);
        
        // Send to MotorControl device
        sendToMotorControl(jsonBuffer, respStr);
    }
}
```

#### Supported REST Commands

| REST Endpoint | Description | Example |
|--------------|-------------|---------|
| `motors/setPos` | Move to absolute position | `motors/setPos?a0=100&a1=200&speedUps=50` |
| `motors/stop` | Stop all motion | `motors/stop` |
| `motors/disable` | Stop and disable motors | `motors/disable` |
| `motors/setOrigin` | Set current position as origin | `motors/setOrigin` |

### 5. Device Manager

**Location**: `test_rigs/TwoStepperMagRotTestRig/build/SysTypeMain/raft/RaftCore/components/core/DeviceManager/`

DeviceManager maintains a registry of all devices in the system and provides routing services.

#### Device Registration

Devices are configured in the system configuration:

```json
"DevMan": {
  "Devices": [{
    "class": "MotorControl",
    "name": "MotorControl",
    "bus": "SERA",
    "motion": { ... },
    "axes": [ ... ]
  }]
}
```

#### Command Routing Methods

**Method 1: Direct Device Access**
```cpp
RaftDevice* pMotorControl = deviceManager.getDevice("MotorControl");
pMotorControl->sendCmdJSON(cmdJSON);
```

**Method 2: Named Value Access**
```cpp
// Format: "DeviceName.paramName"
double xPos = deviceManager.getNamedValue("MotorControl.x", isValid);
```

**Method 3: JSON Command with Device Field**
```cpp
// Routes to device specified in "device" field
deviceManager.receiveCmdJSON("{\"device\":\"MotorControl\",\"cmd\":\"stop\"}");
```

### 6. MotorControl Device

**Location**: `components/MotorControl/`

MotorControl is a RaftDevice that provides the interface to the motion control system.

#### Command Processing

```cpp
RaftRetCode MotorControl::sendCmdJSON(const char* cmdJSON, String* respMsg)
{
    RaftJson jsonInfo(cmdJSON);
    String cmd = jsonInfo.getString("cmd", "");
    
    if (cmd.equalsIgnoreCase("motion"))
    {
        // Parse motion arguments and execute
        MotionArgs motionArgs;
        motionArgs.fromJSON(cmdJSON);
        return _motionController.moveTo(motionArgs, respMsg);
    }
    else if (cmd.equalsIgnoreCase("stop"))
    {
        // Stop motion with optional motor disable
        bool disableMotors = jsonInfo.getBool("disableMotors", false);
        _motionController.stopAll(disableMotors);
        return RAFT_OK;
    }
    else if (cmd.equalsIgnoreCase("setOrigin"))
    {
        // Set current position as origin
        _motionController.setCurPositionAsOrigin();
        return RAFT_OK;
    }
    // ... other commands
}
```

#### Supported JSON Commands

##### Motion Command
```json
{
  "cmd": "motion",
  "mode": "abs",              // abs, rel, pos-abs-steps, pos-rel-steps
  "speed": "100upm",          // Speed in units per minute
  "pos": [100, 200],          // Target positions
  "imm": 0,                   // 1 = stop and clear queue first
  "nosplit": 1                // 1 = don't split into smaller moves
}
```

##### Stop Command
```json
{
  "cmd": "stop",
  "disableMotors": true       // Optional: disable motors after stop
}
```

##### Set Origin Command
```json
{
  "cmd": "setOrigin"
}
```

##### Max Current Command
```json
{
  "cmd": "maxCurrent",
  "axisIdx": 0,
  "maxCurrentA": 1.5
}
```

##### Motor Off Timer Command
```json
{
  "cmd": "offAfter",
  "offAfterS": 10
}
```

##### Pattern Commands
```json
{
  "cmd": "startPattern",
  "pattern": "homing",
  "forMs": 5000,
  "params": { ... }
}
```

```json
{
  "cmd": "stopPattern"
}
```

### 7. Motion Controller

**Location**: `components/MotorControl/Controller/`

The MotionController handles the actual motion planning and stepper motor control through:
- **MotionPlanner**: Path planning and trajectory generation
- **RampGenerator**: Step timing and acceleration profiles
- **StepDriverBase**: Hardware driver abstraction (TMC2209, etc.)

## Message Flow Examples

### Example 1: WebSocket Position Command

1. **Client sends WebSocket message:**
   ```javascript
   websocket.send(JSON.stringify({
     cmdName: "motors/setPos",
     params: {
       a0: 1000,
       a1: -500,
       speedUps: 100
     }
   }));
   ```

2. **WebServer → ProtocolExchange:**
   - Encodes in RICSerial protocol
   - Adds frame boundaries and escaping

3. **ProtocolExchange → RestAPIEndpointManager:**
   - Decodes RICRESTMsg
   - Extracts URL: `motors/setPos?a0=1000&a1=-500&speedUps=100`

4. **RestAPIEndpointManager → MainSysMod:**
   - Matches "motors" endpoint
   - Calls `MainSysMod::apiControl()`

5. **MainSysMod → DeviceManager:**
   - Translates to: `{"cmd":"motion","mode":"abs","speed":"6000upm","pos":[1000,-500]}`
   - Calls `deviceManager.getDevice("MotorControl")->sendCmdJSON()`

6. **MotorControl → MotionController:**
   - Parses JSON to `MotionArgs`
   - Calls `_motionController.moveTo(motionArgs)`

7. **MotionController executes:**
   - Plans trajectory
   - Generates acceleration profile
   - Commands stepper drivers

### Example 2: Direct Device Command

WebSocket clients can also send commands directly using the command frame format:

```javascript
websocket.send(JSON.stringify({
  cmdName: "cmd",
  device: "MotorControl",
  cmd: "motion",
  mode: "abs",
  speed: "100upm",
  pos: [1000, -500]
}));
```

This bypasses MainSysMod and goes directly through DeviceManager to MotorControl.

## Data Flow - Telemetry and Status

Communication is bidirectional. Device status flows back to clients:

### Subscription System

Clients can subscribe to device data updates:

```javascript
// Subscribe to device JSON data at 10Hz
websocket.send(JSON.stringify({
  cmdName: "subscription",
  action: "update",
  pubRecs: [{
    name: "devjson",
    trigger: "timeorchange",
    rateHz: 10
  }]
}));
```

### Status Publishing

DeviceManager publishes device status through registered data sources:

```cpp
// JSON data source
getSysManager()->registerDataSource("Publish", "devjson", 
    [this](const char* messageName, CommsChannelMsg& msg) {
        String statusStr = getDevicesDataJSON();
        msg.setFromBuffer((uint8_t*)statusStr.c_str(), statusStr.length());
        return true;
    }
);

// Binary data source
getSysManager()->registerDataSource("Publish", "devbin", 
    [this](const char* messageName, CommsChannelMsg& msg) {
        std::vector<uint8_t> binaryData = getDevicesDataBinary();
        msg.setFromBuffer(binaryData.data(), binaryData.size());
        return true;
    }
);
```

## Configuration Reference

### Complete Configuration Hierarchy

```json
{
  "NetMan": {
    "wifiSTAEn": 1,
    "wifiAPEn": 1,
    "wifiAPSSID": "RaftAP"
  },
  "WebServer": {
    "enable": 1,
    "webServerPort": 80,
    "websockets": [{
      "pcol": "RICSerial",
      "pfix": "ws"
    }]
  },
  "ProtExchg": {
    "RICSerial": {
      "CtrlEscape": "0xD7",
      "FrameBound": "0xE7"
    }
  },
  "DevMan": {
    "Devices": [{
      "class": "MotorControl",
      "name": "MotorControl",
      "motion": { ... },
      "axes": [ ... ]
    }]
  }
}
```

## Key Design Principles

1. **Protocol Agnostic**: The same MotorControl commands work over WebSocket, Serial, MQTT, or any other transport

2. **Layered Architecture**: Each layer has a specific responsibility:
   - Transport layer (WebSocket, Serial)
   - Protocol layer (RICSerial, RICFrame)
   - Routing layer (RestAPIEndpointManager)
   - Translation layer (MainSysMod)
   - Device layer (MotorControl)

3. **Multiple Access Patterns**: Commands can reach MotorControl through:
   - REST API endpoints (via MainSysMod)
   - Direct device commands (via DeviceManager)
   - Named value access (via DeviceManager)

4. **Flexible Command Format**: JSON commands are human-readable and extensible

5. **Bidirectional Communication**: Both commands (down) and telemetry (up) use the same infrastructure

## Debugging

To enable detailed logging for WebSocket communication, configure log levels in the system JSON:

```json
{
  "WebServer": {
    "logLevel": "D"  // Debug
  },
  "DevMan": {
    "logLevel": "D"
  },
  "MainSysMod": {
    "logLevel": "D"
  }
}
```

Enable debug defines in source code:
- `DEBUG_RICREST_MESSAGES` - ProtocolExchange message details
- `DEBUG_API_CONTROL` - MainSysMod command translation
- `DEBUG_MOTOR_CMD_JSON` - MotorControl command execution

## See Also

- [MotionArgs Documentation](MotionArgs_Documentation.md) - Detailed motion command format
- [MotionArgs JSON Format Proposal](MotionArgs_JSON_Format_Proposal.md) - JSON format specification
