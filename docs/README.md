# RaftMotorControl Documentation

Comprehensive documentation for the RaftMotorControl stepper motor control system within the Raft ESP32 framework.

## Documentation Overview

### 📖 [Overview](overview.md)
High-level system architecture and feature overview, including:
- Core components and subsystems
- Key features and capabilities
- Hardware integration points
- Framework integration

### 🔧 [API Reference](api-reference.md)
Complete API documentation for controlling motors:
- JSON command interface (`sendCmdJSON`)
- Data query methods (`getNamedValue`, `getDataJSON`)
- Supported commands and parameters
- Response formats and error handling

### 🏗️ [Architecture](architecture.md)
Detailed system architecture documentation:
- Layered architecture design
- Subsystem interactions
- Data flow patterns
- Extensibility framework

### ⚙️ [Configuration](configuration.md)
Complete configuration guide:
- JSON configuration format
- Axis, driver, and kinematics setup
- Hardware configuration examples
- Best practices and troubleshooting

## Quick Start

### Basic Usage Example

1. **Configure the motor system** via JSON:
```json
{
  "name": "MotorControl",
  "type": "MotorControl",
  "axes": [
    {
      "name": "X",
      "params": {"maxSpeed": 1000.0, "stepsPerUnit": 80.0},
      "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15}
    }
  ],
  "kinematics": {"type": "XYZ"}
}
```

2. **Send motion commands** via JSON:
```json
{"cmd":"motion","rel":0,"pos":[{"a":0,"p":100}]}
```

3. **Query system status**:
```cpp
double position = motorControl.getNamedValue("0pos", isFresh);
bool busy = motorControl.getNamedValue("b", isFresh);
```

## System Capabilities

### ✅ Supported Features
- **Stepper Motor Control**: TMC2209 drivers with current control
- **Multi-Axis Coordination**: Up to 5 synchronized axes
- **Kinematics Systems**: Cartesian (XYZ) and SCARA robot kinematics
- **Motion Planning**: Ramped motion with acceleration/deceleration
- **Safety Systems**: End-stop monitoring and boundary checking
- **Real-time Operation**: Hardware timer-based step generation
- **Pattern Execution**: Automated motion patterns (homing, etc.)
- **JSON Interface**: Complete JSON command and query API

### 🔄 Motion Patterns
- **Homing**: Automated axis homing sequence
- **Extensible**: Framework for custom motion patterns

### 🛡️ Safety Features
- **End-stop Protection**: Automatic motion stopping
- **Boundary Checking**: Configurable position limits
- **Emergency Stop**: Immediate motion termination
- **Motor Power Management**: Automatic motor disable

## Integration Guide

### Raft Framework Integration

RaftMotorControl integrates seamlessly with the Raft framework:

```cpp
// Create motor control device
MotorControl* pMotorControl = new MotorControl("MotorControl", configJson);

// Register with device manager
deviceManager.addDevice(pMotorControl);

// Use via device interface
pMotorControl->sendCmdJSON("{\"cmd\":\"motion\",\"pos\":[{\"a\":0,\"p\":100}]}");
```

### Communication Interfaces

- **JSON Commands**: Primary control interface
- **REST API**: Via Raft web server integration
- **WebSocket**: Real-time command and status updates
- **Serial**: Direct serial command interface
- **MQTT**: Publish/subscribe integration

## Hardware Requirements

### Supported Platforms
- **ESP32**: Primary target platform
- **ESP32-S3**: Full compatibility
- **ESP32-C3**: Supported with limitations

### Driver Support
- **TMC2209**: Primary stepper driver (current control, microstepping)
- **Extensible**: Framework for additional driver types

### Peripheral Requirements
- **Hardware Timers**: For precise step timing
- **GPIO Pins**: Step, direction, and end-stop signals
- **Serial Bus**: Optional for advanced driver features

## Examples and Applications

### Example Applications
- **CNC Machines**: 3-axis milling and routing
- **3D Printers**: Multi-axis additive manufacturing
- **Robot Arms**: SCARA and cartesian robot control
- **Positioning Systems**: Precise automated positioning

### Example Projects
See the `examples/` directory for:
- **DriverStatus**: Driver monitoring and diagnostics
- **Unit Tests**: Comprehensive test suite

## Development and Testing

### Unit Tests
Comprehensive test suite available in `unit_tests/` and `linux_unit_tests/`:
- **Motion planning tests**: Verify trajectory generation
- **Driver tests**: Hardware abstraction testing
- **Kinematics tests**: Coordinate transformation validation
- **Ramp generator tests**: Timing and performance validation

### Debugging Tools
- **Debug JSON output**: Comprehensive system diagnostics
- **Performance monitoring**: Real-time performance statistics
- **Logging integration**: Detailed operation logging

## Contributing

### Extension Points
- **Kinematics**: Add new coordinate systems
- **Drivers**: Support additional stepper drivers
- **Patterns**: Create custom motion patterns
- **Safety**: Add new safety monitoring features

### Code Organization
- **Modular design**: Clean separation of concerns
- **Interface-based**: Abstract interfaces for extensibility
- **Configuration-driven**: JSON-based configuration system
- **Test coverage**: Comprehensive automated testing

## Support and Resources

### Documentation Structure
```
docs/
├── README.md          # This file - documentation index
├── overview.md        # System overview and features
├── api-reference.md   # Complete API documentation
├── architecture.md    # System architecture details
└── configuration.md   # Configuration guide
```

### Source Code Organization
```
components/MotorControl/
├── MotorControl.{h,cpp}     # Main device interface
├── Controller/              # Motion control engine
├── RampGenerator/          # Hardware timing and acceleration
├── Kinematics/             # Coordinate system support
├── Steppers/               # Driver abstraction layer
├── EndStops/               # Safety monitoring
└── MotionPatterns/         # Automated motion sequences
```

For specific implementation details, refer to the individual documentation files or explore the source code with the architectural guidance provided in this documentation suite.