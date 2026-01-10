# RaftMotorControl Architecture

## System Architecture Overview

RaftMotorControl implements a layered architecture designed for precise stepper motor control with real-time performance, safety features, and extensible kinematics support.

```
┌─────────────────────────────────────────────────────────────┐
│                    Raft Framework                          │
├─────────────────────────────────────────────────────────────┤
│  MotorControl (RaftDevice)                                  │
│  ├─ JSON Command Interface (sendCmdJSON)                    │
│  ├─ Data Query Interface (getNamedValue, getDataJSON)       │
│  └─ Device Lifecycle (setup, loop)                         │
├─────────────────────────────────────────────────────────────┤
│  MotionController                                          │
│  ├─ Motion Planning & Coordination                         │
│  ├─ Pattern Management                                      │
│  ├─ Safety & End-Stop Monitoring                          │
│  └─ Driver & Hardware Coordination                        │
├─────────────────────────────────────────────────────────────┤
│  Subsystems                                               │
│  ├─ RampGenerator     ├─ Kinematics      ├─ EndStops      │
│  ├─ MotionPlanner     ├─ StepDrivers     ├─ MotorEnabler  │
│  └─ BlockManager      └─ PatternManager  └─ AxesParams    │
├─────────────────────────────────────────────────────────────┤
│  Hardware Layer                                           │
│  ├─ TMC2209 Drivers   ├─ End-Stop Sensors                 │
│  ├─ ESP32 Timers      ├─ Serial Communication             │
│  └─ GPIO Control      └─ Hardware Interrupts               │
└─────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. MotorControl Layer (Device Interface)

**File**: `components/MotorControl/MotorControl.{h,cpp}`

**Responsibilities**:
- Raft framework integration (`RaftDevice` inheritance)
- JSON command parsing and routing
- Data formatting and response generation
- Device lifecycle management
- Serial bus configuration

**Key Methods**:
```cpp
void setup() override                              // Device initialization
void loop() override                               // Main execution loop
RaftRetCode sendCmdJSON(const char* jsonCmd)       // Command interface
double getNamedValue(const char* param, bool& isFresh) // Query interface
String getDataJSON(RaftDeviceJSONLevel level)      // Status interface
```

### 2. MotionController Layer (Control Engine)

**File**: `components/MotorControl/Controller/MotionController.{h,cpp}`

**Responsibilities**:
- Motion planning and coordination
- Multi-axis synchronization
- Safety system integration
- Hardware driver management
- Pattern execution

**Key Features**:
- **Thread-safe operation**: Handles concurrent access from multiple contexts
- **Real-time constraints**: Maintains precise timing through hardware integration
- **Safety monitoring**: Continuous end-stop and boundary checking
- **Queue management**: Buffered motion commands for smooth operation

## Subsystem Architecture

### Motion Planning Subsystem

```
MotionArgs → MotionPlanner → MotionBlockManager → RampGenerator
    ↓             ↓               ↓                    ↓
  Command     Path Planning   Queue Management   Hardware Timing
```

#### MotionPlanner (`Controller/MotionPlanner.{h,cpp}`)
- **Path optimization**: Multi-axis trajectory calculation
- **Coordinate transformation**: Integration with kinematics systems
- **Speed profiling**: Velocity and acceleration planning

#### MotionBlockManager (`Controller/MotionBlockManager.{h,cpp}`)
- **Command queuing**: Buffered motion command management
- **Block segmentation**: Large moves split into manageable segments
- **Pipeline optimization**: Overlapped planning and execution

#### RampGenerator (`RampGenerator/RampGenerator.{h,cpp}`)
- **Acceleration profiles**: Smooth acceleration and deceleration
- **Hardware timing**: Direct hardware timer integration
- **Step pulse generation**: Precise timing for stepper motors

### Kinematics Subsystem

```
           RaftKinematics (Interface)
                    ↓
        ┌─────────────────────┬─────────────────────┐
        ↓                     ↓                     ↓
  KinematicsXYZ      KinematicsSingleArmSCARA   [Extensible]
```

**Design Pattern**: Strategy pattern for coordinate system abstraction

#### Supported Coordinate Systems
1. **Cartesian (XYZ)**: Standard 3D Cartesian coordinates
2. **Single-Arm SCARA**: Articulated robot arm kinematics
3. **Extensible Framework**: Interface for custom coordinate systems

#### Transformation Flow
```
User Coordinates → Forward Kinematics → Motor Coordinates → Step Counts
Step Counts → Reverse Kinematics → Motor Coordinates → User Coordinates
```

### Hardware Abstraction Layer

#### Stepper Driver Architecture
```
        StepDriverBase (Abstract Interface)
                    ↓
        ┌─────────────────────┬─────────────────────┐
        ↓                     ↓                     ↓
  StepDriverTMC2209      [Other Drivers]      [Extensible]
```

**Features**:
- **Current control**: Configurable motor current limits
- **Microstepping**: High-resolution positioning
- **Communication**: Serial bus integration for advanced features
- **Diagnostics**: Driver status monitoring and error reporting

#### Timer Integration (`RampGenerator/RampGenTimer.{h,cpp}`)
- **Hardware timers**: ESP32 hardware timer utilization
- **Interrupt-driven**: Precise step timing in interrupt context
- **Multi-axis coordination**: Synchronized multi-motor control
- **Performance monitoring**: Real-time performance statistics

### Safety and Monitoring

#### EndStops System (`EndStops/EndStops.{h,cpp}`)
- **Collision prevention**: Automatic motion stopping
- **Configurable sensors**: Support for various end-stop types
- **Real-time monitoring**: Interrupt-driven sensor checking
- **Per-axis configuration**: Independent end-stop setup per axis

#### MotorEnabler (`MotorEnabler/MotorEnabler.h`)
- **Power management**: Automatic motor disable after movement
- **Thermal protection**: Prevents motor overheating
- **Energy efficiency**: Reduces power consumption when idle

## Data Flow Architecture

### Command Processing Flow
```
JSON Command → MotorControl → MotionController → Subsystems → Hardware
      ↓              ↓              ↓              ↓            ↓
   Parsing      Validation     Planning      Execution    Step Pulses
```

### Feedback and Monitoring Flow
```
Hardware Sensors → EndStops → MotionController → MotorControl → JSON Response
      ↓              ↓             ↓                ↓             ↓
  Sensor States   Safety Check   State Update   Data Format   User Interface
```

### Real-time Data Pipeline
```
Position Tracking: Hardware Steps → RampGenerator → MotionController → User Query
Status Monitoring: Driver State → MotionController → Device Status → JSON Output
```

## Memory Management

### Buffer Architecture
- **Motion queue**: Ring buffer for pending motion commands
- **Step buffer**: Hardware-optimized step pulse buffering  
- **Minimal allocation**: Avoid dynamic allocation in real-time paths
- **Stack efficiency**: Optimized for ESP32 memory constraints

### Performance Optimization
- **Zero-copy operations**: Direct hardware buffer access where possible
- **Compile-time configuration**: Template-based optimization
- **Interrupt efficiency**: Minimal processing in interrupt context

## Extensibility Framework

### Plugin Architecture
```cpp
// Kinematics extension example
class CustomKinematics : public RaftKinematics {
    bool forward(AxesValues<AxisPosDataType>& coords) override;
    bool reverse(AxesValues<AxisPosDataType>& coords) override;
};
```

### Driver Extension
```cpp
// Driver extension example  
class CustomDriver : public StepDriverBase {
    void step(bool direction) override;
    void setEnabled(bool enabled) override;
    // Custom driver implementation
};
```

### Pattern Extension
```cpp
// Motion pattern extension example
class CustomPattern : public MotionPatternBase {
    void service() override;
    static MotionPatternBase* create();
};
```

## Thread Safety and Concurrency

### Threading Model
- **Main thread**: Command processing, configuration, non-time-critical operations
- **Interrupt context**: Step pulse generation, end-stop monitoring
- **Background tasks**: Status monitoring, diagnostics

### Synchronization Strategy
- **Atomic operations**: Lock-free data structures for performance-critical paths
- **Critical sections**: Protected shared state access
- **Queue-based communication**: Thread-safe command passing

## Error Handling and Recovery

### Error Categories
1. **Configuration errors**: Invalid parameters, missing hardware
2. **Runtime errors**: End-stop triggers, communication failures
3. **Safety errors**: Boundary violations, emergency conditions

### Recovery Mechanisms
- **Graceful degradation**: Fallback to safe operational modes
- **State preservation**: Maintain position tracking through errors
- **Diagnostic reporting**: Comprehensive error logging and status reporting

## Performance Characteristics

### Timing Requirements
- **Step precision**: Microsecond-level timing accuracy
- **Command latency**: Sub-millisecond command processing
- **Queue depth**: Configurable buffering for continuous operation

### Scalability
- **Axis support**: Up to 5 axes with current implementation
- **Step rates**: Up to several kHz per axis (hardware dependent)
- **Memory scaling**: Linear scaling with axis count and queue depth

This architecture provides a robust foundation for precise motion control while maintaining flexibility for diverse applications and hardware configurations.