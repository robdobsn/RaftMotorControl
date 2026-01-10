# RaftMotorControl Overview

RaftMotorControl is a comprehensive stepper motor control component for the Raft ESP32 framework, providing sophisticated motion control capabilities with 3D forward and reverse kinematics, ramp generation, and fine positional control with acceleration/deceleration.

## Architecture Overview

### Core Components

#### MotorControl Class (`MotorControl.h/cpp`)
- **Inheritance**: Derives from `RaftDevice` providing standard Raft device interface
- **Location**: `components/MotorControl/MotorControl.{h,cpp}`
- **Primary Interface**: Main entry point for motor control operations
- **Key Responsibilities**:
  - Device configuration and setup
  - JSON command processing via `sendCmdJSON()`
  - Data retrieval and monitoring
  - Integration with Raft framework

#### MotionController Class (`Controller/MotionController.h/cpp`)
- **Core Engine**: Central motion control orchestrator
- **Location**: `components/MotorControl/Controller/MotionController.{h,cpp}`
- **Key Responsibilities**:
  - Motion planning and execution
  - Coordinate system transformations
  - Stepper driver management
  - End-stop handling
  - Motion pattern execution

### Key Subsystems

#### 1. Axes Management (`Axes/`)
- **AxesParams.h**: Axis configuration parameters
- **AxesState.h**: Current axis state tracking
- **AxesValues.h**: Multi-axis value containers
- **AxisParams.h**: Individual axis parameters
- **AxisUtils.h**: Utility functions for axis operations
- **AxisEndstopChecks**: End-stop validation and safety

#### 2. Motion Control (`Controller/`)
- **MotionPlanner**: Path planning and trajectory generation
- **MotionBlockManager**: Motion command queuing and management
- **MotionArgs**: Command argument structure for motion requests
- **MotionControlIF**: Abstract interface for motion control

#### 3. Ramp Generation (`RampGenerator/`)
- **RampGenerator**: Acceleration/deceleration profile generation
- **MotionBlock**: Individual motion segment representation
- **MotionPipeline**: Motion command pipeline management
- **RampGenTimer**: Hardware timer integration for precise timing
- **RampGenStats**: Performance monitoring and statistics

#### 4. Stepper Drivers (`Steppers/`)
- **StepDriverBase**: Abstract base class for stepper drivers
- **StepDriverTMC2209**: TMC2209 stepper driver implementation
- **StepDriverParams**: Driver configuration parameters

#### 5. Kinematics (`Kinematics/`)
- **RaftKinematics**: Abstract kinematics interface
- **KinematicsXYZ**: Cartesian coordinate system
- **KinematicsSingleArmSCARA**: SCARA robot kinematics
- **RaftKinematicsSystem**: Kinematics system management

#### 6. End-Stops (`EndStops/`)
- **EndStops**: End-stop sensor management and monitoring

#### 7. Motion Patterns (`MotionPatterns/`)
- **MotionPatternBase**: Abstract base for motion patterns
- **HomingPattern**: Automated homing sequence
- **MotionPatternManager**: Pattern registration and execution

## API Interface

### Primary Control Interface

The main control interface is through the `sendCmdJSON()` method in the MotorControl class, which accepts JSON-formatted commands for various operations.

### Supported JSON Commands

#### Motion Commands
```json
{
  "cmd": "motion",
  "rel": 0,           // 0=absolute, 1=relative movement
  "nosplit": 0,       // 0=allow move splitting, 1=don't split
  "speed": 10,        // target speed
  "speedOk": 1,       // 1=use speed parameter
  "pos": [            // array of axis positions
    {"a": 0, "p": 100},  // axis 0 to position 100
    {"a": 1, "p": 50}    // axis 1 to position 50
  ],
  "clearQ": 0,        // 1=clear motion queue
  "stop": 0           // 1=stop current motion
}
```

#### Motor Configuration Commands
```json
// Set maximum current for a motor
{
  "cmd": "maxCurrent",
  "axisIdx": 0,
  "maxCurrentA": 0.5
}

// Set motor auto-off time
{
  "cmd": "offAfter",
  "offAfterS": 10.0
}
```

#### Motion Pattern Commands
```json
// Start a motion pattern (e.g., homing)
{
  "cmd": "startPattern",
  "pattern": "homing",
  "forMs": 30000
}

// Stop current pattern
{
  "cmd": "stopPattern"
}
```

### Data Retrieval Interface

#### Named Value Queries
- **Position queries**: `"0pos"`, `"1pos"`, `"2pos"` (axis position)
- **End-stop queries**: `"0min"`, `"0max"`, `"1min"`, `"1max"` (end-stop states)
- **Step counts**: `"0steps"`, `"1steps"`, `"2steps"` (total steps)
- **Legacy queries**: `"x"`, `"y"`, `"z"` (Cartesian positions)
- **Status queries**: `"b"` (busy state)

#### JSON Data Output
The `getDataJSON()` method provides comprehensive system status including:
- Current axis positions
- Motion queue status
- Driver states
- End-stop conditions
- Performance statistics

## Hardware Integration

### Stepper Motor Drivers
- **Primary Support**: TMC2209 stepper drivers
- **Communication**: Serial bus integration via RaftBus system
- **Features**: Current control, microstepping, stall detection

### End-Stop Sensors
- **Types**: Min/max end-stops per axis
- **Integration**: Configurable per axis
- **Safety**: Automatic motion stopping on trigger

### Coordinate Systems
- **Cartesian (XYZ)**: Standard 3D Cartesian coordinates
- **SCARA**: Single-arm SCARA robot kinematics
- **Extensible**: Framework for additional coordinate systems

## Key Features

### 1. Motion Planning
- **Ramped Motion**: Smooth acceleration and deceleration profiles
- **Path Optimization**: Efficient multi-axis coordination
- **Queue Management**: Buffered motion commands for continuous operation

### 2. Safety Systems
- **End-Stop Integration**: Automatic collision prevention
- **Boundary Checking**: Configurable motion limits
- **Emergency Stop**: Immediate motion termination capability

### 3. Performance Optimization
- **Hardware Timers**: Precise step timing via ESP32 hardware timers
- **Pipeline Processing**: Overlapped motion planning and execution
- **Memory Management**: Efficient buffering for continuous operation

### 4. Extensibility
- **Plugin Architecture**: Modular kinematics and driver systems
- **Pattern Framework**: Extensible motion pattern system
- **Configuration**: JSON-based configuration system

## Configuration

Motor control is configured through JSON configuration passed to the MotorControl constructor. Key configuration sections include:

- **Axes Definition**: Individual axis parameters and limits
- **Driver Configuration**: Stepper driver settings and communication
- **Kinematics Setup**: Coordinate system selection and parameters
- **Motion Parameters**: Default speeds, accelerations, and limits
- **End-Stop Configuration**: Sensor types and trigger conditions

## Integration Points

### Raft Framework Integration
- **RaftDevice**: Standard device interface compliance
- **RaftBus**: Serial communication infrastructure
- **Configuration**: JSON-based configuration system
- **Logging**: Integrated logging and diagnostics

### Communication Interfaces
- **JSON Commands**: Primary control interface
- **Named Values**: Real-time status queries
- **Binary Data**: Structured data output (extensible)
- **Debug Interface**: Comprehensive debugging and diagnostics

This architecture provides a robust, extensible platform for precise stepper motor control with safety features and performance optimization suitable for CNC, 3D printing, and robotic applications.