# RaftMotorControl Configuration Guide

## Overview

RaftMotorControl uses JSON-based configuration for comprehensive setup of motors, drivers, kinematics, and safety systems. Configuration is passed to the MotorControl constructor and processed during the `setup()` phase.

## Configuration Structure

### Basic Configuration Template
```json
{
  "name": "MotorControl",
  "type": "MotorControl", 
  "bus": "motorSerial",
  "axes": [
    {
      "name": "X",
      "params": {
        "maxSpeed": 1000.0,
        "maxAcc": 500.0,
        "stepsPerUnit": 80.0,
        "maxPos": 200.0,
        "minPos": -200.0
      },
      "driver": {
        "driver": "TMC2209",
        "stepPin": 14,
        "dirPin": 15,
        "addr": 0
      },
      "endStops": {
        "minPin": 18,
        "maxPin": 19,
        "minActiveLevel": 0,
        "maxActiveLevel": 0
      }
    }
  ],
  "kinematics": {
    "type": "XYZ"
  },
  "rampGenerator": {
    "timerIndex": 0,
    "queueLength": 100
  }
}
```

## Configuration Sections

### Device-Level Configuration

#### Basic Device Settings
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `name` | string | Device instance name | "MotorControl" |
| `type` | string | Device type identifier | "MotorControl" |
| `bus` | string | Serial bus name for driver communication | "" |

### Axes Configuration

The `axes` array defines individual axis parameters, drivers, and end-stops.

#### Axis Parameters (`params` section)
| Parameter | Type | Description | Default | Units |
|-----------|------|-------------|---------|-------|
| `maxSpeed` | float | Maximum axis speed | 1000.0 | units/min |
| `maxAcc` | float | Maximum acceleration | 500.0 | units/min² |
| `stepsPerUnit` | float | Steps per unit of measurement | 80.0 | steps/unit |
| `maxPos` | float | Maximum position limit | 1000.0 | units |
| `minPos` | float | Minimum position limit | -1000.0 | units |
| `homeSpeed` | float | Homing speed | 100.0 | units/min |
| `homingDir` | int | Homing direction (1 or -1) | 1 | direction |
| `homeOffsetMM` | float | Offset from home position | 0.0 | units |

#### Driver Configuration (`driver` section)
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `driver` | string | Driver type ("TMC2209") | "TMC2209" |
| `stepPin` | int | Step signal GPIO pin | 14 |
| `dirPin` | int | Direction signal GPIO pin | 15 |
| `addr` | int | Driver address (for TMC2209) | 0 |
| `maxCurrent` | float | Maximum motor current (amps) | 1.0 |
| `rSense` | float | Current sense resistor value | 0.11 |
| `invDir` | bool | Invert direction signal | false |
| `microsteps` | int | Microstepping configuration | 16 |

#### End-Stop Configuration (`endStops` section)
| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `minPin` | int | Minimum end-stop GPIO pin | -1 (disabled) |
| `maxPin` | int | Maximum end-stop GPIO pin | -1 (disabled) |
| `minActiveLevel` | int | Min end-stop active level (0/1) | 0 |
| `maxActiveLevel` | int | Max end-stop active level (0/1) | 0 |
| `pullup` | bool | Enable internal pullup | true |
| `debounceMs` | int | Debounce time in milliseconds | 10 |

### Kinematics Configuration

#### Cartesian (XYZ) Kinematics
```json
{
  "kinematics": {
    "type": "XYZ"
  }
}
```

#### SCARA Kinematics
```json
{
  "kinematics": {
    "type": "SingleArmSCARA",
    "arm1Length": 150.0,
    "arm2Length": 100.0,
    "shoulderOffset": 0.0,
    "elbowOffset": 0.0
  }
}
```

**SCARA Parameters**:
| Parameter | Type | Description | Units |
|-----------|------|-------------|-------|
| `arm1Length` | float | First arm segment length | mm |
| `arm2Length` | float | Second arm segment length | mm |
| `shoulderOffset` | float | Shoulder joint offset | degrees |
| `elbowOffset` | float | Elbow joint offset | degrees |

### Ramp Generator Configuration

Controls motion planning and hardware timing.

```json
{
  "rampGenerator": {
    "timerIndex": 0,
    "queueLength": 100,
    "allowOutOfBounds": false,
    "stopOnEndStop": true
  }
}
```

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `timerIndex` | int | Hardware timer index | 0 |
| `queueLength` | int | Motion queue buffer size | 100 |
| `allowOutOfBounds` | bool | Allow motion beyond limits | false |
| `stopOnEndStop` | bool | Stop on end-stop trigger | true |

### Motor Enabler Configuration

Controls automatic motor power management.

```json
{
  "motorEnabler": {
    "onTimeAfterMoveSecs": 10.0,
    "defaultCurrentPercent": 100
  }
}
```

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `onTimeAfterMoveSecs` | float | Time to keep motors on after movement | 10.0 |
| `defaultCurrentPercent` | int | Default current as percentage of max | 100 |

## Complete Configuration Examples

### 3-Axis CNC Machine
```json
{
  "name": "CNCController",
  "type": "MotorControl",
  "bus": "stepperBus",
  "axes": [
    {
      "name": "X",
      "params": {
        "maxSpeed": 2000.0,
        "maxAcc": 1000.0,
        "stepsPerUnit": 80.0,
        "maxPos": 300.0,
        "minPos": 0.0,
        "homeSpeed": 500.0,
        "homingDir": -1
      },
      "driver": {
        "driver": "TMC2209",
        "stepPin": 14,
        "dirPin": 15,
        "addr": 0,
        "maxCurrent": 1.2,
        "microsteps": 16
      },
      "endStops": {
        "minPin": 18,
        "maxPin": 19,
        "minActiveLevel": 0,
        "maxActiveLevel": 0
      }
    },
    {
      "name": "Y", 
      "params": {
        "maxSpeed": 2000.0,
        "maxAcc": 1000.0,
        "stepsPerUnit": 80.0,
        "maxPos": 200.0,
        "minPos": 0.0,
        "homeSpeed": 500.0,
        "homingDir": -1
      },
      "driver": {
        "driver": "TMC2209",
        "stepPin": 16,
        "dirPin": 17,
        "addr": 1,
        "maxCurrent": 1.2,
        "microsteps": 16
      },
      "endStops": {
        "minPin": 20,
        "maxPin": 21,
        "minActiveLevel": 0,
        "maxActiveLevel": 0
      }
    },
    {
      "name": "Z",
      "params": {
        "maxSpeed": 800.0,
        "maxAcc": 400.0,
        "stepsPerUnit": 400.0,
        "maxPos": 100.0,
        "minPos": 0.0,
        "homeSpeed": 200.0,
        "homingDir": 1
      },
      "driver": {
        "driver": "TMC2209",
        "stepPin": 22,
        "dirPin": 23,
        "addr": 2,
        "maxCurrent": 0.8,
        "microsteps": 16
      },
      "endStops": {
        "minPin": 24,
        "maxPin": 25,
        "minActiveLevel": 0,
        "maxActiveLevel": 0
      }
    }
  ],
  "kinematics": {
    "type": "XYZ"
  },
  "rampGenerator": {
    "timerIndex": 0,
    "queueLength": 200,
    "allowOutOfBounds": false,
    "stopOnEndStop": true
  },
  "motorEnabler": {
    "onTimeAfterMoveSecs": 30.0,
    "defaultCurrentPercent": 80
  }
}
```

### SCARA Robot Arm
```json
{
  "name": "SCARABot",
  "type": "MotorControl",
  "bus": "robotSerial",
  "axes": [
    {
      "name": "Shoulder",
      "params": {
        "maxSpeed": 360.0,
        "maxAcc": 180.0,
        "stepsPerUnit": 8.888,
        "maxPos": 180.0,
        "minPos": -180.0,
        "homeSpeed": 90.0,
        "homingDir": 1
      },
      "driver": {
        "driver": "TMC2209",
        "stepPin": 14,
        "dirPin": 15,
        "addr": 0,
        "maxCurrent": 2.0,
        "microsteps": 32
      },
      "endStops": {
        "minPin": 18,
        "minActiveLevel": 0
      }
    },
    {
      "name": "Elbow",
      "params": {
        "maxSpeed": 360.0,
        "maxAcc": 180.0,
        "stepsPerUnit": 8.888,
        "maxPos": 180.0,
        "minPos": -180.0,
        "homeSpeed": 90.0,
        "homingDir": 1
      },
      "driver": {
        "driver": "TMC2209",
        "stepPin": 16,
        "dirPin": 17,
        "addr": 1,
        "maxCurrent": 1.5,
        "microsteps": 32
      },
      "endStops": {
        "minPin": 20,
        "minActiveLevel": 0
      }
    }
  ],
  "kinematics": {
    "type": "SingleArmSCARA",
    "arm1Length": 150.0,
    "arm2Length": 100.0,
    "shoulderOffset": 0.0,
    "elbowOffset": 0.0
  },
  "rampGenerator": {
    "timerIndex": 0,
    "queueLength": 50,
    "allowOutOfBounds": false,
    "stopOnEndStop": true
  },
  "motorEnabler": {
    "onTimeAfterMoveSecs": 5.0,
    "defaultCurrentPercent": 70
  }
}
```

## Configuration Validation

### Automatic Validation
The system performs automatic validation during setup:
- **Parameter ranges**: Values checked against acceptable limits
- **Hardware availability**: GPIO pins and timer availability verified
- **Driver compatibility**: Driver configuration validated against hardware
- **Kinematics consistency**: Axis count matches kinematics requirements

### Common Configuration Errors

#### GPIO Pin Conflicts
```json
// ERROR: Same pin used for multiple functions
{
  "stepPin": 14,  // Axis 0
  "dirPin": 14    // Same pin - CONFLICT
}
```

#### Invalid Parameter Ranges
```json
// ERROR: Invalid microstep setting
{
  "microsteps": 17  // Must be power of 2: 1, 2, 4, 8, 16, 32, 64, etc.
}
```

#### Missing Required Parameters
```json
// ERROR: Missing step pin
{
  "driver": {
    "driver": "TMC2209",
    "dirPin": 15
    // stepPin is required
  }
}
```

## Configuration Updates

### Runtime Configuration Changes
Some parameters can be updated at runtime via JSON commands:

#### Motor Current
```json
{"cmd": "maxCurrent", "axisIdx": 0, "maxCurrentA": 1.5}
```

#### Motor Power Management  
```json
{"cmd": "offAfter", "offAfterS": 15.0}
```

### Configuration Persistence
- Configuration stored in device flash memory
- Automatic reload on device restart
- Factory reset capability available

## Best Practices

### Performance Optimization
1. **Queue sizing**: Larger queues for continuous operation, smaller for responsiveness
2. **Acceleration tuning**: Conservative acceleration prevents missed steps
3. **Current setting**: Match motor specifications for optimal performance

### Safety Configuration
1. **End-stop setup**: Always configure end-stops for automated systems
2. **Position limits**: Set conservative position limits
3. **Emergency stops**: Configure reliable emergency stop mechanisms

### Debugging Configuration
1. **Start simple**: Begin with basic configuration, add features incrementally
2. **Monitor diagnostics**: Use debug output to verify configuration
3. **Test individually**: Test each axis independently before combined operation

This configuration system provides comprehensive control over all aspects of motor control while maintaining safety and performance optimization.