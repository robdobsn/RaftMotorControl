# MotionArgs Documentation

**Last Updated:** February 1, 2026  
**Version:** 2.1 - Mode-Based Motion Control with Velocity Mode
**Author:** Rob Dobson

## Overview

`MotionArgs` is a class that encapsulates all parameters needed to define a motion command in the RaftMotorControl system. It provides a simplified, mode-based interface for specifying motor movements with URL-compatible JSON formatting.

The class includes JSON serialization/deserialization capabilities and uses a mode string that indicates the type of motion being requested.

## Core Arguments

### Motion Mode (`mode`)

**Type:** String  
**Default:** `"abs"` (absolute positioning in machine units - which are generally mm but may be something different for some geometries/machines)
**JSON Field:** `"mode"`

#### Available Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `"abs"` | Absolute position in units (mm, degrees) | Standard positioning moves |
| `"rel"` | Relative position in units | Incremental movements |
| `"pos-abs-steps"` | Absolute position in motor steps | Low-level step control |
| `"pos-rel-steps"` | Relative position in motor steps | Incremental step control |
| `"pos-rel-steps-noramp"` | Relative steps, no acceleration | Direct step execution (used by homing) |
| `"vel"` | Velocity mode in units/sec | Continuous motion at specified velocity |
| `"vel-steps"` | Velocity mode in steps/sec | Continuous motion at specified step rate |
| `"prop"` | Proportionate absolute (0-1 maps to axis min-max) | Scale-independent patterns |
| `"prop-rel"` | Proportionate relative (fraction of axis range) | Scale-independent increments |

**Helper Methods:**
- `isRelative()` - Returns true if mode contains relative positioning
- `areUnitsSteps()` - Returns true if mode uses steps instead of units
- `isRamped()` - Returns true unless mode contains "noramp"
- `isVelocityMode()` - Returns true if mode is "vel" or starts with "vel-"
- `areVelocityUnitsSteps()` - Returns true if velocity mode uses steps/sec
- `isProportionate()` - Returns true if mode is "prop" or starts with "prop-"

**Examples:**
```json
{"mode": "abs"}                    // Move to absolute position in mm
{"mode": "rel"}                    // Move relative in mm
{"mode": "pos-rel-steps-noramp"}   // Direct step control for homing
{"mode": "vel", "vel": [10, 5]}    // Velocity mode: axis 0 at 10 u/s, axis 1 at 5 u/s
{"mode": "prop", "pos": [0.5, 0.5]}    // Move to center of each axis range
{"mode": "prop-rel", "pos": [0.1, 0]}  // Move axis 0 by 10% of its range
```

### Proportionate Mode Details

The proportionate modes (`prop` and `prop-rel`) allow position commands to be specified as values between 0 and 1, which are mapped to the configured axis bounds (`minUnits` and `maxUnits`). This enables:

- **Portable patterns**: A pattern designed with proportionate coordinates will work on any machine, automatically scaling to fit the working area.
- **Scale-independent control**: UI sliders or joysticks can use 0-1 values without knowing machine dimensions.
- **Bounds safety**: Values are inherently within bounds when using 0-1 range.

**Prerequisites:** Axis bounds must be configured in the system JSON:
```json
{
  "axes": [
    {
      "name": "X",
      "params": {
        "minUnits": -100,
        "maxUnits": 100,
        ...
      }
    }
  ]
}
```

**Mode: `prop` (Proportionate Absolute)**

Values are clamped to 0-1 and linearly mapped to the axis range:
- `0` → `minUnits`
- `0.5` → midpoint of range
- `1` → `maxUnits`

```json
{"mode": "prop", "pos": [0, 0]}        // Move to minimum position on both axes
{"mode": "prop", "pos": [1, 1]}        // Move to maximum position on both axes
{"mode": "prop", "pos": [0.5, 0.5]}    // Move to center of working area
```

**Mode: `prop-rel` (Proportionate Relative)**

Values represent a fraction of the axis range to move relative to current position:
- `0.1` → move 10% of the range in positive direction
- `-0.1` → move 10% of the range in negative direction

```json
{"mode": "prop-rel", "pos": [0.1, 0]}     // Move axis 0 by 10% of its range
{"mode": "prop-rel", "pos": [-0.05, 0.05]} // Move axis 0 back 5%, axis 1 forward 5%
```

**Note:** For `prop-rel`, values are not clamped, allowing the resulting position to potentially exceed bounds (standard out-of-bounds handling applies).

---

## Position Arguments

### Position Array (`pos`)

**Type:** Array of numbers (nulls allowed)  
**Default:** Empty array  
**JSON Field:** `"pos"`

Direct array format for specifying axis positions. Array index corresponds to axis number.

**Format:** `[axis0, axis1, axis2, ...]`

**Features:**
- Null values skip that axis (axis maintains current position)
- Sparse arrays are supported (using null)
- Units determined by `mode` field (units vs steps)
- Absolute or relative determined by `mode` field

**Examples:**
```json
{"pos": [100, 50]}           // Move axis 0 to 100, axis 1 to 50
{"pos": [100, 50, null, 25]} // Move axes 0, 1, 3; skip axis 2
{"pos": [10]}                // Move only axis 0
```

**Internal Storage:**
- `_axesPos` - AxesValues<AxisPosDataType> containing position values
- `_axesSpecified` - AxesValues<AxisSpecifiedDataType> tracking which axes are specified

### Velocity Array (`vel`)

**Type:** Array of numbers  
**Default:** Empty array  
**JSON Field:** `"vel"`

### Speed (`speed`)

**Type:** String or Number  
**Default:** `""` (empty, uses 100% of configured maximum)  
**JSON Field:** `"speed"`

Unified speed control field that accepts either numeric percentages or strings with embedded units. The system automatically caps speed at the configured maximum.

#### Numeric Format (Percentage)
```json
{"speed": 80}      // 80% of configured maximum speed
{"speed": 50}      // 50% of configured maximum speed
{"speed": 100}     // 100% of configured maximum speed (default)
```

#### String Format with Units
```json
{"speed": "10mmps"}    // 10 millimeters per second
{"speed": "500upm"}    // 500 units per minute
{"speed": "80pc"}      // 80 percent of maximum
{"speed": "1000sps"}   // 1000 steps per second
```

#### Supported Unit Suffixes
| Suffix | Meaning | Conversion |
|--------|---------|------------|
| `pc`, `percent` | Percentage of max | Direct percentage |
| `ups`, `unitsps` | Units per second | Direct rate |
| `upm`, `unitspm` | Units per minute | Divided by 60 |
| `mmps` | Millimeters per second | Assumes axis units are mm |
| `mmpm` | Millimeters per minute | Divided by 60 |
| `sps` | Steps per second | Direct step rate |

**Speed Calculation Method:**
```cpp
double getSpeedUps(double configMaxSpeedUps) const
```
- Parses the speed string
- Converts to units per second
- Automatically caps at `configMaxSpeedUps` (safety limit)
- Returns the final speed value

**Examples:**
```json
{"cmd":"motion", "mode":"abs", "pos":[100,50], "speed":"10mmps"}
{"cmd":"motion", "mode":"rel", "pos":[5], "speed":50}
{"cmd":"motion", "mode":"pos-rel-steps-noramp", "pos":[200], "speed":"100upm"}
```

---

## Motion Control Flags

### Don't Split Move (`nosplit`)evel control or testing

### `_unitsAreSteps`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"unitsAreSteps"` or `"steps"` (alias)
- **Getters/Setters:** `areUnitsSteps()`, `setUnitsSteps()`
**Type:** Boolean  
**Default:** `false`  
**JSON Field:** `"nosplit"`

Controls whether long moves should be split into smaller blocks.

- `false` (default): Long moves are split based on `maxBlockDistMM` parameter for smoother motion
- `true`: Move is processed as a single block regardless of distance

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[100,50], "nosplit":true}
```

### Clockwise Motion (`cw`)

**Type:** Boolean  
**Default:** `false`  
**JSON Field:** `"cw"`

Reserved for future circular/arc motion control. Specifies direction of circular moves.

### More Moves Coming (`more`)

**Type:** Boolean  
**Default:** `false`  
**JSON Field:** `"more"`

Optimization hint indicating more motion commands will follow.

- `false` (default): Last move in sequence
- `true`: More moves coming, enables lookahead optimization

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[50,25], "more":true}
{"cmd":"motion", "mode":"abs", "pos":[100,50], "more":false}
```

### Immediate Execution (`imm`)

**Type:** Boolean  
**Default:** `false`  
**JSON Field:** `"imm"`

Stops current motion, clears queue, then executes this motion command.

- `false` (default): Adds to motion queue normally
- `true`: Emergency execution - stops all motion, clears queue, then executes

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[0,0], "imm":true}  // Emergency return to origin
```

**Implementation:** Calls `_rampGenerator.stop()` and `_blockManager.clear()` before executing the new motion.

### Constrain to Bounds (`constrain`)
  - `false` (default): Normal motion processing
  - `true`: Immediately stops all motor motion
    - Emergency stop functionality
    - Halts ramp generator
    - Motion stops as quickly as mechanically possible
    - Does not add any motion blocks

### `_constrainToBounds`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"constrain"`
- **Getter:** `constrainToBounds()`
- **Usage:**
  - MotionBlockManager line 514: Passed to kinematics `ptToActuator()`
  - Kinematics implementations (XYZ, SCARA): Controls out-of-bounds handling
- **Impact:**
  - `false` (default): Out-of-bounds moves are rejected with error
    - Returns RAFT_INVALID_DATA
    - Protects mechanical limits
  - `true`: Out-of-bounds coordinates are constrained to valid workspace
    - Move is modified to stay within bounds
    - Prevents error returns
    - Useful for graceful handling of boundary conditions

---

## Speed Control Arguments

### `_targetSpeed`
- **Type:** `double` (AxisSpeedDataType)
- **Default:** `0`
- **JSON Field:** `"speed"`
- **Getters/Setters:** `getTargetSpeed()`, `setTargetSpeed()`, `isTargetSpeedValid()`
- **Usage:**
  - MotionPlanner lines 101, 223: Caps requested velocity if valid
  - Acts as absolute maximum speed limit for this move
- **Impact:**
  - When `_targetSpeedValid=false`: No speed limiting, uses axis maximum speeds
  - When `_targetSpeedValid=true`: Enforces hard cap on move velocity
    - Units are steps per second
    - Overrides feedrate calculations if lower
    - Example: If calculated speed is 1000 steps/sec but targetSpeed is 500, move uses 500
  - Use case: Enforcing speed limits for specific move types (e.g., probing, careful positioning)

### `_feedrate`
- **Type:** `double`
- **Default:** `100.0`
- **JSON Field:** `"feedrate"`
- **Getters/Setters:** `getFeedrate()`, `setFeedratePercent()`, `setFeedrateUnitsPerMin()`
- **Usage:**
  - MotionPlanner lines 105, 110, 227, 232: Scales velocity calculations
  - Interpretation depends on `_feedrateUnitsPerMin` flag
- **Impact:**
  - When `_feedrateUnitsPerMin=false` (default):
    - Value is percentage of maximum axis speed (0-100+)
**Type:** Boolean  
**Default:** `false`  
**JSON Field:** `"constrain"`

Controls out-of-bounds behavior for workspace limits.

- `false` (default): Out-of-bounds moves return error (RAFT_INVALID_DATA)
- `true`: Out-of-bounds coordinates are constrained to valid workspace

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[250,150], "constrain":true}
```

---

## Additional Arguments

### Motor Current (`motorCurrent`)

**Type:** Number  
**Default:** `0` (no override)  
**JSON Field:** `"motorCurrent"`

Sets motor current as a percentage of maximum for this move (0-100). Zero means use default configuration.

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[100,50], "motorCurrent":75}
```

### Motion Tracking Index (`idx`)
## Reserved/Unused Arguments

### `_extrudeDistance`
- **Type:** `double`
- **Default:** `0`
- **JSON Field:** `"exDist"`
- **Getters/Setters:** `getExtrudeDist()`, `setExtrudeDist()`, `isExtrudeValid()`
- **Usage:**
  - Defined and serialized but not used in motion planning code
- **Impact:**
  - Currently no impact on motor behavior
  - Reserved for future 3D printer extruder control
  - When implemented, would control extrusion amount during coordinated moves
  - Typical use: Synchronized material extrusion with XYZ movement in 3D printing

### `_ampsPercentOfMax`
- **Type:** `double`
- **Default:** `0`
- **JSON Field:** `"ampsPCofMax"`
- **Usage:**
  - Defined and serialized but not used in motion planning code
- **Impact:**
  - Currently no impact on motor behavior
  - Reserved for future dynamic current control
  - When implemented, would adjust motor current as percentage of maximum:
    - Higher current = more torque, more heat
    - Lower current = less torque, cooler operation, power savings
  - Use cases:
    - Reduce current during idle/hold
    - Increase current for heavy loads
    - Optimize power consumption

---

## Field Aliases

Some JSON fields support multiple names for compatibility:

- `"unitsAreSteps"` also accepts `"steps"`
- `"immediate"` also accepts `"stop"`

**Type:** Integer  
**Default:** `0`  
**JSON Field:** `"idx"`

Optional index for tracking motion completion. Allows external systems to correlate commands with execution.

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[100,50], "idx":1234}
```
```json
{
  "pos": [{"a": 0, "p": 10.0}],
  "rel": true,
  "feedrate": 50.0
}
```
Moves axis 0 by 10mm relative to current position at 50% speed.

### Emergency Stop
```json
{
  "immediate": true
}
```
Immediately stops all motion.
# Endstop Configuration (`endstops`)

**Type:** Object  
**Default:** No endstop checking  
**JSON Field:** `"endstops"`

Configures which endstops should be monitored during motion. Used primarily for homing operations.

**Example:**
```json
{"cmd":"motion", "mode":"abs", "pos":[-200], "endstops": {...}}
```

### Extrude Distance (`exDist`)

**Type:** Number  
**Default:** `0`  
**JSON Field:** `"exDist"`

Reserved for future 3D printer extruder control. Specifies extrusion amount during coordinated moves.rudeDistance` for 3D printing
4. **Dynamic Current**: Using `_ampsPercentOfMax` for power optimization
5. **Motion Tracking**: Enabling completion callbacks via `_motionTrackingIdx`

---

### Basic Absolute Move
```json
{
  "cmd": "motion",
  "mode": "abs",
  "pos": [100, 50],
  "speed": 80
}
```
Moves axis 0 to 100mm, axis 1 to 50mm at 80% of maximum speed.

### Relative Move with Units
```json
{
  "cmd": "motion",
  "mode": "rel",
  "pos": [10],
  "speed": "10mmps"
}
```
Moves axis 0 by 10mm relative to current position at 10mm/sec.

### Stop Command
```json
{
  "cmd": "stop"
}
```
Stops all motion immediately and clears queue.

### Stop with Motor Disable
```json
{
  "cmd": "stop",
  "disableMotors": true
}
```
Stops motion, clears queue, and disables motors.

### Non-Ramped Step Move (Homing)
```json
{
  "cmd": "motion",
  "mode": "pos-rel-steps-noramp",
  "pos": [100],
  "speed": "500upm"
}
```
Moves axis 0 by 100 steps without acceleration at 500 units/min.

### Immediate Execution
```json
{
  "cmd": "motion",
  "mode": "abs",
  "pos": [0, 0],
  "imm": true
}
```
Emergency return to origin - stops current motion, clears queue, executes immediately.

### Multi-Axis with Speed String
```json
{
  "cmd": "motion",
  "mode": "abs",
  "pos": [100, 50, null, 25],
  "speed": "15mmps",
  "constrain": true
}
```
Moves axes 0, 1, 3 (skips 2) at 15mm/sec, constrains to workspace bounds.

### Velocity Mode
```json
{
  "cmd": "motion",
  "mode": "vel",
  "vel": [10, -5]
}
```
Continuous motion: axis 0 at 10 units/sec, axis 1 at -5 units/sec.
Motion continues indefinitely until stopped or a new command is received.

### Velocity Mode (Steps)
```json
{
  "cmd": "motion",
  "mode": "vel-steps",
  "vel": [1000, 500]
}
```
Continuous motion: axis 0 at 1000 steps/sec, axis 1 at 500 steps/sec.

### Stop Velocity Mode
```json
{
  "cmd": "stop"
}
```
Stops all motion including velocity mode.
```
Continuous motion: axis 0 at 10 u/s, axis 1 at -5 u/s, axis 2 stopped