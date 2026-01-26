# MotionArgs Documentation

**Last Updated:** January 26, 2026  
**Author:** Analysis of RaftMotorControl codebase

## Overview

`MotionArgs` is a packed structure class that encapsulates all parameters needed to define a motion command in the RaftMotorControl system. It serves as the primary interface for specifying how motors should move, with support for both absolute and relative positioning, various speed control methods, and multiple motion control flags.

The class is designed for binary communication (packed structure) and includes JSON serialization/deserialization capabilities for integration with higher-level control systems.

---

## Structure Version

### `_motionArgsStructVersion`
- **Type:** `uint8_t`
- **Default:** `MULTISTEPPER_MOTION_ARGS_BINARY_FORMAT_1` (0)
- **JSON Field:** N/A (internal versioning)
- **Usage:** Version tracking for binary communication format
- **Impact:** Ensures compatibility when structure is transmitted in binary format

---

## Motion Coordinate Arguments

### `_axesPos`
- **Type:** `AxesValues<AxisPosDataType>`
- **Default:** Cleared (0 for all axes)
- **JSON Field:** `"pos"` (array of `{"a": axisIdx, "p": position}`)
- **Getters/Setters:** `getAxesPos()`, `getAxesPosConst()`, `setAxesPositions()`
- **Usage:** 
  - Primary position target for motion commands
  - Interpreted differently based on `_isRelative` and `_unitsAreSteps` flags
  - Used by kinematics system to convert to actuator coordinates
  - In MotionPlanner: Determines step counts for each axis
  - In MotionBlockManager: Used for coordinate transformation via kinematics
- **Impact:** 
  - Core argument that defines where the motion should go
  - When `_unitsAreSteps=false`: Position in axis units (mm, degrees, etc.)
  - When `_unitsAreSteps=true`: Position in motor steps
  - When `_isRelative=true`: Added to current position
  - When `_isRelative=false`: Absolute target position

### `_axesSpecified`
- **Type:** `AxesValues<AxisSpecifiedDataType>`
- **Default:** All false
- **JSON Field:** Derived from `"pos"` array (implicit)
- **Getter:** `getAxesSpecified()`
- **Usage:**
  - Tracks which axes have been explicitly specified in the command
  - Used by MotionPlanner to determine which axes participate in motion
  - Set automatically when positions are provided
- **Impact:**
  - Only specified axes are moved; unspecified axes maintain their position
  - Critical for multi-axis systems where not all axes move every command

---

## Motion Control Flags

### `_isRelative`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"rel"`
- **Getters/Setters:** `isRelative()`, `setRelative()`
- **Usage:**
  - MotionPlanner: Determines if positions are added to current position or used as absolute targets
  - RaftKinematics: Converts relative positions to absolute before kinematic transformation
- **Impact:**
  - `false` (default): Positions are absolute coordinates in the workspace
  - `true`: Positions are relative offsets from current position
  - Example: With current position at (10, 20), command with pos=(5, 5):
    - Relative: Moves to (15, 25)
    - Absolute: Moves to (5, 5)

### `_rampedMotion`
- **Type:** `bool`
- **Default:** `true`
- **JSON Field:** `"ramped"`
- **Getters/Setters:** `isRamped()`, `setRamped()`
- **Usage:**
  - MotionController: Routes to either `moveToRamped()` or non-ramped block addition
  - Determines motion planning algorithm
- **Impact:**
  - `true` (default): Uses velocity ramping with acceleration/deceleration
    - Smooth motion with trapezoidal velocity profile
    - Position specified in axis units (mm, degrees)
    - Suitable for normal operations
  - `false`: Direct step-by-step motion without acceleration
    - Instant start/stop
    - Position must be specified in steps (`_unitsAreSteps` should be true)
    - Used for low-level control or testing

### `_unitsAreSteps`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"unitsAreSteps"` or `"steps"` (alias)
- **Getters/Setters:** `areUnitsSteps()`, `setUnitsSteps()`
- **Usage:**
  - MotionPlanner: Interprets position values as steps vs. axis units
- **Impact:**
  - `false` (default): Positions in axis-defined units (mm, degrees, etc.)
  - `true`: Positions are raw motor step counts
  - Typically used with `_rampedMotion=false` for direct stepper control

### `_dontSplitMove`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"nosplit"`
- **Getters/Setters:** `dontSplitMove()`, `setDoNotSplitMove()`
- **Usage:**
  - MotionController: Checked when determining if long moves should be split
  - Line 283: `if (maxBlockDistMM > 0.01f && !args.dontSplitMove())`
- **Impact:**
  - `false` (default): Long moves are split into smaller blocks based on `maxBlockDistMM` parameter
    - Allows for smoother curves through multiple waypoints
    - Improves pipeline processing
    - Prevents buffer overflow on very long moves
  - `true`: Move is processed as a single block regardless of distance
    - Useful for moves that must not be interrupted
    - May be required for certain motion patterns

### `_moveClockwise`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"cw"`
- **Getters/Setters:** `isMoveClockwise()`, `setClockwise()`
- **Usage:**
  - Defined in structure but not currently used in motion execution code
  - Reserved for circular/arc motion control
- **Impact:**
  - Currently no impact on motor behavior
  - Intended for future arc interpolation features
  - When implemented, would control direction of circular moves (CW vs CCW)

### `_moveRapid`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"rapid"`
- **Getters/Setters:** `isMoveRapid()`, `setMoveRapid()`
- **Usage:**
  - Defined in structure but not currently used in motion execution code
  - Traditionally indicates a G0 rapid positioning move (vs G1 linear move)
- **Impact:**
  - Currently no impact on motor behavior
  - Reserved for future implementation where rapid moves might:
    - Use different acceleration profiles
    - Skip certain safety checks
    - Have higher priority in motion queue

### `_moreMovesComing`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"more"`
- **Getters/Setters:** `getMoreMovesComing()`, `setMoreMovesComing()`
- **Usage:**
  - MotionPlanner line 213: `block._blockIsFollowed = args.getMoreMovesComing()`
  - MotionPlanner line 379: Resets debug log counter when false
  - Set automatically by block splitter for intermediate blocks
- **Impact:**
  - Optimization hint for motion pipeline processing
  - `true`: System expects more motion commands, keeps motors enabled, may defer certain calculations
  - `false`: Last move in sequence, allows pipeline to optimize for stopping
  - Critical for smooth continuous motion through multiple waypoints
  - Affects lookahead optimization in motion planning

### `_isHoming`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"homing"`
- **Getter:** No public getter (set only via JSON)
- **Usage:**
  - Defined in structure but not directly checked in motion execution
  - Likely intended for future homing sequence identification
- **Impact:**
  - Currently no direct impact on motor behavior
  - Reserved for marking moves as part of homing sequence
  - Could be used to:
    - Bypass certain safety checks during homing
    - Enable special endstop handling
    - Track homing completion status

### `_enableMotors`
- **Type:** `bool`
- **Default:** `true` (motors enabled)
- **JSON Field:** `"en"`
- **Getter:** `isEnableMotors()`
- **Usage:**
  - MotionController line 199: Checked to disable motors
  - When false, motors are explicitly disabled and command returns without motion
- **Impact:**
  - `true` (default): Normal operation, motors remain energized
  - `false`: Disables motor power, allowing free movement
    - Used to conserve power
    - Allows manual positioning
    - Command completes immediately without adding motion blocks

### `_preClearMotionQueue`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"clearQ"`
- **Getter:** `isClearQueue()`
- **Usage:**
  - MotionController line 193: `if (args.isClearQueue()) { _blockManager.clear(); }`
- **Impact:**
  - `false` (default): New motion is added to existing queue
  - `true`: Clears all pending motion blocks before processing this command
    - Emergency queue clearing
    - Switching between different motion sequences
    - Cancel pending operations

### `_stopMotion`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"immediate"` or `"stop"` (alias)
- **Getter:** `isStopMotion()`
- **Usage:**
  - MotionController line 187: `if (args.isStopMotion()) { _rampGenerator.stop(); }`
- **Impact:**
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
    - 100 = full speed, 50 = half speed, 200 = double speed (if hardware allows)
    - Formula: `requestedVelocity *= (feedrate / 100.0)`
  - When `_feedrateUnitsPerMin=true`:
    - Value is absolute speed in units per minute
    - Formula: `ratio = feedrate / 60.0 / masterAxisMaxSpeed`
    - Used for G-code compatibility (F parameter)
  - Primary speed control mechanism for normal operations

### `_feedrateUnitsPerMin`
- **Type:** `bool`
- **Default:** `false`
- **JSON Field:** `"feedPerMin"`
- **Getter:** `isFeedrateUnitsPerMin()`
- **Usage:**
  - MotionPlanner lines 106, 228: Switches feedrate interpretation
  - Set by `setFeedrateUnitsPerMin()` vs `setFeedratePercent()`
- **Impact:**
  - `false` (default): `_feedrate` is percentage (0-100+)
  - `true`: `_feedrate` is units/minute (absolute speed)
  - Critical for G-code compatibility where F parameter is in mm/min or degrees/min

---

## Motion Tracking Arguments

### `_motionTrackingIdx`
- **Type:** `uint32_t`
- **Default:** `0`
- **JSON Field:** `"idx"`
- **Getters/Setters:** `getMotionTrackingIndex()`, `setMotionTrackingIndex()`, `isMotionTrackingIndexValid()`
- **Usage:**
  - MotionPlanner lines 97, 219: Copied to MotionBlock
  - MotionBlock stores and tracks the index through execution
  - RampGenerator line 462-463: (Commented out) Would update last completed command index
- **Impact:**
  - When `_motionTrackingIndexValid=false`: No tracking
  - When `_motionTrackingIndexValid=true`: Index is propagated through motion system
    - Allows external systems to track which commands have completed
    - Useful for synchronized operations
    - Can correlate commands sent with execution completion
  - Currently framework is in place but completion reporting is disabled in code

---

## Endstop Configuration

### `_endstops`
- **Type:** `AxisEndstopChecks`
- **Default:** All cleared (no endstop checking)
- **JSON Field:** `"endstops"`
- **Getters/Setters:** `getEndstopCheck()`, `setEndStops()`, `setTestAllEndStops()`, `setTestNoEndStops()`, `setTestEndStop()`
- **Usage:**
  - MotionPlanner lines 94, 216: Copied to MotionBlock via `setEndStopsToCheck()`
  - RampGenerator: Checks endstops during motion execution
- **Impact:**
  - Configures which endstops should be monitored during this move
  - Each axis can have up to 2 endstops (min/max)
  - Endstop check types:
    - `END_STOP_NOT_HIT`: Monitor but expect not to trigger
    - `END_STOP_HIT`: Expect endstop to trigger
    - `END_STOP_TOWARDS`: Check only when moving toward endstop
    - `END_STOP_NONE`: Don't check this endstop
  - Critical for:
    - Homing operations
    - Safety boundaries
    - Preventing collisions
  - When endstop triggers:
    - Motion can be stopped immediately
    - Homing sequence can detect home position
    - Error can be reported

---

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

---

## Usage Patterns

### Basic Absolute Move
```json
{
  "pos": [{"a": 0, "p": 100.0}, {"a": 1, "p": 50.0}],
  "feedrate": 80.0,
  "ramped": true
}
```
Moves axis 0 to 100mm, axis 1 to 50mm at 80% speed with acceleration.

### Relative Move
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

### Homing Move with Endstops
```json
{
  "pos": [{"a": 0, "p": -200.0}],
  "endstops": [...],
  "feedrate": 20.0,
  "homing": true
}
```
Moves toward home position slowly, monitoring endstops.

### Non-Ramped Step Move
```json
{
  "pos": [{"a": 0, "p": 100}],
  "ramped": false,
  "steps": true
}
```
Moves axis 0 by 100 steps without acceleration.

### Speed-Limited Precise Move
```json
{
  "pos": [{"a": 0, "p": 25.5}, {"a": 1, "p": 30.2}],
  "speed": 500.0,
  "feedrate": 100.0,
  "constrain": true
}
```
Precise move with absolute speed limit, constrained to workspace bounds.

---

## Implementation Notes

1. **Structure Packing**: The class uses `#pragma pack(push, 1)` to ensure consistent binary layout for communication.

2. **Thread Safety**: The structure is not inherently thread-safe. Calling code must manage synchronization.

3. **Validation**: Many fields have "Valid" flags (`_targetSpeedValid`, `_extrudeValid`, etc.) to distinguish between "not set" and "set to default value".

4. **Kinematics Integration**: Position arguments pass through kinematics system which converts from workspace coordinates to actuator coordinates based on the mechanical configuration.

5. **Pipeline Processing**: Multiple motion blocks can be in the pipeline simultaneously, with lookahead optimization considering `_moreMovesComing` flag.

6. **Split Block Handling**: When moves are split, the `_blockMotionArgs` is reused with modified positions while preserving other flags.

---

## Related Classes

- **MotionBlock**: Receives processed motion arguments and manages execution
- **MotionPlanner**: Interprets MotionArgs and creates optimized motion blocks
- **MotionBlockManager**: Coordinates block creation and splitting
- **RaftKinematics**: Transforms positions based on mechanical configuration
- **AxisEndstopChecks**: Detailed endstop configuration
- **AxesValues<T>**: Template container for per-axis values

---

## Future Enhancements

Based on reserved fields, potential future features include:

1. **Arc/Circular Motion**: Using `_moveClockwise` for G2/G3 commands
2. **Rapid Positioning**: Using `_moveRapid` for optimized G0 moves
3. **Extrusion Control**: Using `_extrudeDistance` for 3D printing
4. **Dynamic Current**: Using `_ampsPercentOfMax` for power optimization
5. **Motion Tracking**: Enabling completion callbacks via `_motionTrackingIdx`

---

## Revision History

- **Version 0 (MULTISTEPPER_MOTION_ARGS_BINARY_FORMAT_1)**: Initial implementation with core motion control features
