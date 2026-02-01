# Bounds Checking Improvements

**Date:** January 28, 2026  
**Status:** Design Document - Implementation Pending

## Problem Statement

The current bounds checking implementation has several issues:

1. **Default bounds cause false rejections**: `minUnits` defaults to 0, causing any negative coordinate to fail bounds checking even when no bounds are explicitly configured
2. **Overlapping boolean flags**: Two flags (`allowOutOfBounds` in SysType, `constrainToBounds` in MotionArgs) create confusing semantics and overlapping functionality
3. **No distinction between set/unset bounds**: The system cannot differentiate between "no bounds configured" vs "bounds set to 0"
4. **Circular path failures**: XYZ geometry rejects moves with negative coordinates, preventing circular paths centered at origin

### Current Behavior Example

**Cartesian system with default config:**
- `minUnits` = 0 (default)
- `maxUnits` = 0 (default, meaning "unlimited")
- Circle path with points like `[-10, 161]` → **REJECTED** (`MOTION_NO_MOVEMENT`)
- Only first quadrant (positive X and Y) works

## Proposed Solution

### 1. Explicit Bounds Tracking

Add flags to track whether bounds were explicitly configured:

**AxisParams.h additions:**
```cpp
private:
    bool _minUnitsSet = false;  // true if minUnits explicitly configured
    bool _maxUnitsSet = false;  // true if maxUnits explicitly configured

public:
    bool ptInBounds(const AxisPosDataType &val) const
    {
        // Only check bounds that were explicitly set
        if (_minUnitsSet && val < _minUnits)
            return false;
        if (_maxUnitsSet && val > _maxUnits)
            return false;
        return true;  // No bounds set = always valid
    }
    
    bool hasMinUnitsSet() const { return _minUnitsSet; }
    bool hasMaxUnitsSet() const { return _maxUnitsSet; }
```

**JSON Parsing:**
```cpp
void setFromJSON(const char *axisJSON)
{
    RaftJson config(axisJSON);
    // ... existing code ...
    
    // Check if bounds were explicitly set
    _minUnitsSet = config.contains("minUnits");
    _maxUnitsSet = config.contains("maxUnits");
    
    if (_minUnitsSet)
        _minUnits = config.getDouble("minUnits", 0);
    if (_maxUnitsSet)
        _maxUnits = config.getDouble("maxUnits", 0);
}
```

**Benefit:** Axes without explicit bounds are unbounded by default, allowing negative coordinates.

### 2. Out-of-Bounds Action Enumeration

Replace two boolean flags with a single, clear enumeration:

**Current (confusing):**
- `allowOutOfBounds` (SysType): bool
- `constrainToBounds` (MotionArgs): bool
- Four possible combinations with unclear semantics

**Proposed (clear):**
```cpp
enum class OutOfBoundsAction
{
    USE_DEFAULT = 0,  // For MotionArgs: use SysType default (not valid for SysType)
    DISCARD,          // Reject move if out of bounds
    CLAMP,            // Constrain/clamp coordinates to bounds
    ALLOW             // Allow out of bounds coordinates
};
```

### 3. SysType Configuration (Default Behavior)

**AxesParams.h:**
```cpp
private:
    OutOfBoundsAction _outOfBoundsDefault = OutOfBoundsAction::DISCARD;

public:
    OutOfBoundsAction getOutOfBoundsDefault() const { return _outOfBoundsDefault; }
```

**JSON Configuration (motion section):**
```json
"motion": {
  "geom": "XYZ",
  "outOfBounds": "discard",  // or "clamp" or "allow"
  "blockDistMM": 10,
  ...
}
```

**Parsing:**
```cpp
String oobStr = config.getString("motion/outOfBounds", "discard");
if (oobStr == "allow" || oobStr == "ok")
    _outOfBoundsDefault = OutOfBoundsAction::ALLOW;
else if (oobStr == "clamp" || oobStr == "constrain")
    _outOfBoundsDefault = OutOfBoundsAction::CLAMP;
else
    _outOfBoundsDefault = OutOfBoundsAction::DISCARD;
```

### 4. Per-Move Override (MotionArgs)

**MotionArgs.h:**
```cpp
private:
    OutOfBoundsAction _outOfBoundsAction = OutOfBoundsAction::USE_DEFAULT;

public:
    OutOfBoundsAction getOutOfBoundsAction() const { return _outOfBoundsAction; }
    
    // Helper to resolve with default
    OutOfBoundsAction getEffectiveOutOfBoundsAction(const AxesParams& axesParams) const
    {
        return (_outOfBoundsAction == OutOfBoundsAction::USE_DEFAULT) ? 
            axesParams.getOutOfBoundsDefault() : _outOfBoundsAction;
    }
```

**JSON Parsing (MotionArgs.cpp):**
```cpp
// Parse outOfBounds string value
String oobStr = cmdJson.getString("outOfBounds", "");
if (oobStr == "allow" || oobStr == "ok")
    _outOfBoundsAction = OutOfBoundsAction::ALLOW;
else if (oobStr == "clamp" || oobStr == "constrain")
    _outOfBoundsAction = OutOfBoundsAction::CLAMP;
else if (oobStr == "discard" || oobStr == "reject")
    _outOfBoundsAction = OutOfBoundsAction::DISCARD;
// If not specified, USE_DEFAULT (inherits from SysType)
```

**Motion Command JSON:**
```json
{"cmd":"motion", "mode":"abs", "pos":[100,200], "outOfBounds":"clamp"}
{"cmd":"motion", "mode":"abs", "pos":[100,200], "outOfBounds":"allow"}
{"cmd":"motion", "mode":"abs", "pos":[100,200]}  // Uses SysType default
```

### 5. Updated Kinematics Logic

**KinematicsXYZ.h (clean switch statement):**
```cpp
virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                          AxesValues<AxisStepsDataType>& outActuator,
                          const AxesState& curAxesState,
                          const AxesParams& axesParams,
                          const MotionArgs& args) const override final
{
    // Check machine bounds (only checks axes with explicitly set bounds)
    AxesValues<AxisPosDataType> targetPtCopy = targetPt;
    bool pointIsValid = axesParams.ptInBounds(targetPt);
    
    if (!pointIsValid)
    {
        // Bounds are set AND point is outside them
        switch (args.getEffectiveOutOfBoundsAction(axesParams))
        {
            case OutOfBoundsAction::ALLOW:
                // Allow out of bounds, no modification
#ifdef DEBUG_KINEMATICS_XYZ_BOUNDS
                LOG_I(MODULE_PREFIX, "ptToActuator: OOB allowed X=%.2f Y=%.2f", 
                      targetPt.getVal(0), targetPt.getVal(1));
#endif
                break;
                
            case OutOfBoundsAction::CLAMP:
                // Constrain to bounds
                axesParams.constrainPtToBounds(targetPtCopy);
#ifdef DEBUG_KINEMATICS_XYZ_BOUNDS
                LOG_I(MODULE_PREFIX, "ptToActuator: OOB clamped from (%.2f,%.2f) to (%.2f,%.2f)", 
                      targetPt.getVal(0), targetPt.getVal(1),
                      targetPtCopy.getVal(0), targetPtCopy.getVal(1));
#endif
                break;
                
            case OutOfBoundsAction::DISCARD:
            case OutOfBoundsAction::USE_DEFAULT:  // Should not happen after getEffective()
            default:
                // Reject the move
#ifdef WARN_KINEMATICS_XYZ_OUT_OF_BOUNDS
                LOG_W(MODULE_PREFIX, "ptToActuator FAIL out of bounds X=%.2f Y=%.2f", 
                      targetPt.getVal(0), targetPt.getVal(1));
#endif
                return false;
        }
    }

    // Perform conversion
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        float axisValFromHome = targetPtCopy.getVal(axisIdx);
        outActuator.setVal(axisIdx, round(axisValFromHome * axesParams.getStepsPerUnit(axisIdx)));
    }
    return true;
}
```

**Note:** Signature change from `bool constrainToBounds` parameter to `const MotionArgs& args` parameter to access the enum.

### 6. Geometry-Specific Constraints

Different geometries have different constraint types:

#### XYZ/Cartesian Geometries
- **Workspace limits**: Per-axis `minUnits`/`maxUnits` (rectangular bounds)
- **Coordinate system**: Cartesian X/Y/Z
- **Out-of-bounds behavior**: Controlled by `outOfBounds` setting

#### SCARA Geometries
- **Physical reachability**: `maxRadiusMM` (circular bounds) - **NOT affected by outOfBounds**
- **Joint angle limits**: Per-axis `minUnits`/`maxUnits` (theta1/theta2 limits) - controlled by `outOfBounds`
- **Coordinate system**: Joint angles (degrees)

**KinematicsSingleArmSCARA.h:**
```cpp
bool cartesianToPolar(...) const
{
    AxisCalcDataType thirdSideL3MM = sqrt(pow(targetPt.getVal(0), 2) + pow(targetPt.getVal(1), 2));

    // Physical reachability check - ALWAYS enforced (geometry physics)
    bool posValid = thirdSideL3MM <= (_arm1LenMM + _arm2LenMM) && 
                    (thirdSideL3MM >= fabs(_arm1LenMM - _arm2LenMM)) &&
                    thirdSideL3MM <= _maxRadiusMM;
    
    if (!posValid)
    {
#ifdef WARN_KINEMATICS_SA_SCARA_POS_OUT_OF_BOUNDS
        LOG_W(MODULE_PREFIX, "cartesianToPolar: physically unreachable X=%.2f Y=%.2f dist=%.2f maxRadius=%.2f", 
              targetPt.getVal(0), targetPt.getVal(1), thirdSideL3MM, _maxRadiusMM);
#endif
        return false;  // Cannot reach regardless of outOfBounds setting
    }
    
    // Joint angle calculations...
    return true;
}

// In ptToActuator, could optionally check per-axis joint limits:
// axesParams.ptInBounds() would check theta1/theta2 limits if set
// This check WOULD respect outOfBounds setting
```

**Important distinction:**
- **Physical constraints** (arm reach): Always enforced
- **Workspace/joint limits** (configured bounds): Respect `outOfBounds` setting

### 7. Logging

Add compile-time debug and warning flags following existing patterns:

**KinematicsXYZ.h:**
```cpp
// #define DEBUG_KINEMATICS_XYZ
// #define DEBUG_KINEMATICS_XYZ_BOUNDS
// #define WARN_KINEMATICS_XYZ_OUT_OF_BOUNDS
```

**AxisParams.h:**
```cpp
// #define DEBUG_AXIS_PARAMS_BOUNDS
```

**Example logging:**
```cpp
#ifdef DEBUG_KINEMATICS_XYZ_BOUNDS
    if (!_minUnitsSet && !_maxUnitsSet)
    {
        LOG_I(MODULE_PREFIX, "ptInBounds: axis%d has no bounds set, allowing %.2f", 
              axisIdx, val);
    }
#endif
```

## Implementation Order

1. **Add enum definition** (`OutOfBoundsAction`) in appropriate header (AxisParams.h or new MotionTypes.h)
2. **Update AxisParams**: Add `_minUnitsSet`/`_maxUnitsSet` flags, modify `ptInBounds()`
3. **Update AxesParams**: Replace `_allowOutOfBounds` with `_outOfBoundsDefault`, update parsing
4. **Update MotionArgs**: Replace `_constrainToBounds` with `_outOfBoundsAction`, update parsing
5. **Update kinematics classes**: Modify `ptToActuator()` signature and logic (XYZ, SCARA, others)
6. **Update MotionBlockManager**: Pass MotionArgs instead of bool to kinematics
7. **Add logging**: Debug and warning statements with #define controls
8. **Update documentation**: MotionArgs_Documentation.md, WebSocket_to_MotorControl_Communication.md
9. **Test**: Verify circle paths work with default config, verify explicit bounds work

## Benefits

✅ **No false rejections**: Unbounded axes allow any coordinate  
✅ **Clear semantics**: Three explicit behaviors vs confusing booleans  
✅ **Single source of truth**: One enum replaces two overlapping flags  
✅ **Per-move control**: Override system default on individual commands  
✅ **Geometry-appropriate**: Each kinematics handles its coordinate system  
✅ **Extensible**: Easy to add new behaviors (e.g., WARN)  
✅ **Type-safe**: Compiler catches invalid enum values  

## Testing Scenarios

### Scenario 1: Circle Path on Unbounded Cartesian
**Config:** XYZ geometry, no minUnits/maxUnits set, `outOfBounds="discard"` (default)  
**Command:** Circle path with radius 162mm centered at origin  
**Expected:** Full circle completes (including negative quadrants)  
**Reason:** No bounds set → `ptInBounds()` returns true for all points

### Scenario 2: Bounded Workspace with Clamp
**Config:** XYZ geometry, `minUnits=-100`, `maxUnits=100`, `outOfBounds="clamp"`  
**Command:** Move to [150, 150]  
**Expected:** Move clamped to [100, 100], motion succeeds  

### Scenario 3: Bounded Workspace with Discard
**Config:** XYZ geometry, `minUnits=-100`, `maxUnits=100`, `outOfBounds="discard"`  
**Command:** Move to [150, 150]  
**Expected:** Move rejected with error message

### Scenario 4: SCARA Physical Limit
**Config:** SCARA geometry, arm1=150mm, arm2=150mm, `maxRadiusMM=290`  
**Command:** Move to [300, 0] (beyond reach)  
**Expected:** Move rejected (physical constraint, ignores `outOfBounds` setting)

### Scenario 5: Per-Move Override
**Config:** XYZ geometry, bounds set, `outOfBounds="discard"` (default)  
**Command:** Move to [150, 150] with `"outOfBounds":"allow"`  
**Expected:** Move succeeds (per-move override takes precedence)

## Related Files

### Core Implementation
- `components/MotorControl/Axes/AxisParams.h`
- `components/MotorControl/Axes/AxesParams.h`
- `components/MotorControl/Controller/MotionArgs.h`
- `components/MotorControl/Controller/MotionArgs.cpp`
- `components/MotorControl/Kinematics/KinematicsXYZ.h`
- `components/MotorControl/Kinematics/KinematicsSingleArmSCARA.h`
- `components/MotorControl/Kinematics/RaftKinematics.h` (interface)
- `components/MotorControl/Controller/MotionBlockManager.cpp`

### Documentation
- `docs/MotionArgs_Documentation.md`
- `docs/WebSocket_to_MotorControl_Communication.md`
- `test_rigs/WebUI/README.md`

### Configuration Examples
- `test_rigs/TwoStepperMagRotTestRig/systypes/SysTypeMain/SysTypes.json`
- `test_rigs/TwoStepperMagRotTestRig/systypes/SysTypeMain/SysTypes - cartesian.json`
- `test_rigs/TwoStepperMagRotTestRig/systypes/SysTypeMain/SysTypes - sandbot.json`
