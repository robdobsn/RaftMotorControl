# Sensor Angle Tracking Architecture

## Overview

The WebUI now uses a dedicated `SensorAngleTracker` service to manage multi-turn angle accumulation for magnetic rotation sensors (MT6701 and AS5600). This separates the concern of angle tracking from visualization components.

## Architecture

### Previous Implementation (Removed)
- **Problem**: React component state tracked rotations locally
- **Issues**:
  - State lost on component remount or browser refresh
  - Gear factor applied before rotation tracking (incorrect for gearFactor ≠ 1)
  - Tight coupling between tracking logic and UI
  - Synchronization issues across multiple components

### New Implementation (Current)

#### `SensorAngleTracker` Service (`src/utils/SensorAngleTracker.ts`)

**Singleton service** that provides:

1. **Multi-turn Accumulation**
   - Detects 360° wraparounds (transitions across 0°/360° boundary)
   - Accumulates total motor shaft rotation
   - Tracks forward and backward rotation

2. **Gear Factor Handling**
   - Applies gear ratio AFTER accumulation (correct order)
   - Example: 1080° motor shaft ÷ 3.0 gear ratio = 360° joint angle

3. **State Management**
   - Persists across component re-renders
   - Separate tracker per sensor (device ID)
   - Reset capability for calibration/homing

4. **API Methods**
   ```typescript
   configure(deviceId, gearFactor)     // Set up tracker with gear ratio
   update(deviceId, rawAngle)          // Process new reading, returns joint angle
   getAccumulatedAngle(deviceId)       // Get current joint angle
   getRawAccumulatedAngle(deviceId)    // Get motor shaft angle
   getRotationCount(deviceId)          // Get number of full rotations
   reset(deviceId?)                    // Reset one or all trackers
   getDebugInfo(deviceId)              // Get detailed tracking state
   ```

#### Integration in `RobotVisualization.tsx`

1. **Configuration** (on robot config change):
   ```typescript
   angleTracker.configure('1_6', gearFactor1);   // MT6701
   angleTracker.configure('1_36', gearFactor2);  // AS5600
   ```

2. **Angle Updates** (on sensor data):
   ```typescript
   const jointAngle = angleTracker.update('1_6', rawAngle);
   // jointAngle is already multi-turn and gear-corrected
   ```

3. **User Controls**:
   - **Reset Angles** button: Resets all angle tracking to zero
   - Clears position history trail simultaneously

## Data Flow

```
Sensor (MT6701/AS5600)
    ↓ [raw angle: 0-360°]
Device Manager
    ↓
Angle Direction Inversion (if configured)
    ↓ [inverted raw angle: 0-360°]
SensorAngleTracker.update()
    ↓ [detects wraparound]
    ↓ [accumulates rotations]
    ↓ [applies gear factor]
    ↓ [joint angle: -∞ to +∞]
RobotVisualization
    ↓ [normalize for kinematics]
Forward Kinematics
    ↓
End Effector Position
```

## Key Benefits

1. **Correct Gear Math**: `(accumulated + raw) / gearFactor` instead of `accumulated / gearFactor + raw`
2. **Persistent State**: Survives component re-renders
3. **Separation of Concerns**: Tracking logic independent of UI
4. **Reusability**: Multiple components can use the same tracker
5. **Reset Capability**: User can calibrate/zero angles
6. **Debug Support**: Detailed state inspection available

## Configuration

### Device IDs
- `'1_6'`: MT6701 magnetic encoder (typically joint 1/base)
- `'1_36'`: AS5600 magnetic encoder (typically joint 2/arm)

### Gear Factors
Configured from robot's axes config:
```typescript
robotConfig.axes[0].gearFactor  // Joint 1 gear ratio
robotConfig.axes[1].gearFactor  // Joint 2 gear ratio
```

Example: `gearFactor = 3.0` means 3 motor shaft rotations = 1 joint rotation

## Future Enhancements

### When Firmware Exposes Step Counts
If firmware provides `stepsFromOrigin` or accumulated position via API:

1. Replace sensor-based tracking with firmware position queries
2. Eliminates client-side accumulation entirely
3. Always synchronized with motor control system
4. Handles homing/reset automatically

**Current Note**: Firmware does not yet publish stepper positions, so all tracking is based on magnetic sensor data.

## Debugging

Enable debug logging in `RobotVisualization.tsx`:
```typescript
const DEBUG_ANGLE_LOGGING = true;
```

Console output shows:
- Raw sensor readings (0-360°)
- Accumulated motor shaft angles
- Gear-corrected joint angles
- Gear factors
- Rotation counts
- Kinematic angles (θ1, θ2)
- End effector position

Access detailed state programmatically:
```typescript
const info = angleTracker.getDebugInfo('1_6');
console.log(info);
// {
//   accumulated: 720,        // 2 full rotations
//   prevRaw: 45.2,          // current raw angle
//   gearFactor: 3.0,        // gear ratio
//   rotations: 2,           // full rotation count
//   jointAngle: 255.07      // (720 + 45.2) / 3.0
// }
```

## Reset Behavior

Clicking "Reset Angles" button:
1. Resets all sensor accumulators to zero
2. Clears position history trail
3. Next sensor reading starts fresh accumulation
4. Use after:
   - Powering on the robot
   - Homing operations
   - Manual position adjustments
   - Lost synchronization

## Testing

To verify correct operation:

1. **Single Rotation**: Rotate joint one full turn (360°)
   - Joint angle should show ~360° (accounting for gear ratio)
   
2. **Multiple Rotations**: Rotate joint multiple full turns
   - Joint angle should accumulate correctly
   - Rotation count should increment
   
3. **Reverse Direction**: Rotate backward
   - Angle should decrease
   - Should cross zero smoothly
   
4. **Gear Factor**: With gearFactor = 3.0
   - 3 motor rotations = 1 joint rotation
   - Motor at 1080° → Joint at 360°
   
5. **Reset**: Click "Reset Angles"
   - All angles should return to current raw sensor value / gear factor
   - Position trail should clear
