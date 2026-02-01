# SCARA Coordinate System Analysis

## Summary
The Python kinematics script confirms that **the firmware's kinematics calculations are 100% correct**. Both theta1 and theta2 are computed as absolute angles from the +X axis, and the forward/inverse kinematics produce accurate results (< 0.01mm error).

## Firmware Kinematics (VERIFIED CORRECT)

### Test Results
For position (261mm, 0mm):
- Firmware computes: θ1=330.46°, θ2=29.54°
- Python verifies: θ1=330.46°, θ2=29.54° 
- Forward kinematics check: (261.00, -0.00) - Error < 0.004mm ✓

All test positions show perfect agreement between Python and firmware.

### Coordinate Convention
- **θ1**: Absolute angle of link 1 from +X axis
- **θ2**: Absolute angle of link 2 from +X axis
- Both angles are in the range [0°, 360°)
- Forward kinematics: `x = L1*cos(θ1) + L2*cos(θ2)`, `y = L1*sin(θ1) + L2*sin(θ2)`

## The Real Problem

The issue is NOT with the kinematics math - it's with **how angles are measured and communicated**.

### Firmware Flow
1. **Step counts** → stored in motion system
2. `calculateAnglesFromSteps()` → converts steps to kinematic angles
   - Applies gear ratio (1:3)
   - Adds originTheta2OffsetDegrees (180°) to joint2
   - Result: θ1 and θ2 as absolute angles for kinematics
3. Forward kinematics → produces (x, y) position

### WebUI Flow (Current - WRONG)
1. **Sensor readings** (MT6701 for joint1, AS5600 for joint2)
2. Divide by gear ratio (3)
3. Add 180° to joint2
4. Forward kinematics

### The Mismatch
**Sensors measure physical motor shaft angles, NOT the same as step-derived angles!**

The sensors could be:
- Reading from a different zero position
- Measuring in a different direction  
- Not accounting for mechanical offsets

## What We Need to Know

To fix the WebUI, we need to understand:

1. **What do the sensors actually read?**
   - When the robot is at (261, 0), what raw angles do MT6701 and AS5600 report?
   - From the firmware logs, steps (-263, -1337) should correspond to angles (331°, 30°)
   - But what do the SENSORS read for this same physical configuration?

2. **How does firmware convert step counts to angles?**
   ```cpp
   // From KinematicsSingleArmSCARA.h line 282-289
   double theta1Degrees = wrapDegrees(stepValues.getVal(0) * 360 / stepsPerRot);
   double theta2Degrees = wrapDegrees(stepValues.getVal(1) * 360 / stepsPerRot + _originTheta2OffsetDegrees);
   ```
   - `stepsPerRot` is calculated from steps per degree and gear ratio
   - Joint 2 adds the 180° offset

3. **Do the sensors match the step-based angles?**
   - If sensors read the actual shaft position after gearing
   - And firmware computes angles from steps (also after gearing)
   - They SHOULD match... but do they?

## Hypothesis

The WebUI is trying to use sensor readings directly, but:
- The sensors may have a different zero position than the step counter
- The sensors may not be calibrated to match the kinematic frame
- There may be mechanical backlash or offset between sensor and stepper

## Solution Approaches

### Option 1: Use Step-Derived Angles (Not Sensors)
- Have firmware send the calculated θ1 and θ2 (from steps) to WebUI
- WebUI just displays these angles and does forward kinematics
- Bypasses any sensor calibration issues

### Option 2: Calibrate Sensors to Match Step-Based Angles
- Find the sensor offset at a known position (e.g., home/origin)
- Apply correction: `θ_kinematic = θ_sensor / gearRatio + offset`
- Requires careful calibration procedure

### Option 3: Debug Current Implementation
- Add logging to see what sensors actually read vs. what firmware calculates
- Compare sensor readings to step-based angles at multiple positions
- Find the transform between sensor space and kinematic space

## Recommendation

**Immediate action**: Add firmware logging to report BOTH:
1. The angles calculated from steps (what kinematics uses)
2. The raw sensor readings (what WebUI receives)

This will reveal the exact relationship and allow us to apply the correct transform in the WebUI.

## Test Data

From Python kinematics verification:

| Position (x, y) | θ1 (deg) | θ2 (deg) | FK Error (mm) |
|-----------------|----------|----------|---------------|
| (261.00, 0.00)  | 330.46   | 29.54    | < 0.004       |
| (258.94, 32.71) | 337.66   | 36.74    | < 0.005       |
| (252.80, 64.91) | 344.86   | 43.94    | < 0.004       |
| (211.15, 153.41)| 6.46     | 65.54    | < 0.006       |
| (16.39, 260.48) | 56.86    | 115.94   | < 0.008       |

All errors are sub-millimeter, confirming the kinematics math is correct.
