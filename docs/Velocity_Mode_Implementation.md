# Velocity Mode Implementation

**Document Version:** 1.0  
**Date:** February 1, 2026  
**Author:** Rob Dobson  
**Status:** Design Document (Pre-Implementation)

## Executive Summary

This document details the implementation plan for adding velocity mode (`vel` and `vel-steps`) to the RaftMotorControl system. The design leverages the existing `MotionBlockManager` and `RampGenerator` block processing framework, allowing blocks to continue indefinitely until terminated by a stop, velocity change, or mode change command. This approach reuses the proven acceleration/deceleration ramping behavior for smooth velocity transitions.

---

## 1. Current Architecture Overview

### 1.1 Existing Motion Flow

The current position-based motion system follows this flow:

```
MotionArgs (JSON) 
    → MotionController::moveTo()
        → MotionBlockManager::addRampedBlock() / addNonRampedBlock()
            → MotionPlanner::moveToRamped() / moveToNonRamped()
                → MotionBlock (created with finite step count)
                    → MotionPipeline (block queue)
                        → RampGenerator::generateMotionPulses() (ISR/loop)
                            → Stepper drivers (step pulses)
```

### 1.2 Key Components

| Component | Role |
|-----------|------|
| `MotionArgs` | Encapsulates motion command parameters (mode, position, velocity, speed) |
| `MotionController` | High-level API for motion commands |
| `MotionBlockManager` | Manages block splitting and pipeline feeding |
| `MotionPlanner` | Computes acceleration profiles and creates MotionBlocks |
| `MotionBlock` | Discrete motion segment with defined step count and velocity profile |
| `MotionPipeline` | Ring buffer queue of MotionBlocks awaiting execution |
| `RampGenerator` | ISR-driven step pulse generator with acceleration handling |

### 1.3 Current Block Termination Logic

In `RampGenerator::generateMotionPulses()`, a block terminates when:

```cpp
// In handleStepMotion()
bool anyAxisMoving = false;
if (_curStepCount[axisIdxMaxSteps] < _stepsTotalAbs[axisIdxMaxSteps])
{
    // ... step the axis ...
    if (_curStepCount[axisIdxMaxSteps] < _stepsTotalAbs[axisIdxMaxSteps])
        anyAxisMoving = true;
}
// ...
if (!anyAxisMoving)
{
    endMotion(pBlock);  // Block complete, remove from pipeline
}
```

The block terminates when all axes reach their target step counts.

---

## 2. Velocity Mode Concept

### 2.1 Mode Definitions

| Mode | Description | Parameters |
|------|-------------|------------|
| `vel` | Velocity mode in machine units per second | `vel` array: velocities per axis |
| `vel-steps` | Velocity mode in steps per second | `vel` array: step rates per axis |

### 2.2 Behavioral Requirements

1. **Acceleration to Target Velocity**: Use existing ramping to smoothly accelerate from current velocity to requested velocity
2. **Indefinite Duration**: Motion continues until externally terminated
3. **Termination Conditions**:
   - Stop command (`cmd: stop`)
   - Velocity change command (new `vel` or `vel-steps` command)
   - Mode change (position command like `abs` or `rel`)
   - End-stop hit (optional, configurable)
   - Timeout (optional, configurable)
4. **Smooth Transitions**: Velocity changes should use ramped acceleration/deceleration
5. **Multi-Axis Coordination**: Each axis can have independent velocity, or velocities can be coordinated

### 2.3 JSON Command Format

```json
{
  "cmd": "motion",
  "mode": "vel",
  "vel": [10.0, -5.0],
  "speed": "10mmps"
}
```

- `vel`: Array of target velocities (units/sec or steps/sec depending on mode)
- `speed`: Optional, specifies acceleration rate (repurposed) or can be omitted for default

For stopping:
```json
{
  "cmd": "stop"
}
```

For velocity change (decelerate to new velocity):
```json
{
  "cmd": "motion",
  "mode": "vel",
  "vel": [5.0, 0.0]
}
```

---

## 3. Implementation Strategy

### 3.1 Design Philosophy: "Infinite Block" Approach

Rather than fundamentally changing the block-based architecture, implement velocity mode as a **continuous block** that:

1. Has an effectively infinite step count (or very large count that gets refreshed)
2. Uses existing acceleration logic to reach target velocity
3. Maintains steady-state velocity indefinitely
4. Can be interrupted and replaced for velocity changes

### 3.2 Two Implementation Options Considered

#### Option A: Truly Infinite Block (Recommended)
- MotionBlock has a new `_isVelocityMode` flag
- When set, `handleStepMotion()` never decrements toward zero
- Block continues stepping at steady-state velocity until replaced or stopped

**Pros:**
- Minimal changes to existing architecture
- Clean separation of velocity vs. position modes
- No pipeline management complexity

**Cons:**
- Requires modification to RampGenerator ISR code
- Special handling in block termination

#### Option B: Block Regeneration Pattern
- Create blocks with large but finite step counts
- MotionBlockManager monitors velocity blocks and regenerates them before completion
- Uses `_moreMovesComing` flag to maintain smooth motion

**Pros:**
- Fewer changes to ISR code
- Works with existing block termination

**Cons:**
- Requires active monitoring and timing-critical regeneration
- Risk of gaps in motion if regeneration is delayed
- Higher complexity in MotionBlockManager

### 3.3 Recommended Approach: Option A (Infinite Block)

This document details Option A as it provides cleaner semantics and lower runtime overhead.

---

## 4. Detailed Implementation Plan

### 4.1 Phase 1: MotionArgs Extensions

#### 4.1.1 New Mode Detection

Extend `MotionArgs` to recognize velocity modes:

```cpp
// In MotionArgs.h
bool isVelocityMode() const
{
    return _mode.startsWith("vel");
}

bool areVelocityUnitsSteps() const
{
    return _mode == "vel-steps";
}
```

**Status:** Already partially implemented in current codebase.

#### 4.1.2 Velocity Array Support

The `_velocities` member already exists in `MotionArgs`:

```cpp
// Already present in MotionArgs.h
AxesValues<AxisPosDataType> _velocities;

// Accessor methods exist:
AxesValues<AxisPosDataType>& getVelocities() { return _velocities; }
const AxesValues<AxisPosDataType>& getVelocitiesConst() const { return _velocities; }
```

#### 4.1.3 JSON Parsing Updates

Extend `MotionArgs::fromJSON()` to parse velocity array:

```cpp
void MotionArgs::fromJSON(const char* jsonStr)
{
    RaftJson json(jsonStr);
    
    // ... existing parsing ...
    
    // Parse velocity array for velocity modes
    if (isVelocityMode())
    {
        std::vector<String> velArray;
        if (json.getArrayElems("vel", velArray))
        {
            for (size_t i = 0; i < velArray.size() && i < AXIS_VALUES_MAX_AXES; i++)
            {
                if (velArray[i].length() > 0 && velArray[i] != "null")
                {
                    _velocities.setVal(i, velArray[i].toDouble());
                    _axesSpecified.setVal(i, true);
                }
            }
        }
    }
}
```

### 4.2 Phase 2: MotionBlock Extensions

#### 4.2.1 Velocity Mode Flag

Add velocity mode support to `MotionBlock`:

```cpp
// In MotionBlock.h
class MotionBlock
{
public:
    // ... existing members ...
    
    // Velocity mode support
    bool _isVelocityMode = false;
    AxesValues<AxisSpeedDataType> _targetVelocities;  // Target velocities per axis
    
    // Check if this is a velocity mode block
    bool isVelocityMode() const { return _isVelocityMode; }
    
    // Configure for velocity mode
    void configureVelocityMode(const AxesValues<AxisSpeedDataType>& targetVelocities);
    
    // ... existing members ...
};
```

#### 4.2.2 Velocity Block Preparation

Add new preparation method for velocity blocks:

```cpp
// In MotionBlock.cpp
bool MotionBlock::prepareForVelocityStepping(const AxesParams& axesParams)
{
    if (_isExecuting)
        return false;
    
    _isVelocityMode = true;
    
    // Find the axis with maximum absolute velocity (becomes the "dominant" axis)
    AxisSpeedDataType maxAbsVel = 0;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        AxisSpeedDataType absVel = fabs(_targetVelocities.getVal(axisIdx));
        if (absVel > maxAbsVel)
        {
            maxAbsVel = absVel;
            _axisIdxWithMaxSteps = axisIdx;  // Repurpose for dominant velocity axis
        }
    }
    
    if (maxAbsVel < 0.001)  // Near-zero velocity = stop
    {
        _isVelocityMode = false;
        return false;
    }
    
    // Set up step directions based on velocity signs
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        AxisSpeedDataType vel = _targetVelocities.getVal(axisIdx);
        // Direction is encoded in _stepsTotalMaybeNeg sign (1 or -1 for infinite mode)
        _stepsTotalMaybeNeg.setVal(axisIdx, (vel >= 0) ? 1 : -1);
    }
    
    // Calculate target step rate from velocity (for dominant axis)
    double stepDistMM = 1.0 / axesParams.getStepsPerUnit(_axisIdxWithMaxSteps);
    float targetStepRatePerSec = fabs(maxAbsVel / stepDistMM);
    
    // Cap at maximum
    if (targetStepRatePerSec > axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps))
        targetStepRatePerSec = axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps);
    
    // Set up acceleration profile to reach target velocity
    // Initial rate = current velocity (or min if starting from stop)
    _initialStepRatePerTTicks = _minStepRatePerTTicks;  // Will be set from current velocity
    _maxStepRatePerTTicks = uint32_t((targetStepRatePerSec * TTICKS_VALUE) / _ticksPerSec);
    _finalStepRatePerTTicks = _maxStepRatePerTTicks;  // No deceleration in steady state
    
    // Acceleration in steps per TTicks per millisecond
    float maxAccStepsPerSec2 = fabs(axesParams.getMaxAccelUps2(_axisIdxWithMaxSteps) / stepDistMM);
    _accStepsPerTTicksPerMS = uint32_t((maxAccStepsPerSec2 * TTICKS_VALUE) / _ticksPerSec / 1000);
    
    // For velocity mode, _stepsBeforeDecel is set to MAX so we never decelerate
    _stepsBeforeDecel = UINT32_MAX;
    
    _requestedSpeed = maxAbsVel;
    
    return true;
}
```

#### 4.2.3 Velocity Ratio Calculation

For coordinated multi-axis velocity, calculate step ratios:

```cpp
// In MotionBlock.cpp
void MotionBlock::calculateVelocityRatios()
{
    // The dominant axis steps at _maxStepRatePerTTicks
    // Other axes step at proportional rates based on their velocity ratios
    
    AxisSpeedDataType dominantVel = fabs(_targetVelocities.getVal(_axisIdxWithMaxSteps));
    
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        if (axisIdx == _axisIdxWithMaxSteps)
            continue;
            
        AxisSpeedDataType thisVel = fabs(_targetVelocities.getVal(axisIdx));
        if (thisVel > 0.001 && dominantVel > 0.001)
        {
            // Store ratio for Bresenham-style interpolation in RampGenerator
            // This can use the existing _curAccumulatorRelative mechanism
        }
    }
}
```

### 4.3 Phase 3: RampGenerator Modifications

#### 4.3.1 Velocity Mode Block Handling

Modify `handleStepMotion()` to handle velocity mode:

```cpp
// In RampGenerator.cpp
bool IRAM_ATTR RampGenerator::handleStepMotion(MotionBlock *pBlock)
{
    bool anyAxisMoving = false;
    int axisIdxMaxSteps = pBlock->_axisIdxWithMaxSteps;
    
    if ((axisIdxMaxSteps < 0) || (axisIdxMaxSteps >= _stepperDrivers.size()))
        return false;
    
    _curAccumulatorStep = _curAccumulatorStep - MotionBlock::TTICKS_VALUE;
    
    // Handle velocity mode - always continue stepping
    if (pBlock->isVelocityMode())
    {
        // Step the dominant axis
        if (_stepperDrivers[axisIdxMaxSteps])
            _stepperDrivers[axisIdxMaxSteps]->stepStart();
        _stats.stepStart(axisIdxMaxSteps);
        
        // Step other axes using Bresenham-style ratio calculation
        // (reuse existing _curAccumulatorRelative mechanism)
        for (uint32_t axisIdx = 0; axisIdx < _stepperDrivers.size(); axisIdx++)
        {
            if (axisIdx == axisIdxMaxSteps)
                continue;
            
            // Use velocity ratio to determine stepping frequency
            // Similar to existing relative accumulator logic
            AxisSpeedDataType dominantVel = fabs(pBlock->_targetVelocities.getVal(axisIdxMaxSteps));
            AxisSpeedDataType thisVel = fabs(pBlock->_targetVelocities.getVal(axisIdx));
            
            if (thisVel > 0.001 && dominantVel > 0.001)
            {
                // Scale accumulator threshold by velocity ratio
                uint32_t threshold = uint32_t(dominantVel / thisVel);
                _curAccumulatorRelative[axisIdx]++;
                
                if (_curAccumulatorRelative[axisIdx] >= threshold)
                {
                    _curAccumulatorRelative[axisIdx] = 0;
                    if (_stepperDrivers[axisIdx])
                        _stepperDrivers[axisIdx]->stepStart();
                    _stats.stepStart(axisIdx);
                }
            }
        }
        
        return true;  // Velocity mode always returns true (never terminates normally)
    }
    
    // ... existing position mode logic ...
}
```

#### 4.3.2 Velocity Mode Termination

Modify block termination to handle velocity mode:

```cpp
// In RampGenerator::generateMotionPulses()

// Check for velocity mode block replacement (new velocity command)
if (pBlock->isVelocityMode() && _velocityChangeRequested)
{
    // Begin deceleration to new velocity
    // Set _finalStepRatePerTTicks to new target
    // Reset _stepsBeforeDecel to start deceleration now
    _velocityChangeRequested = false;
}

// End stop handling for velocity mode
if (pBlock->isVelocityMode() && endStopHit)
{
    _endStopReached = true;
    endMotion(pBlock);
    return;
}
```

#### 4.3.3 Stop Handling for Velocity Mode

The existing `_stopPending` mechanism handles velocity mode termination:

```cpp
// In generateMotionPulses()
if (_stopPending)
{
    MotionBlock *pBlock = _motionPipeline.peekGet();
    if (pBlock && pBlock->_isExecuting)
    {
        // For velocity mode, we need controlled deceleration
        if (pBlock->isVelocityMode())
        {
            // Option 1: Immediate stop (current behavior)
            endMotion(pBlock);
            
            // Option 2: Controlled deceleration (future enhancement)
            // pBlock->_finalStepRatePerTTicks = 0;
            // pBlock->_stepsBeforeDecel = 0;  // Start deceleration now
            // return;  // Let it decelerate naturally
        }
        else
        {
            endMotion(pBlock);
        }
    }
    _stopPending = false;
    return;
}
```

### 4.4 Phase 4: MotionPlanner Extensions

#### 4.4.1 New Velocity Mode Planner Method

Add method to create velocity mode blocks:

```cpp
// In MotionPlanner.h
class MotionPlanner
{
public:
    // ... existing methods ...
    
    /// @brief Create a velocity mode motion block
    /// @param args MotionArgs containing velocity parameters
    /// @param axesState Current state of the axes
    /// @param axesParams Parameters for the axes
    /// @param motionPipeline Motion pipeline to add the block to
    /// @return RaftRetCode
    RaftRetCode moveVelocity(const MotionArgs& args,
                             AxesState& axesState,
                             const AxesParams& axesParams,
                             MotionPipelineIF& motionPipeline);
};
```

```cpp
// In MotionPlanner.cpp
RaftRetCode MotionPlanner::moveVelocity(const MotionArgs& args,
                                        AxesState& axesState,
                                        const AxesParams& axesParams,
                                        MotionPipelineIF& motionPipeline)
{
    // Check pipeline can accept
    if (!motionPipeline.canAccept())
        return RAFT_MOTION_BUSY;
    
    // Create velocity mode block
    MotionBlock block;
    block.setTimerPeriodNs(_stepGenPeriodNs);
    block._isVelocityMode = true;
    
    // Extract velocities from args
    const AxesValues<AxisPosDataType>& vels = args.getVelocitiesConst();
    
    // Convert to step velocities if needed
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        AxisSpeedDataType vel = vels.getVal(axisIdx);
        
        if (args.areVelocityUnitsSteps())
        {
            // Already in steps/sec
            block._targetVelocities.setVal(axisIdx, vel);
        }
        else
        {
            // Convert from units/sec to steps/sec
            double stepsPerUnit = axesParams.getStepsPerUnit(axisIdx);
            block._targetVelocities.setVal(axisIdx, vel * stepsPerUnit);
        }
    }
    
    // Prepare block for velocity stepping
    if (!block.prepareForVelocityStepping(axesParams))
    {
        // Zero velocity = stop command
        return RAFT_MOTION_NO_MOVEMENT;
    }
    
    block._canExecute = true;
    
    // Add to pipeline
    if (!motionPipeline.add(block))
        return RAFT_OTHER_FAILURE;
    
    return RAFT_OK;
}
```

### 4.5 Phase 5: MotionController Integration

#### 4.5.1 Mode Routing in moveTo()

```cpp
// In MotionController.cpp
RaftRetCode MotionController::moveTo(MotionArgs &args, String* respMsg)
{
    // Handle immediate execution
    if (args.isImmediateExecution())
    {
        _rampGenerator.stop();
        _blockManager.clear();
    }
    
    // Route based on motion mode
    if (args.isVelocityMode())
    {
        return moveToVelocity(args, respMsg);
    }
    else if (args.isRamped())
    {
        return moveToRamped(args, respMsg);
    }
    else
    {
        // Non-ramped (stepwise) motion
        bool success = _blockManager.addNonRampedBlock(args, _rampGenerator.getMotionPipeline());
        return success ? RAFT_OK : RAFT_OTHER_FAILURE;
    }
}
```

#### 4.5.2 New Velocity Mode Handler

```cpp
// In MotionController.cpp
RaftRetCode MotionController::moveToVelocity(MotionArgs& args, String* respMsg)
{
    // For velocity mode, we need to replace any existing velocity block
    // or add to empty pipeline
    
    // Option 1: Clear pipeline and add new velocity block
    // This provides instant velocity change behavior
    
    // Option 2: Add to pipeline, let existing blocks complete first
    // This provides queued velocity change behavior
    
    // Recommended: Option 1 with ramped transition
    // Clear pipeline, but preserve current velocity for smooth transition
    
    MotionPipelineIF& pipeline = _rampGenerator.getMotionPipeline();
    
    // Get current velocity from executing block (if any)
    MotionBlock* pCurrentBlock = pipeline.peekGet();
    AxisSpeedDataType currentVelocity = 0;
    if (pCurrentBlock && pCurrentBlock->_isExecuting)
    {
        // Capture current step rate as starting point
        // This requires RampGenerator to expose current rate
        currentVelocity = _rampGenerator.getCurrentStepRate();
    }
    
    // Stop current motion and clear queue
    _rampGenerator.stop();
    _blockManager.clear();
    
    // Create new velocity block starting from current velocity
    // (for smooth transition)
    RaftRetCode rc = _blockManager.addVelocityBlock(args, currentVelocity, pipeline, respMsg);
    
    return rc;
}
```

### 4.6 Phase 6: MotionBlockManager Extensions

#### 4.6.1 Velocity Block Addition

```cpp
// In MotionBlockManager.h
class MotionBlockManager
{
public:
    // ... existing methods ...
    
    /// @brief Add a velocity mode block
    /// @param args MotionArgs containing velocity parameters
    /// @param initialVelocity Starting velocity for smooth transition
    /// @param motionPipeline Motion pipeline to add block to
    /// @param respMsg Optional error message output
    /// @return RaftRetCode
    RaftRetCode addVelocityBlock(const MotionArgs& args,
                                  AxisSpeedDataType initialVelocity,
                                  MotionPipelineIF& motionPipeline,
                                  String* respMsg = nullptr);
};
```

```cpp
// In MotionBlockManager.cpp
RaftRetCode MotionBlockManager::addVelocityBlock(const MotionArgs& args,
                                                  AxisSpeedDataType initialVelocity,
                                                  MotionPipelineIF& motionPipeline,
                                                  String* respMsg)
{
    return _motionPlanner.moveVelocity(args, _axesState, _axesParams, motionPipeline);
}
```

---

## 5. Velocity Change and Transition Handling

### 5.1 Smooth Velocity Transitions

When receiving a new velocity command while already in velocity mode:

```cpp
// State machine for velocity transitions
enum class VelocityTransitionState
{
    IDLE,           // No velocity mode active
    ACCELERATING,   // Ramping up to target velocity
    STEADY_STATE,   // At target velocity
    DECELERATING,   // Ramping to new (lower) velocity or stop
    STOPPING        // Final deceleration to zero
};
```

### 5.2 Transition Scenarios

| Current State | New Command | Action |
|---------------|-------------|--------|
| IDLE | vel: [10, 0] | Accelerate to target |
| STEADY_STATE (10 u/s) | vel: [20, 0] | Accelerate to new target |
| STEADY_STATE (10 u/s) | vel: [5, 0] | Decelerate to new target |
| STEADY_STATE (10 u/s) | vel: [0, 0] | Decelerate to stop |
| STEADY_STATE (10 u/s) | stop | Immediate stop or controlled deceleration |
| STEADY_STATE (10 u/s) | abs: [100, 50] | Decelerate, then position move |

### 5.3 Implementation of Smooth Transitions

```cpp
// In RampGenerator - support for velocity change
void RampGenerator::requestVelocityChange(const AxesValues<AxisSpeedDataType>& newVelocities)
{
    // Store new target velocities
    _pendingVelocities = newVelocities;
    _velocityChangeRequested = true;
}

// In generateMotionPulses(), during velocity mode execution:
if (pBlock->isVelocityMode() && _velocityChangeRequested)
{
    // Calculate new target step rate
    uint32_t newTargetRate = calculateStepRateFromVelocity(_pendingVelocities);
    
    if (newTargetRate > _curStepRatePerTTicks)
    {
        // Accelerating - adjust max rate, continue ramping up
        pBlock->_maxStepRatePerTTicks = newTargetRate;
        pBlock->_finalStepRatePerTTicks = newTargetRate;
    }
    else if (newTargetRate < _curStepRatePerTTicks)
    {
        // Decelerating - start immediate deceleration
        pBlock->_finalStepRatePerTTicks = newTargetRate;
        pBlock->_stepsBeforeDecel = 0;  // Decelerate now
    }
    
    // Update target velocities for multi-axis coordination
    pBlock->_targetVelocities = _pendingVelocities;
    
    _velocityChangeRequested = false;
}
```

---

## 6. End-Stop Handling in Velocity Mode

### 6.1 Configuration Options

```cpp
// In MotionArgs - velocity mode end-stop behavior
enum class VelocityEndStopBehavior
{
    IGNORE,         // Ignore end-stops in velocity mode
    STOP_AXIS,      // Stop only the axis that hit end-stop
    STOP_ALL        // Stop all axes on any end-stop hit
};
```

### 6.2 End-Stop Handling in Velocity Mode

```cpp
// In RampGenerator::generateMotionPulses()
if (pBlock->isVelocityMode())
{
    for (int i = 0; i < _endStopCheckNum; i++)
    {
        EndStops* pEndStops = _axisEndStops[_endStopChecks[i].axisIdx];
        if (pEndStops->isAtEndStop(_endStopChecks[i].isMax))
        {
            switch (pBlock->_velocityEndStopBehavior)
            {
                case VelocityEndStopBehavior::STOP_ALL:
                    _endStopReached = true;
                    endMotion(pBlock);
                    return;
                    
                case VelocityEndStopBehavior::STOP_AXIS:
                    // Zero velocity for this axis only
                    pBlock->_targetVelocities.setVal(_endStopChecks[i].axisIdx, 0);
                    // Continue with other axes
                    break;
                    
                case VelocityEndStopBehavior::IGNORE:
                default:
                    break;
            }
        }
    }
}
```

---

## 7. Position Tracking in Velocity Mode

### 7.1 Challenge

In velocity mode, there's no target position, but we still need to track where the motors actually are.

### 7.2 Solution

The existing `_axisTotalSteps` tracking in `RampGenerator` continues to work:

```cpp
// In handleStepEnd() - already tracks total steps
if (_stepperDrivers[axisIdx]->stepEnd())
{
    _axisTotalSteps[axisIdx] = _axisTotalSteps[axisIdx] + _totalStepsInc[axisIdx];
}
```

However, `AxesState` (real-world coordinates) requires updates:

```cpp
// In MotionController::loop() or separate velocity mode handler
if (_rampGenerator.isVelocityModeActive())
{
    // Update axes state from step counts
    AxesValues<AxisStepsDataType> steps;
    _rampGenerator.getTotalStepPosition(steps);
    _blockManager.updateAxesStateFromSteps(steps);
}
```

---

## 8. API Extensions Summary

### 8.1 New/Modified Classes

| Class | Changes |
|-------|---------|
| `MotionArgs` | Add velocity array parsing in `fromJSON()` |
| `MotionBlock` | Add `_isVelocityMode`, `_targetVelocities`, `prepareForVelocityStepping()` |
| `RampGenerator` | Modify `handleStepMotion()` for velocity mode, add velocity change support |
| `MotionPlanner` | Add `moveVelocity()` method |
| `MotionBlockManager` | Add `addVelocityBlock()` method |
| `MotionController` | Add `moveToVelocity()` method, route velocity mode in `moveTo()` |

### 8.2 New Public Methods

```cpp
// MotionController
RaftRetCode moveToVelocity(MotionArgs& args, String* respMsg = nullptr);

// MotionBlockManager  
RaftRetCode addVelocityBlock(const MotionArgs& args, AxisSpeedDataType initialVelocity,
                              MotionPipelineIF& motionPipeline, String* respMsg = nullptr);

// MotionPlanner
RaftRetCode moveVelocity(const MotionArgs& args, AxesState& axesState,
                         const AxesParams& axesParams, MotionPipelineIF& motionPipeline);

// RampGenerator
void requestVelocityChange(const AxesValues<AxisSpeedDataType>& newVelocities);
bool isVelocityModeActive() const;
AxisSpeedDataType getCurrentStepRate() const;
```

---

## 9. Testing Strategy

### 9.1 Unit Tests

1. **MotionArgs parsing**: Verify velocity mode detection and velocity array parsing
2. **MotionBlock velocity prep**: Test `prepareForVelocityStepping()` calculations
3. **Single axis velocity**: Verify correct step rate generation
4. **Multi-axis velocity**: Verify coordinated multi-axis velocity ratios
5. **Velocity transitions**: Test acceleration, deceleration, and direction changes
6. **Stop handling**: Test immediate stop and controlled deceleration

### 9.2 Integration Tests

1. **End-to-end velocity command**: JSON → stepping output verification
2. **Velocity change while moving**: Smooth transition verification
3. **End-stop behavior**: Each end-stop handling mode
4. **Mixed mode transitions**: Position → velocity → position

### 9.3 Hardware Tests

1. **Single motor velocity control**: Verify actual motor speed matches commanded
2. **Multi-motor coordination**: Verify velocity ratios on physical hardware
3. **Long-duration stability**: Verify no drift or timing issues over extended operation
4. **Emergency stop**: Verify safe stopping from velocity mode

---

## 10. Implementation Timeline

| Phase | Description | Estimated Effort |
|-------|-------------|------------------|
| 1 | MotionArgs extensions | 2 hours |
| 2 | MotionBlock extensions | 4 hours |
| 3 | RampGenerator modifications | 6 hours |
| 4 | MotionPlanner extensions | 3 hours |
| 5 | MotionController integration | 2 hours |
| 6 | MotionBlockManager extensions | 2 hours |
| 7 | Unit tests | 4 hours |
| 8 | Integration tests | 4 hours |
| 9 | Hardware testing & tuning | 6 hours |
| 10 | Documentation updates | 2 hours |
| **Total** | | **35 hours** |

---

## 11. Future Enhancements

### 11.1 Controlled Deceleration Stop

Instead of immediate stop, implement configurable deceleration profile for stop command.

### 11.2 Jog Mode

Short burst velocity movements with automatic stop after timeout.

### 11.3 Velocity Profiling

Time-based velocity profiles (e.g., sinusoidal velocity for oscillating motion).

### 11.4 Torque/Current Limiting in Velocity Mode

Dynamic current adjustment based on load detection.

---

## 12. Conclusion

This implementation leverages the existing block-based motion architecture by treating velocity mode as a special case of motion blocks with infinite duration. The key insight is that:

1. **Acceleration handling** already exists in `RampGenerator`
2. **Multi-axis coordination** uses the existing Bresenham-style stepping
3. **Block termination** simply needs modification to not terminate on step completion

The "infinite block" approach minimizes changes to the proven ISR-level code while providing clean velocity mode semantics. The main modifications are:

1. New velocity mode flag in `MotionBlock`
2. Modified termination logic in `RampGenerator::handleStepMotion()`
3. New `moveVelocity()` method in `MotionPlanner`
4. Routing logic in `MotionController::moveTo()`

This design maintains backward compatibility with all existing position-mode functionality while adding clean velocity mode support.
