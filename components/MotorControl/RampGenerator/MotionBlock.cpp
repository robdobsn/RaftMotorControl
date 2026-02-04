/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlock
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "MotionBlock.h"
#include "AxesValues.h"
#include "AxesParams.h"
#include "RampGenConsts.h"

// #define DEBUG_ENDSTOPS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionBlock::MotionBlock()
{
    clear();
    _ticksPerSec = calcTicksPerSec(RAMP_GEN_PERIOD_US_DEFAULT * 1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set timer period
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::setTimerPeriodNs(uint32_t stepGenPeriodNs)
{
    _ticksPerSec = calcTicksPerSec(stepGenPeriodNs);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Clear
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::clear()
{
    // Clear values
    _requestedSpeed = 0;
    _moveDistPrimaryAxesMM = 0;
    _maxEntrySpeedMMps = 0;
    _entrySpeedMMps = 0;
    _exitSpeedMMps = 0;
    _debugStepDistMM = 0;
    _isExecuting = false;
    _canExecute = false;
    _blockIsFollowed = false;
    _axisIdxWithMaxSteps = 0;
    _unitVecAxisWithMaxDist = 0;
    _accStepsPerTTicksPerMS = 0;
    _finalStepRatePerTTicks = 0;
    _initialStepRatePerTTicks = 0;
    _maxStepRatePerTTicks = 0;
    _stepsBeforeDecel = 0;
    _motionTrackingIndex = 0;
    _endStopsToCheck.clear();
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        _stepsTotalMaybeNeg[axisIdx] = 0;
    // Velocity mode
    _isVelocityMode = false;
    _targetVelocities.clear();
    _velocityRatios.clear();
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        _velocityRatiosScaled[axisIdx] = 0;
#if USE_SINGLE_SPLIT_BLOCK
    _isSplitBlock = false;
    _totalSubBlocks = 0;
    _currentSubBlock = 0;
#endif
}

#if USE_SINGLE_SPLIT_BLOCK
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configure split-block
// Sets up metadata for a block that represents multiple geometric waypoints
// The block will have a single acceleration profile but multiple position targets
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::configureSplitBlock(uint16_t numSubBlocks,
                                     const AxesValues<AxisStepsDataType>& startCoords,
                                     const AxesValues<AxisStepsDataType>& endCoords)
{
    _isSplitBlock = true;
    _totalSubBlocks = numSubBlocks;
    _currentSubBlock = 0;
    _startActuatorCoords = startCoords;
    
    // Calculate delta per sub-block for linear interpolation in actuator space
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        int32_t totalDelta = endCoords.getVal(axisIdx) - startCoords.getVal(axisIdx);
        // Divide by total sub-blocks to get increment per sub-block
        _actuatorDeltaPerSubBlock.setVal(axisIdx, totalDelta / (int32_t)numSubBlocks);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get current actuator position
// Returns interpolated position for the current sub-block waypoint
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::getCurrentActuatorPosition(AxesValues<AxisStepsDataType>& outCoords) const
{
    if (!_isSplitBlock)
    {
        // For normal blocks, return the target position
        outCoords = _stepsTotalMaybeNeg;
        return;
    }
    
    // Interpolate position for current sub-block
    // Position = start + (delta_per_block * current_index)
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        int32_t interpolatedPos = _startActuatorCoords.getVal(axisIdx) + 
                                  (_actuatorDeltaPerSubBlock.getVal(axisIdx) * (int32_t)_currentSubBlock);
        outCoords.setVal(axisIdx, interpolatedPos);
    }
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Command tracking
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::setMotionTrackingIndex(uint32_t motionTrackingIndex)
{
    _motionTrackingIndex = motionTrackingIndex;
}
uint32_t MOTOR_TICK_FN_DECORATOR MotionBlock::getMotionTrackingIndex()
{
    return _motionTrackingIndex;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Block params
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t MotionBlock::getExitStepRatePerTTicks()
{
    return _finalStepRatePerTTicks;
}

float MotionBlock::maxAchievableSpeed(AxisAccDataType acceleration, 
                AxisSpeedDataType target_velocity, 
                AxisDistDataType distance)
{
    return sqrtf(target_velocity * target_velocity + 2.0F * acceleration * distance);
}

template<typename T>
void MotionBlock::forceInBounds(T &val, T lowBound, T highBound)
{
    if (val < lowBound)
        val = lowBound;
    if (val > highBound)
        val = highBound;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// End-stops
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::setEndStopsToCheck(const AxisEndstopChecks &endStopCheck)
{
#ifdef DEBUG_ENDSTOPS
    LOG_I(MODULE_PREFIX, "Set test endstops %x", endStopCheck.debugGetRawValue());
#endif
    _endStopsToCheck = endStopCheck;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prepare a block for stepping
// If the block is "stepwise" this means that there is no acceleration and deceleration - just steps
//     at the requested rate - this is generally used for homing, etc
// If not "stepwise" then the block's entry and exit speed are now known
//     so the block can accelerate and decelerate as required as long as these criteria are met -
//     we now compute the stepping parameters to make motion happen
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionBlock::prepareForStepping(const AxesParams &axesParams, bool isLinear)
{
    // If block is currently being executed don't change it
    if (_isExecuting)
        return false;

    // Find the max number of steps for any axis
    uint32_t absMaxStepsForAnyAxis = abs(_stepsTotalMaybeNeg[_axisIdxWithMaxSteps]);

    // Check if stepwise movement (see note above)
    float initialStepRatePerSec = 0;
    float finalStepRatePerSec = 0;
    float maxAccStepsPerSec2 = 0;
    float axisMaxStepRatePerSec = 0;
    uint32_t stepsDecelerating = 0; 
    double stepDistMM = 0;
    if (isLinear)
    {
        // requestedVelocity is in steps per second in this case
        float stepRatePerSec = _requestedSpeed;
        if (stepRatePerSec > axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps))
            stepRatePerSec = axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps);
        initialStepRatePerSec = stepRatePerSec;
        finalStepRatePerSec = stepRatePerSec;
        maxAccStepsPerSec2 = stepRatePerSec;
        axisMaxStepRatePerSec = stepRatePerSec;
        stepsDecelerating = 0;
    }
    else
    {
        // Get the initial step rate, final step rate and max acceleration for the axis with max steps
        stepDistMM = fabs(_moveDistPrimaryAxesMM / _stepsTotalMaybeNeg[_axisIdxWithMaxSteps]);
        initialStepRatePerSec = fabs(_entrySpeedMMps / stepDistMM);
        if (initialStepRatePerSec > axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps))
            initialStepRatePerSec = axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps);
        finalStepRatePerSec = fabs(_exitSpeedMMps / stepDistMM);
        if (finalStepRatePerSec > axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps))
            finalStepRatePerSec = axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps);
        maxAccStepsPerSec2 = fabs(axesParams.getMaxAccelUps2(_axisIdxWithMaxSteps) / stepDistMM);

        // Calculate the distance decelerating and ensure within bounds
        // Using the facts for the block ... (assuming max accleration followed by max deceleration):
        //		Vmax * Vmax = Ventry * Ventry + 2 * Amax * Saccelerating
        //		Vexit * Vexit = Vmax * Vmax - 2 * Amax * Sdecelerating
        //      Stotal = Saccelerating + Sdecelerating
        // And solving for Saccelerating (distance accelerating)
        uint32_t stepsAccelerating = 0;
        float stepsAcceleratingFloat =
            ceilf((powf(finalStepRatePerSec, 2) - powf(initialStepRatePerSec, 2)) / 4 /
                        maxAccStepsPerSec2 +
                    absMaxStepsForAnyAxis / 2);
        if (stepsAcceleratingFloat > 0)
        {
            stepsAccelerating = uint32_t(stepsAcceleratingFloat);
            if (stepsAccelerating > absMaxStepsForAnyAxis)
                stepsAccelerating = absMaxStepsForAnyAxis;
        }

        // Decelerating steps
        stepsDecelerating = 0;

        // Find max possible rate for axis with max steps
        axisMaxStepRatePerSec = fabs(_requestedSpeed / stepDistMM);
        if (axisMaxStepRatePerSec > axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps))
            axisMaxStepRatePerSec = axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps);

        // See if max speed will be reached
        uint32_t stepsToMaxSpeed =
            uint32_t((powf(axisMaxStepRatePerSec, 2) - powf(initialStepRatePerSec, 2)) /
                        2 / maxAccStepsPerSec2);
        if (stepsAccelerating > stepsToMaxSpeed)
        {
            // Max speed will be reached
            stepsAccelerating = stepsToMaxSpeed;

            // Decelerating steps
            stepsDecelerating =
                uint32_t((powf(axisMaxStepRatePerSec, 2) - powf(finalStepRatePerSec, 2)) /
                            2 / maxAccStepsPerSec2);
        }
        else
        {
            // Calculate max speed that will be reached
            axisMaxStepRatePerSec =
                sqrtf(powf(initialStepRatePerSec, 2) + 2.0F * maxAccStepsPerSec2 * stepsAccelerating);

            // Decelerating steps
            stepsDecelerating = absMaxStepsForAnyAxis - stepsAccelerating;
        }
    }

    // Fill in the step values for this axis
    _initialStepRatePerTTicks = uint32_t((initialStepRatePerSec * TTICKS_VALUE) / _ticksPerSec);
    _maxStepRatePerTTicks = uint32_t((axisMaxStepRatePerSec * TTICKS_VALUE) / _ticksPerSec);
    _finalStepRatePerTTicks = uint32_t((finalStepRatePerSec * TTICKS_VALUE) / _ticksPerSec);
    _accStepsPerTTicksPerMS = uint32_t((maxAccStepsPerSec2 * TTICKS_VALUE) / _ticksPerSec / 1000);
    _stepsBeforeDecel = absMaxStepsForAnyAxis - stepsDecelerating;
    _debugStepDistMM = stepDistMM;

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debug
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlock::debugShowTimingConsts() const
{
    LOG_I(MODULE_PREFIX, "TTicksValue (accumulator) %u, TicksPerSec %0.0f", TTICKS_VALUE, _ticksPerSec);
}

void MotionBlock::debugShowBlkHead() const
{
    LOG_I(MODULE_PREFIX, "#i EntMMps ExtMMps StTot0 StTot1 StTot2 St>Dec    Init     (perTT)      Pk     (perTT)     Fin     (perTT)     Acc     (perTT) UnitVecMax   FeedRtMMps StepDistMM  MaxStepRate");
}

void MotionBlock::debugShowBlock(int elemIdx, const AxesParams &axesParams) const
{
    char baseStr[100];
    snprintf(baseStr, sizeof(baseStr), "%2d%8.3f%8.3f%7d%7d%7d%7u",                 
                elemIdx,
                _entrySpeedMMps,
                _exitSpeedMMps,
                (int)_stepsTotalMaybeNeg[0],
                (int)_stepsTotalMaybeNeg[1],
                (int)_stepsTotalMaybeNeg[2],
                (int)_stepsBeforeDecel);
    char extStr[200];
    snprintf(extStr, sizeof(extStr), "%8.3f(%10ld)%8.3f(%10ld)%8.3f(%10ld)%8.3f(%10lu)%13.8f%11.6f%11.8f%11.3f",
                debugStepRateToMMps(_initialStepRatePerTTicks), (long int)_initialStepRatePerTTicks,
                debugStepRateToMMps(_maxStepRatePerTTicks), (long int)_maxStepRatePerTTicks,
                debugStepRateToMMps(_finalStepRatePerTTicks), (long int)_finalStepRatePerTTicks,
                debugStepRateToMMps2(_accStepsPerTTicksPerMS), (long unsigned)_accStepsPerTTicksPerMS,
                _unitVecAxisWithMaxDist,
                _requestedSpeed,
                _debugStepDistMM,
                axesParams.getMaxStepRatePerSec(0));
    LOG_I(MODULE_PREFIX, "%s%s", baseStr, extStr);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Prepare for velocity mode stepping
// Velocity mode blocks run indefinitely at the target velocity until stopped or replaced
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionBlock::prepareForVelocityStepping(const AxesParams &axesParams, uint32_t minStepRatePerTTicks)
{
    // If block is currently being executed don't change it
    if (_isExecuting)
        return false;

    _isVelocityMode = true;

    // Find the axis with maximum absolute velocity (becomes the "dominant" axis)
    AxisSpeedDataType maxAbsVelStepsPerSec = 0;
    _axisIdxWithMaxSteps = 0;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        AxisSpeedDataType absVel = fabs(_targetVelocities.getVal(axisIdx));
        if (absVel > maxAbsVelStepsPerSec)
        {
            maxAbsVelStepsPerSec = absVel;
            _axisIdxWithMaxSteps = axisIdx;
        }
    }

    // Near-zero velocity means stop - return false to indicate no movement
    if (maxAbsVelStepsPerSec < 0.001)
    {
        _isVelocityMode = false;
        return false;
    }

    // Cap at maximum step rate for the dominant axis
    if (maxAbsVelStepsPerSec > axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps))
        maxAbsVelStepsPerSec = axesParams.getMaxStepRatePerSec(_axisIdxWithMaxSteps);

    // Set up step directions based on velocity signs
    // For velocity mode, _stepsTotalMaybeNeg encodes direction only (sign of 1 or -1)
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        AxisSpeedDataType vel = _targetVelocities.getVal(axisIdx);
        _stepsTotalMaybeNeg.setVal(axisIdx, (vel >= 0) ? 1 : -1);
    }

    // Calculate velocity ratios for multi-axis coordination
    // Each axis velocity relative to dominant axis velocity
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        AxisSpeedDataType thisAbsVel = fabs(_targetVelocities.getVal(axisIdx));
        if (maxAbsVelStepsPerSec > 0.001)
        {
            float ratio = thisAbsVel / maxAbsVelStepsPerSec;
            _velocityRatios.setVal(axisIdx, ratio);
            // Pre-compute integer-scaled ratio for ISR use (avoids FPU in ISR)
            _velocityRatiosScaled[axisIdx] = uint32_t(ratio * VEL_RATIO_SCALE);
        }
        else
        {
            _velocityRatios.setVal(axisIdx, 0);
            _velocityRatiosScaled[axisIdx] = 0;
        }
    }

    // Calculate target step rate from velocity (for dominant axis)
    float targetStepRatePerSec = maxAbsVelStepsPerSec;

    // Set up acceleration profile to reach target velocity
    // Start from minimum step rate (or could inherit from previous block for smooth transition)
    _initialStepRatePerTTicks = minStepRatePerTTicks;
    _maxStepRatePerTTicks = uint32_t((targetStepRatePerSec * TTICKS_VALUE) / _ticksPerSec);
    _finalStepRatePerTTicks = _maxStepRatePerTTicks;  // No deceleration in steady state

    // Acceleration in steps per TTicks per millisecond
    // Use acceleration from dominant axis
    float maxAccStepsPerSec2 = axesParams.getMaxAccelUps2(_axisIdxWithMaxSteps) * 
                               axesParams.getStepsPerUnit(_axisIdxWithMaxSteps);
    _accStepsPerTTicksPerMS = uint32_t((maxAccStepsPerSec2 * TTICKS_VALUE) / _ticksPerSec / 1000);

    // For velocity mode, _stepsBeforeDecel is set to MAX so we never decelerate
    _stepsBeforeDecel = UINT32_MAX;

    // Set requested speed for reference
    _requestedSpeed = maxAbsVelStepsPerSec;

    // Set unit vector (1.0 for dominant axis in velocity mode)
    _unitVecAxisWithMaxDist = 1.0;

    return true;
}
