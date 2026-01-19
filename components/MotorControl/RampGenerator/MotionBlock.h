/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlock
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <math.h>
#include "AxesParams.h"
#include "AxisEndstopChecks.h"

// Feature flag for single split-block optimization
// Phase 1-5: Keep at 0 for parallel development
// Phase 6: Set to 1 to enable optimization
#define USE_SINGLE_SPLIT_BLOCK 1

class MotionBlock
{
public:
    MotionBlock();

    // Set timer period
    void setTimerPeriodNs(uint32_t stepGenPeriodNs);

    // Clear
    void clear();

    // Motion tracking
    void setMotionTrackingIndex(uint32_t motionTrackingIndex);
    uint32_t getMotionTrackingIndex();

    // Target
    AxesValues<AxisStepsDataType> getStepsToTarget() const
    {
        return _stepsTotalMaybeNeg;
    }
    void setStepsToTarget(const AxesValues<AxisStepsDataType>& steps)
    {
        _stepsTotalMaybeNeg = steps;
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            int steps = _stepsTotalMaybeNeg.getVal(axisIdx);
            if (abs(steps) > abs(_stepsTotalMaybeNeg.getVal(_axisIdxWithMaxSteps)))
                _axisIdxWithMaxSteps = axisIdx;
        }
    }

    // Rates
    uint32_t getExitStepRatePerTTicks();
    static AxisSpeedDataType maxAchievableSpeed(AxisAccDataType acceleration, 
                        AxisSpeedDataType target_velocity, 
                        AxisDistDataType distance);

    // End stops
    void setEndStopsToCheck(const AxisEndstopChecks &endStopCheck);

    // Prepare a block for stepping
    // If the block is "stepwise" this means that there is no acceleration and deceleration - just steps
    //     at the requested rate - this is generally used for homing, etc
    // If not "stepwise" then the block's entry and exit speed are now known
    //     so the block can accelerate and decelerate as required as long as these criteria are met -
    //     we now compute the stepping parameters to make motion happen
    bool prepareForStepping(const AxesParams &axesParams, bool isLinear);

    // Debug
    void debugShowTimingConsts() const;
    void debugShowBlkHead() const;
    void debugShowBlock(int elemIdx, const AxesParams &axesParams) const;
    double debugStepRateToMMps(uint32_t val) const
    {
        return (((val * 1.0) * _ticksPerSec) / MotionBlock::TTICKS_VALUE) * _debugStepDistMM;
    }
    double debugStepRateToMMps2(uint32_t val) const
    {
        return (((val * 1.0) * 1000 * _ticksPerSec) / MotionBlock::TTICKS_VALUE) * _debugStepDistMM;
    }

    // Minimum move distance
    static constexpr double MINIMUM_MOVE_DIST_MM = 0.0001;

    // Number of ticks to accumulate for rate actuation
    static constexpr uint32_t TTICKS_VALUE = 1000000000l;

    // Number of ns in ms
    static constexpr uint32_t NS_IN_A_MS = 1000000;

    // Calculate ticks per second
    static double calcTicksPerSec(uint32_t stepGenPeriodNs)
    {
        return 1.0e9 / stepGenPeriodNs;
    }

    // Calculate minimum step rate per TTICKS 
    static uint32_t calcMinStepRatePerTTicks(uint32_t stepGenPeriodNs)
    {
        // This is to ensure that the robot never goes to 0 tick rate - which would leave it
        // immobile forever
        static constexpr uint32_t MIN_STEP_RATE_PER_SEC = 10;
        return uint32_t((MIN_STEP_RATE_PER_SEC * 1.0 * MotionBlock::TTICKS_VALUE) / calcTicksPerSec(stepGenPeriodNs));
    }

public:
    // Flags
    struct
    {
        // Flag indicating the block is currently executing
        volatile bool _isExecuting : 1;
        // Flag indicating the block can start executing
        volatile bool _canExecute : 1;
        // Block is followed by others
        bool _blockIsFollowed : 1;
    };

    // Requested max speed for move - either axis units-per-sec or 
    // stepsPerSec depending if move is stepwise
    AxisSpeedDataType _requestedSpeed = 0;
    // Distance (pythagorean) to move considering primary axes only
    AxisDistDataType _moveDistPrimaryAxesMM = 0;
    // Unit vector on axis with max movement
    AxisUnitVectorDataType _unitVecAxisWithMaxDist = 0;
    // Computed max entry speed for a block based on max junction deviation calculation
    AxisSpeedDataType _maxEntrySpeedMMps = 0;
    // Computed entry speed for this block
    AxisSpeedDataType _entrySpeedMMps = 0;
    // Computed exit speed for this block
    AxisSpeedDataType _exitSpeedMMps = 0;
    // End-stops to test
    AxisEndstopChecks _endStopsToCheck;

    // Steps to target and before deceleration
    AxesValues<AxisStepsDataType> _stepsTotalMaybeNeg;
    int _axisIdxWithMaxSteps = 0;
    uint32_t _stepsBeforeDecel = 0;

    // Stepping acceleration/deceleration profile
    uint32_t _initialStepRatePerTTicks = 0;
    uint32_t _maxStepRatePerTTicks = 0;
    uint32_t _finalStepRatePerTTicks = 0;
    uint32_t _accStepsPerTTicksPerMS = 0;

    // Motion tracking index - to help keep track of motion execution from other processes
    // like homing
    uint32_t _motionTrackingIndex = 0;

#if USE_SINGLE_SPLIT_BLOCK
    // Split-block support (backward compatible - all false/0 by default)
    // When enabled, a single MotionBlock can represent multiple geometric waypoints
    // This reduces memory allocations, pipeline operations, and planner overhead
    bool _isSplitBlock = false;
    uint16_t _totalSubBlocks = 0;
    uint16_t _currentSubBlock = 0;
    AxesValues<AxisStepsDataType> _startActuatorCoords;
    AxesValues<AxisStepsDataType> _actuatorDeltaPerSubBlock;

    // Split-block accessors
    bool isSplitBlock() const { return _isSplitBlock; }
    uint16_t getTotalSubBlocks() const { return _totalSubBlocks; }
    uint16_t getCurrentSubBlock() const { return _currentSubBlock; }
    bool hasMoreSubBlocks() const { 
        return _isSplitBlock && _currentSubBlock < _totalSubBlocks - 1; 
    }
    void advanceSubBlock() { 
        if (_isSplitBlock && _currentSubBlock < _totalSubBlocks - 1) 
            _currentSubBlock++; 
    }
    
    // Configure block as split-block with waypoint metadata
    void configureSplitBlock(uint16_t numSubBlocks,
                            const AxesValues<AxisStepsDataType>& startCoords,
                            const AxesValues<AxisStepsDataType>& endCoords);
    
    // Get interpolated actuator position for current sub-block
    void getCurrentActuatorPosition(AxesValues<AxisStepsDataType>& outCoords) const;
#endif

private:
    // Step distance in MM
    double _debugStepDistMM = 0;

    // Ticks per second
    double _ticksPerSec = 0;

    // Helpers
    template<typename T>
    void forceInBounds(T &val, T lowBound, T highBound);

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionBlock";    
};
