/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenerator
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RampGenerator.h"
#include "esp_intr_alloc.h"
#include "MotionPipeline.h"
#include "RampGenTimer.h"
#include "RaftArduino.h"
#include "StepDriverBase.h"
#include "EndStops.h"
#include "AxisEndstopChecks.h"

#define RAMP_GEN_DETAILED_STATS
// #define DEBUG_MOTION_PULSE_GEN
// #define DEBUG_MOTION_PEEK_QUEUE
// #define DEBUG_SETUP_NEW_BLOCK
// #define DEBUG_RAMP_GEN_SERVICE

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
RampGenerator::RampGenerator()
{
    // Init
    resetTotalStepPosition();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
RampGenerator::~RampGenerator()
{
    // Release timer hook
    _rampGenTimer.unhookTimer(this);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup ramp generator
/// @param config Configuration
/// @param stepperDrivers Stepper drivers
/// @param axisEndStops End stops
void RampGenerator::setup(const RaftJsonIF& config,
            const std::vector<StepDriverBase*>& stepperDrivers,
            const std::vector<EndStops*>& axisEndStops)
{
    // Check if using ramp timer
    _useRampGenTimer = config.getBool("rampTimerEn", false);

    // Ramp generator config
    long rampTimerUs = config.getLong("rampTimerUs", RampGenTimer::RAMP_GEN_PERIOD_US_DEFAULT);

    // Ramp generator timer
    bool timerSetupOk = false;
    if (_useRampGenTimer)
    {
        // Setup timer (the exact period may be different from the requested period)
        timerSetupOk = _rampGenTimer.setup(rampTimerUs);
        _stepGenPeriodNs = _rampGenTimer.getPeriodUs() * 1000;
        if (!timerSetupOk)
        {
            _useRampGenTimer = false;
            LOG_E(MODULE_PREFIX, "setup timer setup failed");
        }
    }

    // Check if using a timer interrupt
    if (!_useRampGenTimer)
    {
        // Use a fixed period
        _stepGenPeriodNs = rampTimerUs * 1000;
    }

    // Set timing period for step generation
    _minStepRatePerTTicks = MotionBlock::calcMinStepRatePerTTicks(_stepGenPeriodNs);

    // Store steppers and end stops
    _stepperDrivers = stepperDrivers;
    _axisEndStops = axisEndStops;

    // Calculate ramp gen periods
    _minStepRatePerTTicks = MotionBlock::calcMinStepRatePerTTicks(_stepGenPeriodNs);

    // Hook the timer if required
    if (_useRampGenTimer)
        _rampGenTimer.hookTimer(rampGenTimerCallback, this);

    // Setup motion pipeline
    uint32_t pipelineLen = config.getLong("pipelineLen", PIPELINE_LEN_DEFAULT);
    _motionPipeline.setup(pipelineLen);

    // Debug
    LOG_I(MODULE_PREFIX, "setup useTimerInterrupt %s stepGenPeriod %dus numStepperDrivers %d numEndStops %d pipelineLen %d", 
                _useRampGenTimer ? "Y" : "N", 
                _stepGenPeriodNs / 1000, _stepperDrivers.size(), _axisEndStops.size(), pipelineLen);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Loop - must be called very frequently if not using timer ISR (maybe called less frequently if using timer ISR)
void RampGenerator::loop()
{
    // Loop RampGenIO
    // TODO
    // _rampGenIO.loop();

    // Check if timer used for pulse generation - otherwise pump many times to
    // aid testing
    if (!_useRampGenTimer)
    {
        // Check time to generate pulses
        if (Raft::isTimeout(millis(), _nonTimerLoopLastMs, NON_TIMER_SERVICE_CALL_MIN_MS))
        {
            _nonTimerLoopLastMs = millis();
            // Calculate times to call to give equivalent to timer rate
            uint32_t numCalls = (NON_TIMER_SERVICE_CALL_MIN_MS * 1000) / (_stepGenPeriodNs / 1000);
            for (uint32_t i = 0; i < numCalls; i++)
                generateMotionPulses();
        }
    }

#ifdef DEBUG_RAMP_GEN_SERVICE
    // Debug
    _debugRampGenLoopCount++;
    if (Raft::isTimeout(millis(), _debugRampGenLoopLastMs, 1000))
    {
        LOG_I(MODULE_PREFIX, "loop count %d useRampGenTimer %d", _debugRampGenLoopCount, _useRampGenTimer);
        _debugRampGenLoopLastMs = millis();
    }
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief start ramp generation
void RampGenerator::start()
{
    _rampGenEnabled = true;
    _stopPending = false;
    pause(false);
    if (_useRampGenTimer)
        _rampGenTimer.enable(true);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief stop ramp generation
void RampGenerator::stop()
{
    _stopPending = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief pause ramp generation
void RampGenerator::pause(bool pauseIt)
{
    _isPaused = pauseIt;
    if (!_isPaused)
    {
        _endStopReached = false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Axis position handling
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenerator::resetTotalStepPosition()
{
    for (int i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        _axisTotalSteps[i] = 0;
        _totalStepsInc[i] = 0;
    }
}
void RampGenerator::getTotalStepPosition(AxesValues<AxisStepsDataType>& actuatorPos) const
{
    for (int i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        actuatorPos.setVal(i, _axisTotalSteps[i]);
    }
}
void RampGenerator::setTotalStepPosition(int axisIdx, int32_t stepPos)
{
    if ((axisIdx >= 0) && (axisIdx < AXIS_VALUES_MAX_AXES))
        _axisTotalSteps[axisIdx] = stepPos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// End stop handling
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenerator::clearEndstopReached()
{
    _endStopReached = false;
}

bool RampGenerator::isEndStopReached() const
{
    return _endStopReached;
}

void RampGenerator::getEndStopStatus(AxisEndstopChecks& axisEndStopVals) const
{
    // Iterate endstops
    for (int axisIdx = 0; (axisIdx < _axisEndStops.size()) && (axisIdx < AXIS_VALUES_MAX_AXES); axisIdx++)
    {
        if (!_axisEndStops[axisIdx])
            continue;
        axisEndStopVals.set(axisIdx, AxisEndstopChecks::MIN_VAL_IDX, 
            _axisEndStops[axisIdx]->isAtEndStop(false) ? 
                    AxisEndstopChecks::END_STOP_HIT : 
                    AxisEndstopChecks::END_STOP_NOT_HIT);
        axisEndStopVals.set(axisIdx, AxisEndstopChecks::MAX_VAL_IDX, 
            _axisEndStops[axisIdx]->isAtEndStop(true) ?
                    AxisEndstopChecks::END_STOP_HIT : 
                    AxisEndstopChecks::END_STOP_NOT_HIT);
    }
    // _rampGenIO.getEndStopStatus(axisEndStopVals);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tracking progress
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// int RampGenerator::getLastCompletedNumberedCmdIdx()
// {
//     return _lastDoneNumberedCmdIdx;
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle the end of a step for any axis
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IRAM_ATTR RampGenerator::handleStepEnd()
{
    bool anyPinReset = false;
    for (uint32_t axisIdx = 0; axisIdx < _stepperDrivers.size(); axisIdx++)
    {
        if (_stepperDrivers[axisIdx])
        {
            if (_stepperDrivers[axisIdx]->stepEnd())
            {
                anyPinReset = true;
                _axisTotalSteps[axisIdx] = _axisTotalSteps[axisIdx] + _totalStepsInc[axisIdx];
            }
        }
    }
    return anyPinReset;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup new block
/// @param pBlock Motion block defines all motion parameters
/// @note This function is called when a new block is added to the pipeline
///       It sets up the block for execution recording all the info needed to process the block
void IRAM_ATTR RampGenerator::setupNewBlock(MotionBlock *pBlock)
{
    // Setup step counts, direction and endstops for each axis
    _endStopCheckNum = 0;
    for (uint32_t axisIdx = 0; axisIdx < _stepperDrivers.size(); axisIdx++)
    {
        if (!_stepperDrivers[axisIdx])
            continue;
        // Total steps
        int32_t stepsTotal = pBlock->_stepsTotalMaybeNeg[axisIdx];
        _stepsTotalAbs[axisIdx] = UTILS_ABS(stepsTotal);
        _curStepCount[axisIdx] = 0;
        _curAccumulatorRelative[axisIdx] = 0;
        // Set direction for the axis
        _stepperDrivers[axisIdx]->setDirection(stepsTotal >= 0);
        _totalStepsInc[axisIdx] = (stepsTotal >= 0) ? 1 : -1;

#ifdef DEBUG_SETUP_NEW_BLOCK
        if (!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "setupNewBlock setDirection %d stepsTotal %d numSteppers %d stepType %s", 
                        stepsTotal >= 0, stepsTotal, _stepperDrivers.size(), _stepperDrivers[axisIdx]->getDriverType().c_str());
        }
#endif

        // Instrumentation
        _stats.stepDirn(axisIdx, stepsTotal >= 0);

        // Check if any endstops to setup
        if (pBlock->_endStopsToCheck.any())
        { 
            // Check if the axis is moving in a direction which might result in hitting an active end-stop
            for (int minMaxIdx = 0; minMaxIdx < AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS; minMaxIdx++)
            {
                // See if anything to check for
                AxisEndstopChecks::AxisMinMaxEnum minMaxType = pBlock->_endStopsToCheck.get(axisIdx, minMaxIdx);
                if (minMaxType == AxisEndstopChecks::END_STOP_NONE)
                    continue;

                // Check for towards - this is different from MAX or MIN because the axis will still move even if
                // an endstop is hit if the movement is away from that endstop
                if (minMaxType == AxisEndstopChecks::END_STOP_TOWARDS)
                {
                    // Stop at max if we're heading towards max OR
                    // stop at min if we're heading towards min
                    if (!(((minMaxIdx == AxisEndstopChecks::MAX_VAL_IDX) && (stepsTotal > 0)) ||
                            ((minMaxIdx == AxisEndstopChecks::MIN_VAL_IDX) && (stepsTotal < 0))))
                        continue;
                }
                
                // Config for end stop
                if ((axisIdx <= _axisEndStops.size()) && (_axisEndStops[axisIdx]))
                {
                    bool isValid = _axisEndStops[axisIdx]->isValid(minMaxIdx == AxisEndstopChecks::MAX_VAL_IDX);
                    if (isValid)
                    {
                        _endStopChecks[_endStopCheckNum].axisIdx = axisIdx;
                        _endStopChecks[_endStopCheckNum].checkHit = minMaxType != AxisEndstopChecks::END_STOP_NOT_HIT;
                        _endStopCheckNum = _endStopCheckNum + 1;
                    }
                }
            }
        }
    }

    // Accumulator reset
    _curAccumulatorStep = 0;
    _curAccumulatorNS = 0;

    // Step rate
    _curStepRatePerTTicks = pBlock->_initialStepRatePerTTicks;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Update the motion block time accumulators to handle acceleration and deceleration
/// @param pBlock Motion block defines all motion parameters
void IRAM_ATTR RampGenerator::updateMSAccumulator(MotionBlock *pBlock)
{
    // Bump the millisec accumulator
    _curAccumulatorNS = _curAccumulatorNS + _stepGenPeriodNs;

    // Check for millisec accumulator overflow
    if (_curAccumulatorNS >= MotionBlock::NS_IN_A_MS)
    {
        // Subtract from accumulator leaving remainder to combat rounding errors
        _curAccumulatorNS = _curAccumulatorNS - MotionBlock::NS_IN_A_MS;

        // Check if decelerating
        if (_curStepCount[pBlock->_axisIdxWithMaxSteps] > pBlock->_stepsBeforeDecel)
        {
            if (_curStepRatePerTTicks > UTILS_MAX(_minStepRatePerTTicks + pBlock->_accStepsPerTTicksPerMS,
                                                 pBlock->_finalStepRatePerTTicks + pBlock->_accStepsPerTTicksPerMS))
                _curStepRatePerTTicks = _curStepRatePerTTicks - pBlock->_accStepsPerTTicksPerMS;
        }
        else if ((_curStepRatePerTTicks < _minStepRatePerTTicks) || (_curStepRatePerTTicks < pBlock->_maxStepRatePerTTicks))
        {
            if (_curStepRatePerTTicks + pBlock->_accStepsPerTTicksPerMS < MotionBlock::TTICKS_VALUE)
                _curStepRatePerTTicks = _curStepRatePerTTicks + pBlock->_accStepsPerTTicksPerMS;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Handle step motion
/// @param pBlock Motion block defines all motion parameters
/// @return true if any axis is still moving
/// @note Handle the start of step on each axis
bool IRAM_ATTR RampGenerator::handleStepMotion(MotionBlock *pBlock)
{
    // Complete Flag
    bool anyAxisMoving = false;

    // Axis with most steps
    int axisIdxMaxSteps = pBlock->_axisIdxWithMaxSteps;
    if ((axisIdxMaxSteps < 0) || (axisIdxMaxSteps >= _stepperDrivers.size()))
        return false;

    // Subtract from accumulator leaving remainder
    _curAccumulatorStep = _curAccumulatorStep - MotionBlock::TTICKS_VALUE;

    // Step the axis with the greatest step count if needed
    if (_curStepCount[axisIdxMaxSteps] < _stepsTotalAbs[axisIdxMaxSteps])
    {
        // Step this axis
        if (_stepperDrivers[axisIdxMaxSteps])
            _stepperDrivers[axisIdxMaxSteps]->stepStart();
        _curStepCount[axisIdxMaxSteps] = _curStepCount[axisIdxMaxSteps] + 1;
        if (_curStepCount[axisIdxMaxSteps] < _stepsTotalAbs[axisIdxMaxSteps])
            anyAxisMoving = true;

        // Instrumentation
        _stats.stepStart(axisIdxMaxSteps);

#ifdef DEBUG_MOTION_PULSE_GEN
        if (!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "handleStepMotion stepStart axisIdxMaxSteps %d axisDriver %p numStepperDrivers %d driverType %s", 
                        axisIdxMaxSteps, _stepperDrivers[axisIdxMaxSteps], _stepperDrivers.size(), 
                        _stepperDrivers[axisIdxMaxSteps] ? _stepperDrivers[axisIdxMaxSteps]->getDriverType().c_str() : "null");
        }
#endif
    }

    // Check if other axes need stepping
    for (uint32_t axisIdx = 0; axisIdx < _stepperDrivers.size(); axisIdx++)
    {
        // Skip the axis with the most steps and axes already at their target
        if ((axisIdx == axisIdxMaxSteps) || (_curStepCount[axisIdx] == _stepsTotalAbs[axisIdx]))
            continue;

        // Bump the relative accumulator
        _curAccumulatorRelative[axisIdx] = _curAccumulatorRelative[axisIdx] + _stepsTotalAbs[axisIdx];
        if (_curAccumulatorRelative[axisIdx] >= _stepsTotalAbs[axisIdxMaxSteps])
        {
            // Do the remainder calculation
            _curAccumulatorRelative[axisIdx] = _curAccumulatorRelative[axisIdx] - _stepsTotalAbs[axisIdxMaxSteps];

            // Step the axis
            if (_stepperDrivers[axisIdx])
                _stepperDrivers[axisIdx]->stepStart();

#ifdef DEBUG_MOTION_PULSE_GEN
            if (!_useRampGenTimer)
            {
                LOG_I(MODULE_PREFIX, "handleStepMotion otherAxisStep ax %d cur %d tot %d", axisIdx, _curStepCount[axisIdx], _stepsTotalAbs[axisIdx]);
            }
#endif

            // Move the count onward
            _curStepCount[axisIdx] = _curStepCount[axisIdx] + 1;
            if (_curStepCount[axisIdx] < _stepsTotalAbs[axisIdx])
                anyAxisMoving = true;

            // Instrumentation
            _stats.stepStart(axisIdx);
        }
    }

    // Return indicator of block complete
    return anyAxisMoving;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief End motion
/// @param pBlock Motion block defines all motion parameters
/// @note This function is called when a block is completed and removes the block from the pipeline
void IRAM_ATTR RampGenerator::endMotion(MotionBlock *pBlock)
{
    _motionPipeline.remove();
    // Check if this is a numbered block - if so record its completion
    // if (pBlock->getMotionTrackingIndex() != RobotConsts::NUMBERED_COMMAND_NONE)
    //     _lastDoneNumberedCmdIdx = pBlock->getMotionTrackingIndex();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Generate motion pulses
/// @note This function is called from the timer ISR or from the main loop if not using a timer ISR
void IRAM_ATTR RampGenerator::generateMotionPulses()
{
    // Instrumentation code to time ISR execution (if enabled)
    _stats.startMotionProcessing();

    // Count ISR entries
    _isrCount = _isrCount + 1;

    // Indicate tick
    // TODO
    // _rampGenIO.tickIndicator();

    // Do a step-end for any motor which needs one - return here to avoid too short a pulse
    if (handleStepEnd())
    {
#ifdef DEBUG_MOTION_PULSE_GEN
        if(!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "generateMotionPulses stepEnd true exiting");
        }
#endif
        return;
    }

    // Check stop pending
    if (_stopPending)
    {
#ifdef DEBUG_MOTION_PULSE_GEN
        if(!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "generateMotionPulses stopPending clearing pipeline");
        }
#endif
        // Check if a block is executing
        MotionBlock *pBlock = _motionPipeline.peekGet();
        if (pBlock && pBlock->_isExecuting)
        {
            // Cancel motion (by removing the block)
            endMotion(pBlock);
        }
        _stopPending = false;
        return;
    }

    // Check if paused
    if (_isPaused)
    {
#ifdef DEBUG_MOTION_PULSE_GEN
        if(!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "generateMotionPulses paused exiting");
        }
#endif
        return;
    }

    // Peek a MotionPipelineElem from the queue
    MotionBlock *pBlock = _motionPipeline.peekGet();
    if (!pBlock)
    {
#ifdef DEBUG_MOTION_PEEK_QUEUE
        if (Raft::isTimeout(millis(), _debugLastQueuePeekMs, 1000))
        {
            if(!_useRampGenTimer)
            {
                LOG_I(MODULE_PREFIX, "generateMotionPulses no block exiting");
            }
            _debugLastQueuePeekMs = millis();
        }
#endif
        return;
    }

    // Check if the element can be executed
    if (!pBlock->_canExecute)
    {
#ifdef DEBUG_MOTION_PEEK_QUEUE
        if (Raft::isTimeout(millis(), _debugLastQueuePeekMs, 1000))
        {
            if(!_useRampGenTimer)
            {
                LOG_I(MODULE_PREFIX, "generateMotionPulses can't execute exiting");
            }
            _debugLastQueuePeekMs = millis();
        }
#endif
        return;
    }

    // See if the block was already executing and set isExecuting if not
    bool newBlock = !pBlock->_isExecuting;
    pBlock->_isExecuting = true;

    // New block
    if (newBlock)
    {
        // Setup new block
        setupNewBlock(pBlock);

        // Return here to reduce the maximum time this function takes
        // Assuming this function is called frequently (<50uS intervals say)
        // then it will make little difference if we return now and pick up on the next tick
        return;
    }

    // Check endstops        
    bool endStopHit = false;
    for (int i = 0; i < _endStopCheckNum; i++)
    {
        EndStops* pEndStops = _axisEndStops[_endStopChecks[i].axisIdx];
        if (pEndStops->isAtEndStop(_endStopChecks[i].isMax) == _endStopChecks[i].checkHit)
            endStopHit = true;
    }

    // Handle end-stop hit
    if (endStopHit)
    {
        // Cancel motion (by removing the block) as end-stop reached
        _endStopReached = true;
        endMotion(pBlock);

        // Only use this debugging if not driving from ISR
#ifdef DEBUG_MOTION_PULSE_GEN
        if (!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "generateMotionPulses endStopHit - stopping");
        }
#endif
    }

    // Update the millisec accumulator - this handles the process of changing speed incrementally to
    // implement acceleration and deceleration
    updateMSAccumulator(pBlock);

    // Bump the step accumulator
    _curAccumulatorStep = _curAccumulatorStep + UTILS_MAX(_curStepRatePerTTicks, _minStepRatePerTTicks);

#ifdef RAMP_GEN_DETAILED_STATS
    _stats.update(_curAccumulatorStep, _curStepRatePerTTicks, _curAccumulatorNS,
                pBlock->_axisIdxWithMaxSteps,
                pBlock->_accStepsPerTTicksPerMS,
                _curStepCount[pBlock->_axisIdxWithMaxSteps],
                pBlock->_stepsBeforeDecel,
                pBlock->_maxStepRatePerTTicks);
#endif

    // Check for step accumulator overflow
    if (_curAccumulatorStep >= MotionBlock::TTICKS_VALUE)
    {
#ifdef DEBUG_MOTION_PULSE_GEN
        if (!_useRampGenTimer)
        {
            LOG_I(MODULE_PREFIX, "generateMotionPulses accumulator overflow");
        }
#endif

        // Flag indicating this block is finished
        bool anyAxisMoving = false;

        // Handle a step
        anyAxisMoving = handleStepMotion(pBlock);

        // Any axes still moving?
        if (!anyAxisMoving)
        {
            // This block is done
            endMotion(pBlock);
        }
    }

    // Time execution
    _stats.endMotionProcessing();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Debug show stats
void RampGenerator::debugShowStats()
{
    LOG_I(MODULE_PREFIX, "%s isrCount %d", _stats.getStatsStr().c_str(), _isrCount);
}
