/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenerator
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Logger.h"
#include "MotionBlock.h"
#include "RampGenStats.h"
#include "RampGenTimer.h"
#include "MotionPipeline.h"

class RampGenTimer;
class StepDriverBase;
class EndStops;

class RampGenerator
{
public:
    // Constructor / destructor
    RampGenerator();
    virtual ~RampGenerator();

    // Setup ramp generator
    void setup(const RaftJsonIF& config, 
            const std::vector<StepDriverBase*>& stepperDrivers,
            const std::vector<EndStops*>& axisEndStops);

    // Must be called frequently - if useRampGenTimer is false (in setup) then
    // this function generates stepping pulses
    void loop();

    // Start / stop / pause
    void start();
    void stop();
    void pause(bool pauseIt);

    // Access to current state
    void resetTotalStepPosition();
    void getTotalStepPosition(AxesValues<AxisStepsDataType>& actuatorPos) const;
    void setTotalStepPosition(int axisIdx, int32_t stepPos);

    // End stop handling
    void clearEndstopReached();
    void getEndStopStatus(AxisEndstopChecks& axisEndStopVals) const;
    bool isEndStopReached() const;

    // Get ramp gen timer period us
    uint64_t getPeriodUs() const
    {
        return _stepGenPeriodNs / 1000;
    }

    // Get motion pipeline
    MotionPipelineIF& getMotionPipeline()
    {
        return _motionPipeline;
    }
    const MotionPipelineIF& getMotionPipelineConst() const
    {
        return _motionPipeline;
    }

    // Check if using timer ISR
    bool isUsingTimerISR() const
    {
        return _useRampGenTimer;
    }

    // Progress
    // int getLastCompletedNumberedCmdIdx();

    const RampGenStats& getStats() const
    {
        return _stats;
    }
    void debugShowStats();
    String getDebugStr() const
    {
        return _rampGenTimer.getDebugStr();
    }

private:

    // Debug
    static constexpr const char* MODULE_PREFIX = "RampGen";

    // Consts
    static constexpr uint32_t PIPELINE_LEN_DEFAULT = 100;
    static constexpr uint32_t NON_TIMER_SERVICE_CALL_MIN_MS = 5;

    // If this is true nothing will move
    volatile bool _isPaused = true;

    // Stop is pending
    volatile bool _stopPending = false;

    // Steps moved in total and increment based on direction
    volatile int32_t _axisTotalSteps[AXIS_VALUES_MAX_AXES] = {0};
    volatile int32_t _totalStepsInc[AXIS_VALUES_MAX_AXES] = {0};

    // Pipeline of blocks to be processed
    MotionPipeline _motionPipeline;

    // Ramp generation timer
    RampGenTimer _rampGenTimer;
    bool _useRampGenTimer = false;
    uint32_t _stepGenPeriodNs = 0;
    uint32_t _minStepRatePerTTicks = 0;

    // Non-timer loop rate
    uint32_t _nonTimerLoopLastMs = 0;

    // Steppers
    std::vector<StepDriverBase*> _stepperDrivers;
    
    // Endstops
    std::vector<EndStops*> _axisEndStops;

    // Ramp generation enabled
    bool _rampGenEnabled = false;
    // Last completed numbered command
    // volatile int _lastDoneNumberedCmdIdx;
    // Steps
    volatile uint32_t _stepsTotalAbs[AXIS_VALUES_MAX_AXES] = {0};
    volatile uint32_t _curStepCount[AXIS_VALUES_MAX_AXES] = {0};
    // Current step rate (in steps per K ticks)
    volatile uint32_t _curStepRatePerTTicks = 0;
    // Accumulators for stepping and acceleration increments
    volatile uint32_t _curAccumulatorStep = 0;
    volatile uint32_t _curAccumulatorNS = 0;
    volatile uint32_t _curAccumulatorRelative[AXIS_VALUES_MAX_AXES] = {0};

    // End stop handling
    volatile bool _endStopReached = false;
    volatile int _endStopCheckNum = 0;
    struct EndStopChecks
    {
        uint8_t axisIdx;
        bool isMax;
        bool checkHit;
    };
    volatile EndStopChecks _endStopChecks[AXIS_VALUES_MAX_AXES];

    // Stats
    RampGenStats _stats;

    // Helpers
    void generateMotionPulses();
    bool handleStepEnd();
    void setupNewBlock(MotionBlock *pBlock);
    void updateMSAccumulator(MotionBlock *pBlock);
    bool handleStepMotion(MotionBlock *pBlock);
    void endMotion(MotionBlock *pBlock);

    // Timer callback
    static void rampGenTimerCallback(void* pObject);

    // ISR count
    volatile uint32_t _isrCount = 0;

    // Debug
    uint32_t _debugLastQueuePeekMs = 0;

    // Debug ramp gen timer
    uint32_t _debugRampGenLoopLastMs = 0;
    uint32_t _debugRampGenLoopCount = 0;
};
