/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenerator
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "MotionBlock.h"
#include <Logger.h>
#include <AxisInt32s.h>
#include <RampGenStats.h>

class MotionPipelineIF;
class RampGenTimer;
class StepDriverBase;
class EndStops;

class RampGenerator
{
public:
    // Constructor / destructor
    RampGenerator(MotionPipelineIF& motionPipeline, RampGenTimer& rampGenTimer);
    virtual ~RampGenerator();

    // Setup ramp generator
    void setup(bool useRampGenTimer, std::vector<StepDriverBase*> stepperDrivers,
            std::vector<EndStops*> axisEndStops);

    // Must be called frequently - if useRampGenTimer is false (in setup) then
    // this function generates stepping pulses
    void service();

    // Enable
    void enable(bool en);

    // Control
    void stop();
    void pause(bool pauseIt);

    // Access to current state
    void resetTotalStepPosition();
    void getTotalStepPosition(AxesParamVals<AxisStepsDataType>& actuatorPos) const;
    void setTotalStepPosition(int axisIdx, int32_t stepPos);

    // End stop handling
    void clearEndstopReached();
    void getEndStopStatus(AxisEndstopChecks& axisEndStopVals) const;
    bool isEndStopReached() const;

    // Progress
    // int getLastCompletedNumberedCmdIdx();

    RampGenStats& getStats()
    {
        return _stats;
    }
    void debugShowStats();

private:
    // If this is true nothing will move
    volatile bool _isPaused = true;

    // Stop is pending
    volatile bool _stopPending = false;

    // Steps moved in total and increment based on direction
    volatile int32_t _axisTotalSteps[AXIS_VALUES_MAX_AXES] = {0};
    volatile int32_t _totalStepsInc[AXIS_VALUES_MAX_AXES] = {0};

    // Pipeline of blocks to be processed
    MotionPipelineIF& _motionPipeline;

    // Ramp generation timer
    RampGenTimer& _rampGenTimer;
    bool _useRampGenTimer = false;
    uint32_t _stepGenPeriodNs = 0;
    uint32_t _minStepRatePerTTicks = 0;

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
};
