/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenerator
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftCore.h"
#include "MotionBlock.h"
#include "RampGenStats.h"
#include "MotionPipeline.h"

#ifdef ESP_PLATFORM
#include "RampGenTimer.h"
#endif

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

    /// @brief Loop - must be called very frequently if not using timer ISR (maybe called less frequently if using timer ISR)
    void loop();

    // Start / stop / pause
    void start();
    void stop();
    void pause(bool pauseIt);

    // Access to current state
    void resetTotalStepPosition();
    void resetAxisStepPosition(uint32_t axisIdx);
    void getTotalStepPosition(AxesValues<AxisStepsDataType>& actuatorPos) const;

    // End stop handling
    void clearEndstopReached();
    void getEndStopStatus(AxisEndstopChecks& axisEndStopVals) const;
    bool isEndStopReached() const;

    /// @brief Arm step-rate capture of end-stop transitions for one axis
    /// @param axisIdx axis to watch
    /// @param isMax true for max end-stop, false for min
    /// @note Homing needs the axis position at the *instant* an end-stop changes
    ///       state. Sampling from the pattern service loop quantises that
    ///       position to the loop interval (measured: ~27 steps at 400 steps/sec,
    ///       ~1.0 deg), which propagates directly into the sensor midpoint and
    ///       therefore the home position. Sampling here — in the same ISR that
    ///       maintains `_axisTotalSteps` — makes the captured position exact to
    ///       one step generator tick and independent of seek rate.
    ///       Costs one `digitalRead` per tick and only while armed.
    void armEndStopEdgeCapture(uint32_t axisIdx, bool isMax);

    /// @brief Disarm end-stop transition capture
    void disarmEndStopEdgeCapture();

    /// @brief Get the most recent captured end-stop transition
    /// @param seq (out) capture sequence number - increments on each transition
    /// @param posSteps (out) axis total step position at the transition
    /// @param newState (out) end-stop state *after* the transition
    /// @return true if at least one transition has been captured since arming
    bool getEndStopEdgeCapture(uint32_t& seq, AxisStepsDataType& posSteps, bool& newState) const;

    /// @brief Pop the oldest unread end-stop transition from the capture queue
    /// @param posSteps (out) axis total step position at the transition
    /// @param newState (out) end-stop state after the transition
    /// @return true if a transition was available
    /// @note The single-slot latch above only keeps the most recent transition,
    ///       which is fine for homing (one edge per seek leg) but loses data in a
    ///       continuous scan where crossings arrive faster than the service loop
    ///       reads them. This queue keeps them all.
    bool popEndStopEdge(AxisStepsDataType& posSteps, bool& newState);

    /// @brief Number of transitions dropped because the capture queue was full
    uint32_t getEndStopEdgeOverflowCount() const { return _edgeCapOverflow; }

    // Get ramp gen timer period us
    uint64_t getPeriodUs() const
    {
        return _stepGenPeriodNs / 1000;
    }

    // Get minimum step rate per TTicks
    uint32_t getMinStepRatePerTTicks() const
    {
        return _minStepRatePerTTicks;
    }

    // Check if velocity mode is currently active
    bool isVelocityModeActive() const;

    // Get current step rate (for velocity transitions)
    uint32_t getCurrentStepRatePerTTicks() const
    {
        return _curStepRatePerTTicks;
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
    String getDebugJSON(bool includeBraces) const
    {
#if defined(ESP_PLATFORM)
        return _rampGenTimer.getDebugJSON(includeBraces);
#else
        return includeBraces ? "{}" : "";
#endif
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
#if defined(ESP_PLATFORM)
    RampGenTimer _rampGenTimer;
#endif
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

    // End-stop transition capture (see armEndStopEdgeCapture)
    volatile bool _edgeCapArmed = false;
    volatile uint8_t _edgeCapAxisIdx = 0;
    volatile bool _edgeCapIsMax = false;
    volatile bool _edgeCapLastState = false;
    volatile bool _edgeCapPrimed = false;      // last state valid (first sample taken)
    volatile uint32_t _edgeCapSeq = 0;
    volatile int32_t _edgeCapPosSteps = 0;
    volatile bool _edgeCapNewState = false;

    // Lossless transition queue (single producer: ISR, single consumer: loop)
    static const uint32_t EDGE_CAP_QUEUE_SIZE = 32;
    struct EdgeCapEntry
    {
        int32_t posSteps;
        bool newState;
    };
    volatile EdgeCapEntry _edgeCapQueue[EDGE_CAP_QUEUE_SIZE];
    volatile uint32_t _edgeCapHead = 0;      // written by ISR
    volatile uint32_t _edgeCapTail = 0;      // written by consumer
    volatile uint32_t _edgeCapOverflow = 0;

    // Stats
    RampGenStats _stats;

    // Helpers
    void generateMotionPulses();
    void sampleEndStopEdge();
    bool handleStepEnd();
    void setupNewBlock(MotionBlock *pBlock);
    void updateMSAccumulator(MotionBlock *pBlock);
    bool handleStepMotion(MotionBlock *pBlock);
    void endMotion(MotionBlock *pBlock);

    /// @brief Timer callback
    /// @param pObject Object to call (this class instance)
    static MOTOR_TICK_FN_DECORATOR void rampGenTimerCallback(void* pObject)
    {
        if (pObject)
            ((RampGenerator*)pObject)->generateMotionPulses();
    }

    // ISR count
    volatile uint32_t _isrCount = 0;

    // Debug
    uint32_t _debugLastQueuePeekMs = 0;

    // Debug ramp gen timer
    uint32_t _debugRampGenLoopLastMs = 0;
    uint32_t _debugRampGenLoopCount = 0;
};
