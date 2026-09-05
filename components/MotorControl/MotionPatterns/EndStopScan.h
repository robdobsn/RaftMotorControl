////////////////////////////////////////////////////////////////////////////////
//
// EndStopScan
// Motion pattern for characterising an end-stop sensor region
//
// Drives one axis back and forth across its end-stop region, logging every
// transition position in a SINGLE continuous coordinate frame (the origin is
// deliberately NOT reset). Homing cannot answer these questions because it
// zeroes the frame every cycle, varies the start pose, and yields only two
// crossings per run - so sensor repeatability, hysteresis and drift are all
// confounded together.
//
// What each quantity separates out:
//   * same-direction repeatability : spread of enter positions across passes
//                                    in the same direction
//   * hysteresis                   : offset between the position of a given
//                                    physical edge measured in each direction
//   * drift                        : trend of those positions with pass index
//
// Positions come from the ramp generator's ISR edge capture, so each is exact
// to one step-generator tick regardless of seek rate.
//
// Rob Dobson 2026
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "MotionPatternBase.h"
#include "MotionControlIF.h"
#include "RaftArduino.h"

class EndStopScan : public MotionPatternBase
{
public:
    EndStopScan(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);
    virtual ~EndStopScan();

    /// @brief Setup pattern
    /// @param pParamsJson axis, stepsPerSec, passes, clearSteps, timeoutMs, startDir
    virtual void setup(const char* pParamsJson = nullptr) override;

    /// @brief Service loop
    virtual void loop() override;

    static MotionPatternBase* create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

private:
    enum class State {
        IDLE,
        SEEKING,        // travelling toward/through the region, logging transitions
        CLEARING,       // past the region by clearSteps, then reverse
        COMPLETE,
        ERROR
    };

    State _state = State::IDLE;

    int _axis = 0;
    uint32_t _stepsPerSec = 200;
    int _passes = 20;               // number of region crossings to perform
    int _passesDone = 0;
    int _dir = -1;                  // current travel direction
    AxisStepsDataType _clearSteps = 300;
    AxisStepsDataType _clearStartPos = 0;
    uint32_t _timeoutMs = 300000;
    uint32_t _startTimeMs = 0;
    int _fullRotationSteps = 9600;
    int _maxRotations = 3;

    // Transition bookkeeping for the current pass
    bool _sawEnter = false;
    AxisStepsDataType _enterPos = 0;
    int _transitionsLogged = 0;

    void setState(State s) { _state = s; }
    void sendRotate(int dir);
    void stopMotion();
    void drainTransitions();

    static constexpr const char* MODULE_PREFIX = "EndStopScan";
};
