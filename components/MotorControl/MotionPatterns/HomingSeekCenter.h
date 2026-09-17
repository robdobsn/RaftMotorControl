////////////////////////////////////////////////////////////////////////////////
//
// HomingSeekCenter
// Homes each axis to the CENTRE of its end-stop trigger region.
//
// The sensor is triggered over a RANGE of travel (the flag has width), so the
// only repeatable home is the midpoint between the two edges of that range.
// Both edges must therefore be measured; using a single edge inherits all of
// the edge's own hysteresis and approach-direction error.
//
// Per axis, four states and no movement that is not needed:
//
//   1. FAST_APPROACH  fast. End up just OUTSIDE one edge of the flag.
//                     If starting inside the flag, drive out until released.
//                     If starting outside, drive in until triggered, then
//                     reverse out. Either way the arm finishes clear of the
//                     flag by CLEAR_MARGIN_STEPS and no further.
//   2. SLOW_CROSS     slow. ONE continuous pass across the whole flag. The ISR
//                     latches the exact step position of both transitions:
//                     entering (edge A) and leaving (edge B). Stops as soon as
//                     B is captured - it does not run on to the end of the
//                     commanded move.
//   3. FAST_TO_MID    seek speed. Move to (A+B)/2, a known distance, no
//                     sensing. Deliberately NOT the fast rate: the move is only
//                     a few hundred steps and overshot the midpoint when run
//                     fast.
// `homeOffsetSteps` is the distance from the end-stop midpoint to the arm's
// GEOMETRIC zero. It is applied as the position REPORTED at the parked
// midpoint, never as somewhere to drive to: the arm stays on the midpoint (the
// most repeatable point on the sensor) and reported angles still satisfy
// forward kinematics. The two are genuinely different places - measured
// -11.50 deg (axis 0) and -6.25 deg (axis 1) - and they differ PER AXIS, which
// is why leaving them out distorted the position field in a way no global
// similarity fit could absorb (residual 5.27 mm vs 0.73 mm with them).
//
//   4. VERIFY         no movement. The end-stop MUST read triggered at the
//                     midpoint; if it does not, the measurement was wrong and
//                     homing fails loudly rather than silently setting a bogus
//                     origin.
//
// Edge positions come from armEndStopEdgeCapture/popEndStopEdge, which latch
// in the ramp-generator ISR. That is what makes the fast approach safe: the
// recorded position is exact even though the arm overshoots while stopping,
// so approach speed does not bias the edges. Only the CROSS is slow, and only
// because the flag edges are what we are measuring.
//
// Replaces a 15-state version that also carried setHomeHere calibration,
// seek-off clearance margins, edge back-off and re-approach, and three
// separate settle states. That version intermittently finished one clearance
// move (seekOffClearSteps, 11.25 deg) away from home when run above ~15 deg/s,
// because the extra states took different branches at different speeds.
//
// Rob Dobson 2025
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "MotionPatternBase.h"
#include "MotionControlIF.h"
#include "RaftArduino.h"

class HomingSeekCenter : public MotionPatternBase
{
public:
    /// @brief Constructor
    /// @param pNamedValueProvider Named value provider for accessing system state
    /// @param motionControl Motion control interface
    HomingSeekCenter(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

    /// @brief Destructor
    virtual ~HomingSeekCenter();

    /// @brief Setup pattern with optional parameters
    /// @param pParamsJson JSON parameters for pattern configuration
    virtual void setup(const char* pParamsJson = nullptr) override;

    /// @brief Service loop - called frequently to advance homing state machine
    virtual void loop() override;

    /// @brief Factory function for creating homing pattern instances
    static MotionPatternBase* create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

private:
    enum class State {
        IDLE,
        FAST_APPROACH,      // get just outside one edge of the flag, quickly
        SLOW_CROSS,         // one slow pass capturing both edges
        FAST_TO_MID,        // move to the measured midpoint
        VERIFY,             // confirm triggered at midpoint, set origin
        NEXT_AXIS,
        COMPLETE,
        ERROR
    };

    // ---- state ----
    State _state = State::IDLE;
    uint32_t _stateEntryTimeMs = 0;
    int _currentAxis = 0;
    int _startAxis = 0;
    int _numAxes = 2;

    // Sub-phase of FAST_APPROACH: true once we are driving back out of the flag
    bool _approachReversing = false;
    bool _startedInsideFlag = false;
    bool _moveIssued = false;
    uint32_t _moveIssuedMs = 0;

    // A move that has just been queued does not report isBusy() straight away.
    // Wait at least this long after issuing before treating !isBusy() as "the
    // move finished", or the state machine races ahead of the machine.
    static constexpr uint32_t MOVE_PICKUP_MS = 250;

    // ---- measured edges (steps, ISR-latched) ----
    bool _gotEdgeA = false;
    bool _gotEdgeB = false;
    AxisStepsDataType _edgeAPos = 0;    // position where endstop BECAME triggered
    AxisStepsDataType _edgeBPos = 0;    // position where endstop BECAME released
    AxisStepsDataType _midPos = 0;

    // ---- configuration ----
    std::vector<uint32_t> _fastStepsPerSec;   // approach / move-to-mid rate
    std::vector<uint32_t> _slowStepsPerSec;   // edge measurement rate
    // Reported position at the parked midpoint (NOT a park target) - see VERIFY
    std::vector<AxisStepsDataType> _homeOffsetStepsPerAxis;
    int _fullRotationSteps = 9600;
    uint32_t _timeoutMs = 60000;
    int _seekDir = -1;          // direction that moves INTO the flag
    uint32_t _settleDelayMs = 50;

    // Distance to stand clear of an edge before the slow cross. Must exceed the
    // stopping distance of the fast approach or the cross would start inside
    // the flag and miss edge A.
    AxisStepsDataType _clearMarginSteps = 600;

    // Upper bound on flag width; the slow cross is commanded this far and is
    // expected to capture edge B well before the end.
    AxisStepsDataType _maxFlagWidthSteps = 2000;

    // ---- helpers ----
    void enterState(State s);
    bool endStopTriggered(int axis) const;
    AxisStepsDataType axisPos(int axis) const;
    void moveSteps(int axis, AxisStepsDataType steps, uint32_t stepsPerSec);
    void stopMotion();
    void setError(const char* msg);
    void startAxis(int axis);

    static constexpr const char* MODULE_PREFIX = "HomingSeekCenter";
};
