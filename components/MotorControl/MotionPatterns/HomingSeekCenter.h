////////////////////////////////////////////////////////////////////////////////
//
// HomingSeekCenter
// Motion pattern for homing axes by seeking center of endstop trigger region
//
// Algorithm:
// 1. Rotate CW (seekOffDir) until detector NOT triggered (exit trigger region)
// 2. Rotate CCW (seekEdgeDir) until triggered -> record position A
// 3. Continue CCW until NOT triggered -> record position B
// 4. Home position = (A + B) / 2
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
    /// @param pNamedValueProvider Named value provider
    /// @param motionControl Motion control interface
    /// @return Pointer to new HomingSeekCenter instance
    static MotionPatternBase* create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

private:
    // State machine states
    enum class State {
        IDLE,
        START,
        SEEK_OFF,           // Rotate to exit trigger region (if currently triggered)
        SEEK_OFF_CLEAR,     // Continue past the release edge for a configured margin
        SEEK_EDGE_A,        // Rotate to find entering edge (triggered)
        EDGE_A_BACKOFF,     // Back off a fixed margin from the located edge, then re-approach
        SEEK_CROSS,         // Single continuous pass through the region capturing BOTH edges
        SETTLING_A,         // Wait for motor to settle after stop
        SEEK_EDGE_B,        // Continue to find leaving edge (not triggered)
        SETTLING_B,         // Wait for motor to settle after stop
        MOVE_TO_MIDPOINT,   // Move to calculated midpoint
        SETTLING_MID,       // Wait for motor to settle at midpoint
        SET_HOME,           // Set this axis home, move to next
        NEXT_AXIS,
        COMPLETE,
        ERROR
    };

    // State machine
    State _state = State::IDLE;
    uint32_t _stateEntryTimeMs = 0;

    // Axis being homed
    int _currentAxis = 0;
    int _startAxis = 0;

    // Number of axes to home
    int _numAxes = 2;

    // Position tracking
    AxisStepsDataType _edgeAPos = 0;     // Position where endstop became triggered
    AxisStepsDataType _edgeBPos = 0;     // Position where endstop became not triggered
    AxisStepsDataType _midPoint = 0;

    // Configuration parameters
    std::vector<uint32_t> _stepsPerSecPerAxis; // Steps per second for seeking for each axis
    std::vector<AxisStepsDataType> _homeOffsetStepsPerAxis; // Per-axis home offset (steps) applied after the sensor midpoint
    std::vector<AxisStepsDataType> _seekOffClearStepsPerAxis; // Per-axis extra travel past the release edge (0 = disabled)
    AxisStepsDataType _seekOffClearStartPos = 0; // Recorded position at the moment SEEK_OFF exited the region

    // setHomeHere calibration mode (motors?cmd=home&setHomeHere=1): user places the ball
    // at the table centre; homing measures the offset from each sensor midpoint to that
    // captured centre, sets home there, and persists the offsets (Option A).
    bool _setHomeHereMode = false;
    bool _persistOffset = true;
    std::vector<AxisStepsDataType> _centreStartPos;     // captured centre position (steps) per axis
    std::vector<AxisStepsDataType> _setHomeHereOffsets; // computed offset (steps) per axis
    uint32_t _timeoutMs = 180000;   // timeout per axis (raised for slow-homing test)
    int _fullRotationSteps = 9600;
    int _maxRotations = 3;          // Number of full rotations to command for seeking
    int _seekOffDir = 1;            // Direction to seek off trigger region (CW = +1)
    int _seekEdgeDir = -1;          // Direction to seek edges (CCW = -1)
    uint32_t _settleDelayMs = 50;   // Delay after stop to let motor settle
    int _positionTolerance = 10;    // Tolerance for position comparison
    AxisStepsDataType _detectPos = 0; // Position captured at the instant an edge is detected (logs overshoot)

    // Step-rate end-stop edge capture. The service loop only notices a
    // transition at its next poll, which quantises the recorded edge position by
    // (seek rate x loop interval) - measured at ~27 steps (~1.0 deg) at 400
    // steps/sec, and scaling linearly with seek rate. The ramp generator ISR
    // latches the exact position instead; these hold what it captured.
    // Fixed re-approach to edge A. `endstop-scan` (2026-09-03) measured 1.2-1.8 deg
    // of travel-direction lost motion per joint, and same-direction edge
    // repeatability of only 1-2 steps. So the edge position is highly repeatable
    // PROVIDED the approach is identical every time. Locating the edge, backing off
    // a fixed margin and re-approaching guarantees that: same direction, same
    // run-up distance, lost motion taken up the same way on every home.
    // 0 disables (param "reapproachSteps").
    AxisStepsDataType _reapproachSteps = 300;
    bool _reapproachDone = false;         // per-axis: final approach already made
    AxisStepsDataType _reapproachStartPos = 0;  // edge position the back-off is measured from

    // Continuous crossing. Homing used to stopAndClear at edge A, settle, then
    // restart toward edge B. `endstop-scan`, which crosses the region in ONE
    // motion, repeats to 1-2 steps; homing with the stop was bimodal at ~26
    // steps. The abrupt stop lets a belt-driven arm recoil, so lost motion is
    // re-taken-up on restart and adds a variable offset to the A->B distance.
    // Both edges now come from the ISR capture queue during a single pass - the
    // stop only existed because reading position reliably needed a stationary axis.
    bool _crossSawEnter = false;

    // MOVE_TO_MIDPOINT completion. The original check required the polled
    // position to land within `_positionTolerance` (10 steps) of the target,
    // but the axis routinely settles 6-9 steps out - so any cycle landing 11+
    // steps off never satisfied it and homing hung until the 180 s pattern
    // timeout, surfacing as "homing did not complete". Completion is now
    // "motion has stopped", with the tolerance kept only as a fast path.
    AxisStepsDataType _moveLastPos = 0;
    uint32_t _moveLastChangeMs = 0;
    static const uint32_t MOVE_STOPPED_MS = 300;

    bool _edgeCapturedValid = false;      // ISR captured this edge (else fell back to polling)
    AxisStepsDataType _edgeCapturedPos = 0;
    uint32_t _edgeCaptureQuantErr = 0;    // |polled - captured|, for diagnostics

    // Timing
    uint32_t _timeoutStartMs = 0;
    uint32_t _settleStartMs = 0;

    // Error string
    String _lastError = "";

    // Internal helpers
    void setState(State newState)
    {
        _state = newState;
        _stateEntryTimeMs = millis();
    }
    bool readEndStop(int axis, bool& isFresh);
    /// @brief Arm step-rate capture of the next end-stop transition on an axis
    void armEdgeCapture(int axis);
    /// @brief Read the step-exact position of the captured end-stop transition
    /// @param posOut (out) latched position, untouched if no capture available
    /// @return true if the ISR captured a transition since arming
    bool takeEdgeCapture(AxisStepsDataType& posOut);
    void logHomingCompletionState();
    AxisStepsDataType readAxisPosition(int axis, bool& isFresh);
    void stopMotion();
    void setAxisHome(int axis);
    void setError(const String& errStr);
    void resetState();
    void sendRotate(int axis, int dir, uint32_t stepsPerSec = 0);
    void sendMoveTo(int axis, AxisStepsDataType targetPos, AxisStepsDataType currentPos);

    // Debug
    static constexpr const char* MODULE_PREFIX = "HomingSeekCenter";
};
