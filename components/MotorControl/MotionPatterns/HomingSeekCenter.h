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
        SEEK_EDGE_A,        // Rotate to find entering edge (triggered)
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
    uint32_t _timeoutMs = 60000;    // 60 second timeout per axis
    int _fullRotationSteps = 9600;
    int _maxRotations = 3;          // Number of full rotations to command for seeking
    int _seekOffDir = 1;            // Direction to seek off trigger region (CW = +1)
    int _seekEdgeDir = -1;          // Direction to seek edges (CCW = -1)
    uint32_t _settleDelayMs = 50;   // Delay after stop to let motor settle
    int _positionTolerance = 10;    // Tolerance for position comparison

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
    AxisStepsDataType readAxisPosition(int axis, bool& isFresh);
    void stopMotion();
    void setAxisHome(int axis);
    void setError(const String& errStr);
    void resetState();
    void sendRotate(int axis, int dir);
    void sendMoveTo(int axis, AxisStepsDataType targetPos, AxisStepsDataType currentPos);

    // Debug
    static constexpr const char* MODULE_PREFIX = "HomingSeekCenter";
};
