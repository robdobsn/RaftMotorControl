/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// HomingPattern
// Motion pattern for homing axes using endstops
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "MotionPatternBase.h"
#include "MotionControlIF.h"
#include "RaftArduino.h"

class HomingPattern : public MotionPatternBase
{
public:
    /// @brief Constructor
    /// @param pNamedValueProvider Named value provider for accessing system state
    /// @param motionControl Motion control interface
    HomingPattern(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

    /// @brief Destructor
    virtual ~HomingPattern();

    /// @brief Setup pattern with optional parameters
    /// @param pParamsJson JSON parameters for pattern configuration
    virtual void setup(const char* pParamsJson = nullptr) override;

    /// @brief Service loop - called frequently to advance homing state machine
    virtual void loop() override;

    /// @brief Factory function for creating homing pattern instances
    /// @param pNamedValueProvider Named value provider
    /// @param motionControl Motion control interface
    /// @return Pointer to new HomingPattern instance
    static MotionPatternBase* create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

private:
    // State machine states
    enum class State {
        IDLE,
        START,
        FIND_EDGE_1,
        FIND_EDGE_2,
        MOVE_TO_MIDPOINT,
        SET_HOME,
        NEXT_AXIS,
        COMPLETE,
        ERROR
    };

    // State machine
    State _state = State::IDLE;

    // Axis being homed
    int _currentAxis = 0;

    // Number of axes to home (from config)
    int _numAxes = 2;

    // Starting axis (from config, defaults to last axis first)
    int _startAxis = 1;

    // Position tracking
    double _edge1Pos = 0.0;
    double _edge2Pos = 0.0;
    double _midPoint = 0.0;

    // Configuration parameters
    int _feedrate = 5;
    uint32_t _timeoutMs = 10000;
    int _fullRotationSteps = 9600;

    // Timing
    uint32_t _timeoutStartMs = 0;

    // End-stop state
    bool _initialEndStopActive = false;

    // Error string
    String _lastError = "";

    // Internal helpers
    bool readEndStop(int axis, bool& isFresh);
    double readAxisPosition(int axis, bool& isFresh);
    void sendStop();
    void setHome(int axis);
    void setError(const String& errStr);
    void resetState();
    void sendRotate(int axis, int dir);
    void sendMoveTo(int axis, double pos, bool homing = false);
    void sendMoveToOrigin();
    void sendSetPositionZero(int axis);

    // Debug
    static constexpr const char* MODULE_PREFIX = "HomingPattern";
};
