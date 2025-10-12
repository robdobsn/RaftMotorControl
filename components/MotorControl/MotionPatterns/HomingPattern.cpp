/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// HomingPattern
// Motion pattern for homing axes using endstops
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "HomingPattern.h"
#include "MotionArgs.h"
#include "RaftUtils.h"
#include "RaftJson.h"
#include "Logger.h"
#include <math.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
/// @param pNamedValueProvider Named value provider for accessing system state
/// @param motionControl Motion control interface
HomingPattern::HomingPattern(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
    : MotionPatternBase(pNamedValueProvider, motionControl)
{
    resetState();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
HomingPattern::~HomingPattern()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup pattern with optional parameters
/// @param pParamsJson JSON parameters for pattern configuration
void HomingPattern::setup(const char* pParamsJson)
{
    // Parse configuration
    if (pParamsJson)
    {
        RaftJson config(pParamsJson);
        _feedrate = config.getInt("feedrate", _feedrate);
        _timeoutMs = config.getInt("timeoutMs", _timeoutMs);
        _fullRotationSteps = config.getInt("fullRotationSteps", _fullRotationSteps);
        _numAxes = config.getInt("numAxes", _numAxes);
        _startAxis = config.getInt("startAxis", _numAxes - 1); // Default to last axis
    }

    // Start homing
    resetState();
    _state = State::START;
    _currentAxis = _startAxis;
    LOG_I(MODULE_PREFIX, "Homing started: numAxes=%d, startAxis=%d, feedrate=%d, timeout=%ums",
          _numAxes, _startAxis, _feedrate, _timeoutMs);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Service loop - called frequently to advance homing state machine
void HomingPattern::loop()
{
    uint32_t now = millis();
    bool isFresh = false;

    switch (_state)
    {
        case State::IDLE:
        case State::COMPLETE:
        case State::ERROR:
            return;

        case State::START:
        {
            // Read initial end-stop state
            _initialEndStopActive = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh at start");
                return;
            }
            if (_initialEndStopActive) {
                // If active, rotate positive to find inactive (leaving region)
                sendRotate(_currentAxis, +1);
                LOG_I(MODULE_PREFIX, "Axis %d: End-stop active at start, rotating +1 to find inactive", _currentAxis);
            } else {
                // If inactive, rotate positive to find active (entering region)
                sendRotate(_currentAxis, +1);
                LOG_I(MODULE_PREFIX, "Axis %d: End-stop inactive at start, rotating +1 to find active", _currentAxis);
            }
            _state = State::FIND_EDGE_1;
            break;
        }

        case State::FIND_EDGE_1:
        {
            bool endStop = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh during edge 1");
                return;
            }
            if (_initialEndStopActive) {
                // Looking for transition from active to inactive
                if (!endStop) {
                    sendStop();
                    _edge1Pos = readAxisPosition(_currentAxis, isFresh);
                    // Now reverse direction to find active again
                    sendRotate(_currentAxis, -1);
                    LOG_I(MODULE_PREFIX, "Axis %d: Found leaving edge at pos %f, reversing to find entering edge", _currentAxis, _edge1Pos);
                    _state = State::FIND_EDGE_2;
                }
            } else {
                // Looking for transition from inactive to active
                if (endStop) {
                    _edge1Pos = readAxisPosition(_currentAxis, isFresh);
                    LOG_I(MODULE_PREFIX, "Axis %d: Found entering edge at pos %f, continuing to find leaving edge", _currentAxis, _edge1Pos);
                    _state = State::FIND_EDGE_2;
                }
            }
            break;
        }

        case State::FIND_EDGE_2:
        {
            bool endStop = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh during edge 2");
                return;
            }
            if (_initialEndStopActive) {
                // Looking for transition from inactive to active (reverse direction)
                if (endStop) {
                    sendStop();
                    _edge2Pos = readAxisPosition(_currentAxis, isFresh);
                    LOG_I(MODULE_PREFIX, "Axis %d: Found entering edge at pos %f", _currentAxis, _edge2Pos);
                    // Move to midpoint
                    _midPoint = (_edge1Pos + _edge2Pos) / 2.0;
                    sendMoveTo(_currentAxis, _midPoint, true);
                    _state = State::MOVE_TO_MIDPOINT;
                }
            } else {
                // Looking for transition from active to inactive (same direction)
                if (!endStop) {
                    sendStop();
                    _edge2Pos = readAxisPosition(_currentAxis, isFresh);
                    LOG_I(MODULE_PREFIX, "Axis %d: Found leaving edge at pos %f", _currentAxis, _edge2Pos);
                    // Move to midpoint
                    _midPoint = (_edge1Pos + _edge2Pos) / 2.0;
                    sendMoveTo(_currentAxis, _midPoint, true);
                    _state = State::MOVE_TO_MIDPOINT;
                }
            }
            break;
        }

        case State::MOVE_TO_MIDPOINT:
        {
            // Check if at midpoint
            double curPos = readAxisPosition(_currentAxis, isFresh);
            if (!isFresh) {
                setError("Position not fresh");
                return;
            }
            double delta = _midPoint - curPos;
            if (fabs(delta) < 0.5) {
                // At midpoint
                sendStop();
                sendSetPositionZero(_currentAxis);
                setHome(_currentAxis);
                _state = State::SET_HOME;
                LOG_I(MODULE_PREFIX, "Axis %d: At midpoint %f, set position zero and home", _currentAxis, _midPoint);
            }
            break;
        }

        case State::SET_HOME:
        {
            // Set home complete for this axis, move to next
            if (_currentAxis != 0) {
                _currentAxis = 0;
                _state = State::START;
                LOG_I(MODULE_PREFIX, "Switching to axis 0");
            } else {
                // All axes homed, move to origin (0,0)
                sendMoveToOrigin();
                _state = State::COMPLETE;
                LOG_I(MODULE_PREFIX, "Homing complete, moving to origin");
                // Stop pattern
                _motionControl.stopPattern();
            }
            break;
        }

        default:
            setError("Unknown state");
            break;
    }

    // Timeout check
    if (_state != State::IDLE && _state != State::COMPLETE && _state != State::ERROR) {
        if (_timeoutStartMs == 0)
            _timeoutStartMs = now;
        if (now - _timeoutStartMs > _timeoutMs) {
            setError("Homing timeout");
            sendStop();
        }
    } else {
        _timeoutStartMs = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Read the end-stop state for a given axis
/// @param axis Axis index
/// @param isFresh Set to true if the value is fresh
/// @return true if end-stop is triggered, false otherwise
bool HomingPattern::readEndStop(int axis, bool& isFresh)
{
    return _motionControl.getEndStopState(axis, false, isFresh); // min endstop
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Read the current position for a given axis
/// @param axis Axis index
/// @param isFresh Set to true if the value is fresh
/// @return Current position value
double HomingPattern::readAxisPosition(int axis, bool& isFresh)
{
    // Get position from motion control
    AxesValues<AxisPosDataType> pos = _motionControl.getLastMonitoredPos();
    isFresh = true; // Assume position is always fresh
    return pos.getVal(axis);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Send a rotate command for a full rotation in the given direction
void HomingPattern::sendRotate(int axis, int dir)
{
    // Create motion args for full rotation
    int steps = _fullRotationSteps * dir;

    MotionArgs args;
    args.clear();
    args.setFeedrate(_feedrate);
    args.setRelative(true);
    args.setNoSplit(true);
    args.setRamped(false);

    AxesValues<AxisStepsDataType> stepsVals;
    stepsVals.setVal(axis, steps);
    args.setSteps(stepsVals);

    _motionControl.moveTo(args);
    LOG_I(MODULE_PREFIX, "sendRotate axis %d dir %d steps %d", axis, dir, steps);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Send a move-to command to the specified position
void HomingPattern::sendMoveTo(int axis, double pos, bool homing)
{
    MotionArgs args;
    args.clear();
    args.setFeedrate(_feedrate);
    args.setRelative(false);

    AxesPosValues posList;
    posList.setVal(axis, pos);
    args.setPosn(posList);

    _motionControl.moveTo(args);
    LOG_I(MODULE_PREFIX, "sendMoveTo axis %d pos %f homing %d", axis, pos, homing);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Send a stop command to the motor controller
void HomingPattern::sendStop()
{
    _motionControl.pause(true);
    _motionControl.pause(false);
    LOG_D(MODULE_PREFIX, "sendStop");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set the current position as home (origin) for the given axis
/// @param axis Axis index
void HomingPattern::setHome(int axis)
{
    // Log current position before setting home
    bool isFresh = false;
    double curPos = readAxisPosition(axis, isFresh);
    LOG_I(MODULE_PREFIX, "setHome axis %d, current position before setHome: %f (fresh: %d)", axis, curPos, isFresh);

    // Set current position as origin for this axis
    _motionControl.setCurPositionAsOrigin(false, axis);

    LOG_I(MODULE_PREFIX, "setHome axis %d: set as origin", axis);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set an error state and store the error string
/// @param errStr Error message
void HomingPattern::setError(const String& errStr)
{
    _lastError = errStr;
    _state = State::ERROR;
    LOG_E(MODULE_PREFIX, "Error: %s", errStr.c_str());
    _motionControl.stopPattern();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Reset the state machine and all tracking variables
void HomingPattern::resetState()
{
    _state = State::IDLE;
    _currentAxis = _startAxis;
    _edge1Pos = 0.0;
    _edge2Pos = 0.0;
    _midPoint = 0.0;
    _timeoutStartMs = 0;
    _lastError = "";
    _initialEndStopActive = false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Send move to origin command
void HomingPattern::sendMoveToOrigin()
{
    // Move all axes to origin
    MotionArgs args;
    args.clear();
    args.setFeedrate(_feedrate);
    args.setRelative(false);

    AxesPosValues posList;
    for (int i = 0; i < _numAxes; i++)
        posList.setVal(i, 0.0);
    args.setPosn(posList);

    _motionControl.moveTo(args);
    LOG_I(MODULE_PREFIX, "sendMoveToOrigin (all axes to 0)");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set position to zero for specified axis
void HomingPattern::sendSetPositionZero(int axis)
{
    // This is handled by setCurPositionAsOrigin in setHome()
    LOG_D(MODULE_PREFIX, "sendSetPositionZero axis %d (handled via setHome)", axis);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Factory function for creating homing pattern instances
/// @param pNamedValueProvider Named value provider
/// @param motionControl Motion control interface
/// @return Pointer to new HomingPattern instance
MotionPatternBase* HomingPattern::create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
{
    return new HomingPattern(pNamedValueProvider, motionControl);
}
