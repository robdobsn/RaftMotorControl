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

#include "HomingSeekCenter.h"
#include "MotionArgs.h"
#include "RaftUtils.h"
#include "RaftJson.h"
#include "Logger.h"
#include "AxesParams.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
/// @param pNamedValueProvider Named value provider for accessing system state
/// @param motionControl Motion control interface
HomingSeekCenter::HomingSeekCenter(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
    : MotionPatternBase(pNamedValueProvider, motionControl)
{
    resetState();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
HomingSeekCenter::~HomingSeekCenter()
{
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Setup pattern with optional parameters
/// @param pParamsJson JSON parameters for pattern configuration
void HomingSeekCenter::setup(const char* pParamsJson)
{
    // Compute steps for full rotation based on axes parameters
    AxesParams axesParams = _motionControl.getAxesParams();
    if (axesParams.getNumAxes() == 0)
    {
        setError("No axes defined in parameters");
        return;
    }
    _numAxes = axesParams.getNumAxes();
    uint32_t masterAxisIdx = axesParams.getMasterAxisIdx();
    if (masterAxisIdx >= axesParams.getNumAxes())
    {
        setError("Invalid master axis index in parameters");
        return;
    }

    // Steps for a full rotation of the master axis (used for seeking calculations)
    _fullRotationSteps = axesParams.getStepsPerRot(masterAxisIdx);

    // Parse configuration
    uint32_t stepsPerSecOverride = 0;
    if (pParamsJson)
    {
        RaftJson config(pParamsJson);
        _numAxes = config.getInt("numAxes", _numAxes);
        // Home base axis (0) first, most-distal axis last (ascending). On a coupled
        // arm the distal axis MUST be homed last so nothing disturbs it afterwards -
        // see SET_HOME. Override "startAxis" only for testing a single axis.
        _startAxis = config.getInt("startAxis", 0);
        _fullRotationSteps = config.getInt("fullRotationSteps", _fullRotationSteps);
        stepsPerSecOverride = config.getInt("stepsPerSec", stepsPerSecOverride);
        _timeoutMs = config.getInt("timeoutMs", _timeoutMs);
        _seekOffDir = config.getInt("seekOffDir", _seekOffDir);
        _seekEdgeDir = config.getInt("seekEdgeDir", _seekEdgeDir);
        _settleDelayMs = config.getInt("settleDelayMs", _settleDelayMs);
        _positionTolerance = config.getInt("positionTolerance", _positionTolerance);
        _setHomeHereMode = config.getBool("setHomeHere", false);
        _persistOffset = config.getBool("persist", true);
    }

    // Get max steps per second for homing, and the per-axis home offset (steps)
    uint32_t maxStepsPerSec = 0;
    _stepsPerSecPerAxis.clear();
    _homeOffsetStepsPerAxis.clear();
    for (int axisIdx = 0; axisIdx < axesParams.getNumAxes(); axisIdx++)
    {
        uint32_t stepsPerSec = stepsPerSecOverride > 0 ? stepsPerSecOverride : axesParams.getHomingStepRatePerSec(axisIdx);
        _stepsPerSecPerAxis.push_back(stepsPerSec);
        _homeOffsetStepsPerAxis.push_back(axesParams.getHomeOffsetSteps(axisIdx));
        if (maxStepsPerSec == 0 || stepsPerSec < maxStepsPerSec)
            maxStepsPerSec = stepsPerSec;
    }

    // The arms may have been moved by hand between homing actions, so step counts are
    // NOT assumed valid here. Zero the position frame at the start; homing then works
    // purely from freshly-measured switch positions with no stale-count dependence.
    _motionControl.setCurPositionAsOrigin();

    // setHomeHere calibration: capture the current (user-placed-centre) position NOW,
    // before any homing motion. Per-axis offset will be (centreStart - sensorMidpoint).
    _centreStartPos.assign(_numAxes, 0);
    _setHomeHereOffsets.assign(_numAxes, 0);
    if (_setHomeHereMode)
    {
        AxesValues<AxisStepsDataType> curSteps = _motionControl.getAxisTotalSteps();
        for (int i = 0; i < _numAxes; i++)
            _centreStartPos[i] = curSteps.getVal(i);
        LOG_I(MODULE_PREFIX, "setHomeHere ENABLED (persist=%d) centreStart[0]=%d centreStart[1]=%d - will calibrate home offsets from this position",
              _persistOffset, _numAxes > 0 ? (int)_centreStartPos[0] : 0, _numAxes > 1 ? (int)_centreStartPos[1] : 0);
    }

    // Start homing
    resetState();
    setState(State::START);
    _currentAxis = _startAxis;
    LOG_I(MODULE_PREFIX, "setup numAxes=%d, startAxis=%d, fullRotationSteps=%d, maxStepsPerSec=%d, timeout=%ums, seekOffDir=%d, seekEdgeDir=%d",
          _numAxes, _startAxis, _fullRotationSteps, maxStepsPerSec, _timeoutMs, _seekOffDir, _seekEdgeDir);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Service loop - called frequently to advance homing state machine
void HomingSeekCenter::loop()
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
            // Check initial end-stop state
            bool endStopTriggered = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh at start");
                return;
            }
            bool startPosFresh = false;
            AxisStepsDataType startPos = readAxisPosition(_currentAxis, startPosFresh);
            LOG_I(MODULE_PREFIX, "Axis %d: START pos=%d endstop=%s seekSps=%u seekOffDir=%d seekEdgeDir=%d (t=%u)",
                  _currentAxis, startPos, endStopTriggered ? "TRIGGERED" : "off",
                  (unsigned)_stepsPerSecPerAxis[_currentAxis], _seekOffDir, _seekEdgeDir, now);

            if (endStopTriggered) {
                // Currently in trigger region, need to exit first
                sendRotate(_currentAxis, _seekOffDir);
                LOG_I(MODULE_PREFIX, "Axis %d: End-stop triggered at start, rotating dir=%d to exit trigger region",
                      _currentAxis, _seekOffDir);
                setState(State::SEEK_OFF);
            } else {
                // Already outside trigger region, go directly to seeking edge A
                sendRotate(_currentAxis, _seekEdgeDir);
                LOG_I(MODULE_PREFIX, "Axis %d: End-stop NOT triggered at start, rotating dir=%d to find edge A",
                      _currentAxis, _seekEdgeDir);
                setState(State::SEEK_EDGE_A);
            }
            _timeoutStartMs = now;
            break;
        }

        case State::SEEK_OFF:
        {
            // Looking for end-stop to become NOT triggered (exit trigger region)
            bool endStopTriggered = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh during SEEK_OFF");
                return;
            }
            
            if (!endStopTriggered) {
                // Exited trigger region, now seek edge A in opposite direction
                bool pf = false;
                AxisStepsDataType p = readAxisPosition(_currentAxis, pf);
                stopMotion();
                sendRotate(_currentAxis, _seekEdgeDir);
                LOG_I(MODULE_PREFIX, "Axis %d: exited region at pos=%d, now seeking edge A dir=%d",
                      _currentAxis, p, _seekEdgeDir);
                setState(State::SEEK_EDGE_A);
            }
            break;
        }

        case State::SEEK_EDGE_A:
        {
            // Looking for end-stop to become triggered (entering trigger region)
            bool endStopTriggered = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh during SEEK_EDGE_A");
                return;
            }
            
            if (endStopTriggered) {
                // Found edge A (entering) - capture position AT DETECTION before stopping
                _detectPos = readAxisPosition(_currentAxis, isFresh);
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_A);
                LOG_I(MODULE_PREFIX, "Axis %d: edge A ENTER detected at pos=%d (t=%u), settling %ums",
                      _currentAxis, _detectPos, now, _settleDelayMs);
            }
            break;
        }

        case State::SETTLING_A:
        {
            // Wait for motor to settle, then record position
            if (now - _settleStartMs >= _settleDelayMs) {
                _edgeAPos = readAxisPosition(_currentAxis, isFresh);
                if (!isFresh) {
                    setError("Position not fresh during SETTLING_A");
                    return;
                }
                LOG_I(MODULE_PREFIX, "Axis %d: edge A settled pos=%d (detect=%d settleDrift=%d)",
                      _currentAxis, _edgeAPos, _detectPos, _edgeAPos - _detectPos);

                // Continue in same direction to find edge B
                sendRotate(_currentAxis, _seekEdgeDir);
                setState(State::SEEK_EDGE_B);
            }
            break;
        }

        case State::SEEK_EDGE_B:
        {
            // Looking for end-stop to become NOT triggered (leaving trigger region)
            bool endStopTriggered = readEndStop(_currentAxis, isFresh);
            if (!isFresh) {
                setError("End-stop not fresh during SEEK_EDGE_B");
                return;
            }
            
            if (!endStopTriggered) {
                // Found edge B (leaving) - capture position AT DETECTION before stopping
                _detectPos = readAxisPosition(_currentAxis, isFresh);
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_B);
                LOG_I(MODULE_PREFIX, "Axis %d: edge B LEAVE detected at pos=%d (t=%u), settling %ums",
                      _currentAxis, _detectPos, now, _settleDelayMs);
            }
            break;
        }

        case State::SETTLING_B:
        {
            // Wait for motor to settle, then record edge B and compute the midpoint
            if (now - _settleStartMs >= _settleDelayMs) {
                _edgeBPos = readAxisPosition(_currentAxis, isFresh);
                if (!isFresh) {
                    setError("Position not fresh during SETTLING_B");
                    return;
                }
                LOG_I(MODULE_PREFIX, "Axis %d: edge B settled pos=%d (detect=%d settleDrift=%d)",
                      _currentAxis, _edgeBPos, _detectPos, _edgeBPos - _detectPos);

                AxisStepsDataType sensorMid = (_edgeAPos + _edgeBPos) / 2;
                if (_setHomeHereMode)
                {
                    // Calibration: home = the user-placed centre captured at start; record
                    // the offset (centre - sensorMid) for persistence and future homes.
                    AxisStepsDataType centre = (_currentAxis < (int)_centreStartPos.size())
                                                   ? _centreStartPos[_currentAxis] : sensorMid;
                    AxisStepsDataType offset = centre - sensorMid;
                    if (_currentAxis < (int)_setHomeHereOffsets.size())
                        _setHomeHereOffsets[_currentAxis] = offset;
                    _midPoint = centre;
                    LOG_I(MODULE_PREFIX, "Axis %d: setHomeHere A=%d B=%d sensorMid=%d centreStart=%d -> offset=%d (home target=centre)",
                          _currentAxis, _edgeAPos, _edgeBPos, sensorMid, centre, offset);
                }
                else
                {
                    // Normal: home = sensor-region midpoint + configured/effective offset.
                    AxisStepsDataType homeOffset = (_currentAxis < (int)_homeOffsetStepsPerAxis.size())
                                                       ? _homeOffsetStepsPerAxis[_currentAxis] : 0;
                    _midPoint = sensorMid + homeOffset;
                    LOG_I(MODULE_PREFIX, "Axis %d: edges A=%d B=%d width=%d sensorMid=%d homeOffset=%d -> home target=%d",
                          _currentAxis, _edgeAPos, _edgeBPos, _edgeAPos - _edgeBPos, sensorMid, homeOffset, _midPoint);
                }

                sendMoveTo(_currentAxis, _midPoint, _edgeBPos);
                setState(State::MOVE_TO_MIDPOINT);
            }
            break;
        }

        case State::MOVE_TO_MIDPOINT:
        {
            // Check if we've reached the midpoint
            AxisStepsDataType curPos = readAxisPosition(_currentAxis, isFresh);
            if (!isFresh) {
                setError("Position not fresh during MOVE_TO_MIDPOINT");
                return;
            }
            
            AxisStepsDataType delta = abs(_midPoint - curPos);
            if (delta < _positionTolerance) {
                // At midpoint, stop and settle
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_MID);
                LOG_I(MODULE_PREFIX, "Axis %d: Reached midpoint %d (cur=%d), settling",
                      _currentAxis, _midPoint, curPos);
            }
            break;
        }

        case State::SETTLING_MID:
        {
            // Wait for motor to settle at midpoint
            if (now - _settleStartMs >= _settleDelayMs) {
                // Set this axis home (origin)
                setAxisHome(_currentAxis);
                _motionControl.setAxisHomed(_currentAxis, true);
                setState(State::SET_HOME);
                LOG_I(MODULE_PREFIX, "Axis %d: Set as home (origin), marked homed", _currentAxis);
            }
            break;
        }

        case State::SET_HOME:
        {
            // Advance to the next axis (ascending: base -> tip) or complete.
            //
            // Base-first ordering is REQUIRED on mechanically-coupled arms (this
            // single-arm SCARA rotates the elbow/axis-1 when the shoulder/axis-0
            // turns). Homing the most-distal axis LAST guarantees no later axis move
            // can drag it off the switch it was just homed to. The previous logic
            // homed startAxis then forced axis 0 last, so on this arm the elbow was
            // displaced by however far the shoulder had to travel to find its own
            // switch - a non-repeatable home that drifted with shoulder travel.
            if (_currentAxis < _numAxes - 1) {
                _currentAxis++;
                setState(State::START);
                LOG_I(MODULE_PREFIX, "Switching to axis %d", _currentAxis);
            } else {
                // setHomeHere calibration: apply & (optionally) persist the measured offsets
                // so future normal homes land at the user-defined centre.
                if (_setHomeHereMode)
                {
                    LOG_I(MODULE_PREFIX, "setHomeHere complete - applying offsets [%d,%d] persist=%d",
                          _numAxes > 0 ? (int)_setHomeHereOffsets[0] : 0,
                          _numAxes > 1 ? (int)_setHomeHereOffsets[1] : 0, _persistOffset);
                    _motionControl.applyHomeOffsetsSteps(_setHomeHereOffsets, _persistOffset);
                }
                // Diagnostic: log where every axis ended up and whether it is still on
                // its switch (advisory only during bring-up - does NOT abort).
                logHomingCompletionState();
                // All axes homed - each axis is now at its home position (step count 0)
                // Set the current position as the origin (this updates both step and Cartesian tracking)
                _motionControl.setCurPositionAsOrigin();
                setState(State::COMPLETE);
                LOG_I(MODULE_PREFIX, "Homing complete - all axes at origin");
                _motionControl.stopPattern();
            }
            break;
        }

        default:
            setError("Unknown state");
            break;
    }

    // Timeout check (skip for settling and complete states)
    if (_state != State::IDLE && _state != State::COMPLETE && _state != State::ERROR &&
        _state != State::SETTLING_A && _state != State::SETTLING_B && _state != State::SETTLING_MID) {
        if (now - _timeoutStartMs > _timeoutMs) {
            setError("Homing timeout");
            stopMotion();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Read the end-stop state for a given axis
/// @param axis Axis index
/// @param isFresh Set to true if the value is fresh
/// @return true if end-stop is triggered, false otherwise
bool HomingSeekCenter::readEndStop(int axis, bool& isFresh)
{
    return _motionControl.getEndStopState(axis, false, isFresh); // min endstop
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Log where each axis came to rest and whether it is still on its endstop.
/// @note Advisory diagnostic at homing completion. Each axis homes to the midpoint of
///       its trigger region, so on a correct home every endstop reads triggered here.
///       An axis reading not-triggered means it ended up off its switch (e.g. dragged
///       by a later axis move on a coupled arm).
void HomingSeekCenter::logHomingCompletionState()
{
    for (int axis = 0; axis < _numAxes; axis++)
    {
        bool esFresh = false;
        bool triggered = readEndStop(axis, esFresh);
        bool posFresh = false;
        AxisStepsDataType pos = readAxisPosition(axis, posFresh);
        LOG_I(MODULE_PREFIX, "Completion: axis %d pos=%d endstop=%s (esFresh=%d)",
              axis, pos, triggered ? "TRIGGERED" : "off", esFresh ? 1 : 0);
        if (esFresh && !triggered)
            LOG_W(MODULE_PREFIX, "Completion: axis %d is NOT on its endstop (off-switch / disturbed)", axis);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Read the current position for a given axis
/// @param axis Axis index
/// @param isFresh Set to true if the value is fresh
/// @return Current position value
AxisStepsDataType HomingSeekCenter::readAxisPosition(int axis, bool& isFresh)
{
    AxesValues<AxisStepsDataType> pos = _motionControl.getAxisTotalSteps();
    isFresh = true; // Position is always fresh from motion controller
    return pos.getVal(axis);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Stop motion and clear pending moves
void HomingSeekCenter::stopMotion()
{
    _motionControl.stopAndClear();
    LOG_D(MODULE_PREFIX, "stopMotion: stopped and cleared");
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Set the current position as home (origin) for the given axis
/// @param axis Axis index
void HomingSeekCenter::setAxisHome(int axis)
{
    // Use per-axis origin setting
    _motionControl.setAxisOrigin(axis);
    LOG_I(MODULE_PREFIX, "setAxisHome axis %d: set as origin", axis);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Set an error state and store the error string
/// @param errStr Error message
void HomingSeekCenter::setError(const String& errStr)
{
    _lastError = errStr;
    _state = State::ERROR;
    LOG_E(MODULE_PREFIX, "Error: %s", errStr.c_str());
    _motionControl.stopPattern();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Reset the state machine and all tracking variables
void HomingSeekCenter::resetState()
{
    _state = State::IDLE;
    _currentAxis = _startAxis;
    _edgeAPos = 0.0;
    _edgeBPos = 0.0;
    _midPoint = 0.0;
    _timeoutStartMs = 0;
    _settleStartMs = 0;
    _lastError = "";
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Send a rotate command for a full rotation in the given direction
void HomingSeekCenter::sendRotate(int axis, int dir, uint32_t stepsPerSec)
{
    // Command multiple rotations to allow time to find edge
    int steps = _fullRotationSteps * _maxRotations * dir;
    uint32_t sps = (stepsPerSec > 0) ? stepsPerSec : _stepsPerSecPerAxis[axis];

    MotionArgs args;
    args.clear();
    args.setMode("pos-rel-steps");  // Relative steps (non-ramped, bypasses kinematics)
    args.setSpeed(String(sps) + "sps");  // Steps per second
    args.setDoNotSplitMove(true);

    AxesValues<AxisPosDataType>& axisVals = args.getAxesPos();
    axisVals.setVal(axis, steps);
    args.getAxesSpecified().setVal(axis, true);

    _motionControl.moveTo(args);
    LOG_I(MODULE_PREFIX, "sendRotate axis %d dir %d steps %d sps %u (maxRot=%d)", axis, dir, steps, (unsigned)sps, _maxRotations);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Send a move-to command to move the axis to the target position
/// @param axis The axis to move
/// @param targetPos Target position in units (degrees)
/// @param currentPos Current position in units (degrees)
void HomingSeekCenter::sendMoveTo(int axis, AxisStepsDataType targetPos, AxisStepsDataType currentPos)
{
    // Calculate relative movement in units, then convert to steps
    AxisStepsDataType relativeSteps = targetPos - currentPos;

    MotionArgs args;
    args.clear();
    args.setMode("pos-rel-steps");  // Relative steps (bypasses kinematics)
    args.setSpeed(String(_stepsPerSecPerAxis[axis]) + "sps");  // Steps per second
    LOG_I(MODULE_PREFIX,"sendMoveTo axis %d speed %d stepsPerSec targetPos %d steps currentPos %d steps relativeSteps %d",
        axis, (int)(_stepsPerSecPerAxis[axis]), targetPos, currentPos, relativeSteps);

    AxesValues<AxisPosDataType>& posVals = args.getAxesPos();
    posVals.setVal(axis, relativeSteps);
    args.getAxesSpecified().setVal(axis, true);

    _motionControl.moveTo(args);
    LOG_I(MODULE_PREFIX, "sendMoveTo axis %d target %d from %d steps %d", axis, targetPos, currentPos, relativeSteps);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Factory function for creating homing pattern instances
/// @param pNamedValueProvider Named value provider
/// @param motionControl Motion control interface
/// @return Pointer to new HomingSeekCenter instance
MotionPatternBase* HomingSeekCenter::create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
{
    return new HomingSeekCenter(pNamedValueProvider, motionControl);
}
