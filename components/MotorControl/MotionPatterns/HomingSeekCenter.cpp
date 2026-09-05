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
        _reapproachSteps = config.getInt("reapproachSteps", _reapproachSteps);
    }

    // Get max steps per second for homing, and the per-axis home offset (steps)
    uint32_t maxStepsPerSec = 0;
    _stepsPerSecPerAxis.clear();
    _homeOffsetStepsPerAxis.clear();
    _seekOffClearStepsPerAxis.clear();
    for (int axisIdx = 0; axisIdx < axesParams.getNumAxes(); axisIdx++)
    {
        uint32_t stepsPerSec = stepsPerSecOverride > 0 ? stepsPerSecOverride : axesParams.getHomingStepRatePerSec(axisIdx);
        _stepsPerSecPerAxis.push_back(stepsPerSec);
        _homeOffsetStepsPerAxis.push_back(axesParams.getHomeOffsetSteps(axisIdx));
        _seekOffClearStepsPerAxis.push_back(axesParams.getSeekOffClearSteps(axisIdx));
        if (maxStepsPerSec == 0 || stepsPerSec < maxStepsPerSec)
            maxStepsPerSec = stepsPerSec;
    }

    // The arms may have been moved by hand between homing actions, so step counts are
    // NOT assumed valid here. Zero the position frame at the start; homing then works
    // purely from freshly-measured switch positions with no stale-count dependence.
    // Position is NOT yet certain — pass false so a mid-homing failure doesn't leave
    // the firmware thinking the arm is at a known origin.
    _motionControl.setCurPositionAsOrigin(false);

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
                armEdgeCapture(_currentAxis);
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
                // Exited trigger region. If the axis is configured with a
                // clear-off margin, keep going the same way for that many extra
                // steps before flipping direction — otherwise SEEK_EDGE_A can
                // re-trigger on the same edge we just released, at a position
                // dominated by mechanical settling rather than true region width.
                bool pf = false;
                AxisStepsDataType p = readAxisPosition(_currentAxis, pf);
                AxisStepsDataType clearSteps = (_currentAxis < (int)_seekOffClearStepsPerAxis.size())
                                                   ? _seekOffClearStepsPerAxis[_currentAxis] : 0;
                if (clearSteps > 0) {
                    _seekOffClearStartPos = p;
                    LOG_I(MODULE_PREFIX, "Axis %d: exited region at pos=%d, clearing extra %d steps dir=%d",
                          _currentAxis, p, (int)clearSteps, _seekOffDir);
                    // Keep rotating in the same direction; SEEK_OFF_CLEAR watches for the margin.
                    setState(State::SEEK_OFF_CLEAR);
                } else {
                    stopMotion();
                    armEdgeCapture(_currentAxis);
                    sendRotate(_currentAxis, _seekEdgeDir);
                    LOG_I(MODULE_PREFIX, "Axis %d: exited region at pos=%d, now seeking edge A dir=%d",
                          _currentAxis, p, _seekEdgeDir);
                    setState(State::SEEK_EDGE_A);
                }
            }
            break;
        }

        case State::SEEK_OFF_CLEAR:
        {
            // Continue in `seekOffDir` until we've cleared the configured margin
            // past where the endstop released. Guards against the arm starting
            // just inside the region near the release edge, which used to inflate
            // sensor-width variability by up to 5° on the coupled axis.
            bool pf = false;
            AxisStepsDataType p = readAxisPosition(_currentAxis, pf);
            if (!pf) {
                setError("Position not fresh during SEEK_OFF_CLEAR");
                return;
            }
            AxisStepsDataType clearSteps = (_currentAxis < (int)_seekOffClearStepsPerAxis.size())
                                               ? _seekOffClearStepsPerAxis[_currentAxis] : 0;
            if (abs(p - _seekOffClearStartPos) >= clearSteps) {
                stopMotion();
                armEdgeCapture(_currentAxis);
                sendRotate(_currentAxis, _seekEdgeDir);
                LOG_I(MODULE_PREFIX, "Axis %d: clear-off complete at pos=%d (start=%d), now seeking edge A dir=%d",
                      _currentAxis, p, _seekOffClearStartPos, _seekEdgeDir);
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
                // Prefer the ISR-latched transition position: the poll above only
                // sees the edge at the next service tick, which biases _detectPos
                // by up to (seek rate x loop interval) steps.
                _edgeCapturedPos = _detectPos;
                _edgeCapturedValid = takeEdgeCapture(_edgeCapturedPos);

                // First contact with the edge: back off a fixed margin and come
                // back at it, so the measured approach is identical on every home
                // (see _reapproachSteps). Without this the amount of lost motion
                // already taken up on arrival varies, which is what put the
                // measured sensor width into two clusters ~27 steps apart.
                if ((_reapproachSteps > 0) && !_reapproachDone) {
                    _reapproachStartPos = _edgeCapturedPos;
                    stopMotion();
                    sendRotate(_currentAxis, _seekOffDir);
                    LOG_I(MODULE_PREFIX, "Axis %d: edge A located at pos=%d, backing off %d steps dir=%d to re-approach",
                          _currentAxis, _edgeCapturedPos, (int)_reapproachSteps, _seekOffDir);
                    setState(State::EDGE_A_BACKOFF);
                    break;
                }
                _edgeCaptureQuantErr = _edgeCapturedValid ?
                        (uint32_t)abs(_detectPos - _edgeCapturedPos) : 0;
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_A);
                LOG_I(MODULE_PREFIX, "Axis %d: edge A ENTER polled pos=%d captured pos=%d (%s, quantErr=%u) (t=%u), settling %ums",
                      _currentAxis, _detectPos, _edgeCapturedPos,
                      _edgeCapturedValid ? "ISR" : "POLL-FALLBACK",
                      (unsigned)_edgeCaptureQuantErr, now, _settleDelayMs);
            }
            break;
        }

        case State::EDGE_A_BACKOFF:
        {
            // Retreat a fixed margin past the located edge, then re-approach it.
            bool pf = false;
            AxisStepsDataType p = readAxisPosition(_currentAxis, pf);
            if (!pf) {
                setError("Position not fresh during EDGE_A_BACKOFF");
                return;
            }
            if (abs(p - _reapproachStartPos) < _reapproachSteps)
                break;
            stopMotion();
            _reapproachDone = true;
            _crossSawEnter = false;
            armEdgeCapture(_currentAxis);
            sendRotate(_currentAxis, _seekEdgeDir);
            LOG_I(MODULE_PREFIX, "Axis %d: backed off to pos=%d, continuous crossing dir=%d (run-up %d steps, no stop at edge A)",
                  _currentAxis, p, _seekEdgeDir, (int)_reapproachSteps);
            setState(State::SEEK_CROSS);
            break;
        }

        case State::SEEK_CROSS:
        {
            // One continuous pass: take BOTH edges from the ISR capture queue
            // without ever stopping inside the region.
            AxisStepsDataType pos = 0;
            bool newState = false;
            while (_motionControl.popEndStopEdge(pos, newState))
            {
                if (newState && !_crossSawEnter) {
                    _crossSawEnter = true;
                    _edgeAPos = pos;
                    LOG_I(MODULE_PREFIX, "Axis %d: CROSS edge A ENTER pos=%d (continuous)",
                          _currentAxis, (int)_edgeAPos);
                } else if (!newState && _crossSawEnter) {
                    _edgeBPos = pos;
                    stopMotion();
                    _settleStartMs = now;
                    LOG_I(MODULE_PREFIX, "Axis %d: CROSS edge B LEAVE pos=%d (continuous) width=%d",
                          _currentAxis, (int)_edgeBPos, (int)abs(_edgeAPos - _edgeBPos));
                    setState(State::SETTLING_B);
                    break;
                }
            }
            break;
        }

        case State::SETTLING_A:
        {
            // Wait for motor to settle, then record position
            if (now - _settleStartMs >= _settleDelayMs) {
                AxisStepsDataType settledPos = readAxisPosition(_currentAxis, isFresh);
                if (!isFresh) {
                    setError("Position not fresh during SETTLING_A");
                    return;
                }
                // The edge position is where the sensor changed state, NOT where
                // the axis came to rest - use the ISR capture when we have it.
                _edgeAPos = _edgeCapturedValid ? _edgeCapturedPos : settledPos;
                LOG_I(MODULE_PREFIX, "Axis %d: edge A settled pos=%d -> edgeA=%d (%s) (detect=%d settleDrift=%d)",
                      _currentAxis, settledPos, _edgeAPos,
                      _edgeCapturedValid ? "ISR" : "POLL-FALLBACK",
                      _detectPos, settledPos - _detectPos);

                // Continue in same direction to find edge B
                armEdgeCapture(_currentAxis);
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
                _edgeCapturedPos = _detectPos;
                _edgeCapturedValid = takeEdgeCapture(_edgeCapturedPos);
                _edgeCaptureQuantErr = _edgeCapturedValid ?
                        (uint32_t)abs(_detectPos - _edgeCapturedPos) : 0;
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_B);
                LOG_I(MODULE_PREFIX, "Axis %d: edge B LEAVE polled pos=%d captured pos=%d (%s, quantErr=%u) (t=%u), settling %ums",
                      _currentAxis, _detectPos, _edgeCapturedPos,
                      _edgeCapturedValid ? "ISR" : "POLL-FALLBACK",
                      (unsigned)_edgeCaptureQuantErr, now, _settleDelayMs);
            }
            break;
        }

        case State::SETTLING_B:
        {
            // Wait for motor to settle, then record edge B and compute the midpoint
            if (now - _settleStartMs >= _settleDelayMs) {
                AxisStepsDataType settledPosB = readAxisPosition(_currentAxis, isFresh);
                if (!isFresh) {
                    setError("Position not fresh during SETTLING_B");
                    return;
                }
                // When we came through SEEK_CROSS both edges are already set from
                // the capture queue - do NOT overwrite them with a settled read.
                if (!_crossSawEnter)
                    _edgeBPos = _edgeCapturedValid ? _edgeCapturedPos : settledPosB;
                // Capture is finished for this axis - stop the ISR sampling
                _motionControl.disarmEndStopEdgeCapture();
                LOG_I(MODULE_PREFIX, "Axis %d: edge B settled pos=%d -> edgeB=%d (%s) (detect=%d settleDrift=%d)",
                      _currentAxis, settledPosB, _edgeBPos,
                      _crossSawEnter ? "ISR-CROSS" : (_edgeCapturedValid ? "ISR" : "POLL-FALLBACK"),
                      _detectPos, settledPosB - _detectPos);

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
                _moveLastPos = _edgeBPos;
                _moveLastChangeMs = now;
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
                // Fast path: landed inside tolerance
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_MID);
                LOG_I(MODULE_PREFIX, "Axis %d: Reached midpoint %d (cur=%d delta=%d), settling",
                      _currentAxis, _midPoint, curPos, (int)delta);
                break;
            }
            // Otherwise complete once motion has actually stopped. The axis
            // commonly rests 6-9 steps from target, so a tolerance-only check
            // hangs whenever it rests slightly further out.
            if (curPos != _moveLastPos) {
                _moveLastPos = curPos;
                _moveLastChangeMs = now;
                break;
            }
            if (now - _moveLastChangeMs >= MOVE_STOPPED_MS) {
                stopMotion();
                _settleStartMs = now;
                setState(State::SETTLING_MID);
                LOG_W(MODULE_PREFIX, "Axis %d: midpoint move STOPPED %d steps short of %d (cur=%d) - accepting",
                      _currentAxis, (int)delta, _midPoint, curPos);
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
                _reapproachDone = false;    // each axis gets its own re-approach
                _crossSawEnter = false;
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
/// @brief Arm step-rate capture of the next end-stop transition on an axis
/// @param axis Axis index
void HomingSeekCenter::armEdgeCapture(int axis)
{
    _edgeCapturedValid = false;
    _edgeCapturedPos = 0;
    _motionControl.armEndStopEdgeCapture(axis, false); // min endstop
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Read the step-exact position of the captured end-stop transition
/// @param posOut (out) latched position, untouched if no capture available
/// @return true if the ISR captured a transition since arming
/// @note Falls back to the caller's polled position when nothing was captured
///       (e.g. the transition happened while the ramp generator was not
///       ticking), so behaviour degrades to the previous accuracy rather than
///       failing the home.
bool HomingSeekCenter::takeEdgeCapture(AxisStepsDataType& posOut)
{
    uint32_t seq = 0;
    AxisStepsDataType pos = 0;
    bool newState = false;
    if (!_motionControl.getEndStopEdgeCapture(seq, pos, newState))
        return false;
    posOut = pos;
    return true;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Log where each axis came to rest and whether it is still on its endstop.
/// @note Advisory diagnostic at homing completion. Each axis homes to
///       `sensorMid + homeOffset`. When `homeOffset` is zero the axis ends on
///       its trigger switch; once a centre calibration is persisted (`setHomeHere`)
///       the offset can be many hundreds or thousands of steps, so the axis is
///       *intentionally* off-switch at rest. Both cases are normal — this log is
///       purely informational, downstream state is determined by
///       `logHomingCompletionState` in aggregate with the edge/width instrumentation.
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
        // Note: previously emitted LOG_W when off-switch, but with a non-zero
        // homeOffset that's the expected resting state — the warning was a
        // guaranteed false positive after centre calibration.
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
    // Abandoning the home - don't leave the ISR sampling an end-stop
    _motionControl.disarmEndStopEdgeCapture();
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
    _reapproachDone = false;
    _crossSawEnter = false;
    // Never leave the ISR sampling an end-stop once homing is over
    _motionControl.disarmEndStopEdgeCapture();
    _edgeCapturedValid = false;
    _edgeCapturedPos = 0;
    _edgeCaptureQuantErr = 0;
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
