////////////////////////////////////////////////////////////////////////////////
//
// HomingSeekCenter - see HomingSeekCenter.h for the algorithm and why.
//
// Rob Dobson 2025
//
////////////////////////////////////////////////////////////////////////////////

#include "HomingSeekCenter.h"
#include "AxesParams.h"
#include "MotionArgs.h"
#include "RaftJson.h"
#include "RaftCore.h"

#define DEBUG_HOMING_STATES

////////////////////////////////////////////////////////////////////////////////
HomingSeekCenter::HomingSeekCenter(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
    : MotionPatternBase(pNamedValueProvider, motionControl)
{
}

HomingSeekCenter::~HomingSeekCenter()
{
    _motionControl.disarmEndStopEdgeCapture();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Setup
void HomingSeekCenter::setup(const char* pParamsJson)
{
    AxesParams axesParams = _motionControl.getAxesParams();
    _numAxes = axesParams.getNumAxes() < 2 ? axesParams.getNumAxes() : 2;
    _fullRotationSteps = axesParams.getStepsPerRot(0);

    uint32_t fastOverride = 0, slowOverride = 0;
    if (pParamsJson)
    {
        RaftJson config(pParamsJson);
        _numAxes = config.getInt("numAxes", _numAxes);
        _startAxis = config.getInt("startAxis", 0);
        _timeoutMs = config.getInt("timeoutMs", _timeoutMs);
        _seekDir = config.getInt("seekEdgeDir", _seekDir);
        _settleDelayMs = config.getInt("settleDelayMs", _settleDelayMs);
        _clearMarginSteps = config.getInt("clearMarginSteps", _clearMarginSteps);
        _maxFlagWidthSteps = config.getInt("maxFlagWidthSteps", _maxFlagWidthSteps);
        fastOverride = config.getInt("fastStepsPerSec", 0);
        slowOverride = config.getInt("slowStepsPerSec", 0);
    }

    // Fast rate: well above the low-speed stick-slip region (~50-70 deg/s on
    // this arm), because a slow approach is both noisy and pointless - the ISR
    // latches the edge position exactly regardless of approach speed.
    // Slow rate: only the CROSS needs to be slow, and only so that mechanical
    // and sensor lag contribute a small distance at the two edges.
    _fastStepsPerSec.clear();
    _slowStepsPerSec.clear();
    for (int axisIdx = 0; axisIdx < axesParams.getNumAxes(); axisIdx++)
    {
        uint32_t cfgRate = axesParams.getHomingStepRatePerSec(axisIdx);
        uint32_t fast = fastOverride ? fastOverride : (uint32_t)(_fullRotationSteps * 75.0 / 360.0);
        uint32_t slow = slowOverride ? slowOverride : (cfgRate ? cfgRate : (uint32_t)(_fullRotationSteps * 10.0 / 360.0));
        _fastStepsPerSec.push_back(fast);
        _slowStepsPerSec.push_back(slow);
        _homeOffsetStepsPerAxis.push_back(axesParams.getHomeOffsetSteps(axisIdx));
    }

    // The arm may have been moved by hand, so step counts are not trustworthy
    // as absolute positions - but they ARE a consistent relative frame, which
    // is all the edge arithmetic needs.
    _currentAxis = _startAxis;
    LOG_I(MODULE_PREFIX, "setup numAxes=%d startAxis=%d fullRot=%d fast=%u slow=%u sps seekDir=%d clearMargin=%d maxFlag=%d",
          _numAxes, _startAxis, _fullRotationSteps,
          (unsigned)_fastStepsPerSec[0], (unsigned)_slowStepsPerSec[0],
          _seekDir, (int)_clearMarginSteps, (int)_maxFlagWidthSteps);

    startAxis(_currentAxis);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Begin homing one axis
void HomingSeekCenter::startAxis(int axis)
{
    _gotEdgeA = _gotEdgeB = false;
    _edgeAPos = _edgeBPos = _midPos = 0;
    _approachReversing = false;

    bool isFresh = false;
    _startedInsideFlag = _motionControl.getEndStopState(axis, false, isFresh);
    if (!isFresh)
    {
        setError("end-stop not configured");
        return;
    }

    // Arm ISR edge capture for this axis for the whole sequence. Even the fast
    // approach's transition is then exact, which is what lets the approach be
    // fast at all.
    _motionControl.armEndStopEdgeCapture(axis, false);

    // Drain anything left in the capture queue from the previous axis or a
    // previous homing run. A stale transition here satisfies the "saw an edge"
    // test in FAST_APPROACH on the very first service call, so the state
    // machine steps forward before the arm has moved and homing then fails to
    // reach home at all - intermittently, depending on what was left behind.
    {
        AxisStepsDataType stalePos = 0;
        bool staleState = false;
        int drained = 0;
        while (_motionControl.popEndStopEdge(stalePos, staleState))
            drained++;
        if (drained)
            LOG_I(MODULE_PREFIX, "axis %d discarded %d stale edge(s)", axis, drained);
    }

    LOG_I(MODULE_PREFIX, "axis %d START pos=%d endstop=%s -> %s",
          axis, (int)axisPos(axis), _startedInsideFlag ? "TRIGGERED" : "clear",
          _startedInsideFlag ? "driving OUT of flag" : "driving IN to find flag");

    // Drive out of the flag if inside it, otherwise in to find it. Either way
    // the commanded distance is bounded - a full rotation is the most that can
    // ever be needed to find a flag on a rotary axis.
    int dir = _startedInsideFlag ? -_seekDir : _seekDir;
    moveSteps(axis, dir * _fullRotationSteps, _fastStepsPerSec[axis]);
    enterState(State::FAST_APPROACH);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Service loop
void HomingSeekCenter::loop()
{
    if (_state == State::IDLE || _state == State::COMPLETE || _state == State::ERROR)
        return;

    uint32_t now = millis();
    if (Raft::isTimeout(now, _stateEntryTimeMs, _timeoutMs))
    {
        stopMotion();
        setError("timeout");
        return;
    }

    const int axis = _currentAxis;

    switch (_state)
    {
        case State::FAST_APPROACH:
        {
            // Drain any ISR-captured transitions; the one we care about is the
            // change that means "we are now on the far side of an edge".
            AxisStepsDataType edgePos = 0;
            bool newState = false;
            bool sawEdge = false;
            while (_motionControl.popEndStopEdge(edgePos, newState))
                sawEdge = true;

            bool triggered = endStopTriggered(axis);

            if (!_approachReversing)
            {
                if (_startedInsideFlag)
                {
                    // Started inside: we want to be OUT. Done as soon as released.
                    if (!triggered && sawEdge)
                    {
                        stopMotion();
                        LOG_I(MODULE_PREFIX, "axis %d exited flag at %d, standing clear", axis, (int)edgePos);
                        // Stand clear of the edge so the cross starts outside.
                        moveSteps(axis, -_seekDir * _clearMarginSteps, _fastStepsPerSec[axis]);
                        _approachReversing = true;
                    }
                }
                else
                {
                    // Started outside: drive in until triggered, then reverse out.
                    if (triggered && sawEdge)
                    {
                        stopMotion();
                        LOG_I(MODULE_PREFIX, "axis %d found flag edge at %d, reversing out", axis, (int)edgePos);
                        // Reverse far enough to clear the edge plus the margin.
                        // The overshoot while stopping is unknown, so use the
                        // measured edge position as the reference rather than
                        // wherever the arm happened to halt.
                        // Reverse relative to the MEASURED edge, not to wherever
                        // the arm halted: the stopping distance from the approach
                        // rate is unknown and at 75 deg/s exceeded the old 250-step
                        // margin, leaving axis 1 still inside the flag.
                        AxisStepsDataType back = (axisPos(axis) - edgePos);
                        if (back < 0) back = -back;
                        moveSteps(axis, -_seekDir * (back + _clearMarginSteps), _fastStepsPerSec[axis]);
                        _approachReversing = true;
                    }
                }
                if (!_motionControl.isBusy() && !_approachReversing)
                {
                    setError(_startedInsideFlag ? "never left flag in one rotation"
                                                : "no flag found in one rotation");
                }
            }
            else if (!_motionControl.isBusy())
            {
                // Clear of the flag and stopped. Verify, then start the slow cross.
                if (endStopTriggered(axis))
                {
                    setError("still triggered after standing clear - clearMarginSteps too small");
                    return;
                }
                while (_motionControl.popEndStopEdge(edgePos, newState)) {}   // discard
                _gotEdgeA = _gotEdgeB = false;
                LOG_I(MODULE_PREFIX, "axis %d clear at %d, slow cross begins (%u sps)",
                      axis, (int)axisPos(axis), (unsigned)_slowStepsPerSec[axis]);
                moveSteps(axis, _seekDir * (_maxFlagWidthSteps + 2 * _clearMarginSteps),
                          _slowStepsPerSec[axis]);
                enterState(State::SLOW_CROSS);
            }
            break;
        }

        case State::SLOW_CROSS:
        {
            AxisStepsDataType edgePos = 0;
            bool newState = false;
            while (_motionControl.popEndStopEdge(edgePos, newState))
            {
                if (newState && !_gotEdgeA)
                {
                    _edgeAPos = edgePos;
                    _gotEdgeA = true;
                    LOG_I(MODULE_PREFIX, "axis %d EDGE A (enter) at %d", axis, (int)edgePos);
                }
                else if (!newState && _gotEdgeA && !_gotEdgeB)
                {
                    _edgeBPos = edgePos;
                    _gotEdgeB = true;
                    LOG_I(MODULE_PREFIX, "axis %d EDGE B (leave) at %d", axis, (int)edgePos);
                }
            }

            if (_gotEdgeA && _gotEdgeB)
            {
                // Stop the instant both edges are known - do not run on to the
                // end of the commanded distance.
                stopMotion();
                AxisStepsDataType width = _edgeBPos - _edgeAPos;
                if (width < 0) width = -width;
                // Home IS the sensor midpoint. No home offset is applied: the
                // requirement is that the axis finishes midway between its own
                // trigger points, and adding an offset here moved the target
                // outside the flag entirely (measured -248 against a true
                // midpoint of -56).
                // Park on the sensor MIDPOINT. The arm's geometric zero is a
                // per-axis calibration distance away, but it CANNOT be used as
                // the park point: axis 1's offset exceeds its flag half-width
                // (>280 of 561 steps), so parking there would leave its
                // end-stop untriggered. The offset therefore has to be applied
                // in the coordinate frame, not here - see docs SYSTEM_TESTING
                // 2.5. Currently NOT applied anywhere, so the position field
                // carries ~5.3 mm of distortion (0.73 mm with correct offsets).
                _midPos = (_edgeAPos + _edgeBPos) / 2;
                LOG_I(MODULE_PREFIX, "axis %d flag width %d steps (%.2f deg), midpoint %d",
                      axis, (int)width, width * 360.0 / _fullRotationSteps, (int)_midPos);
                _moveIssued = false;
                enterState(State::FAST_TO_MID);
                break;
            }

            if (!_motionControl.isBusy())
            {
                setError(_gotEdgeA ? "crossed flag but never left it (maxFlagWidthSteps too small)"
                                   : "slow cross found no flag");
            }
            break;
        }

        case State::FAST_TO_MID:
        {
            if (!_moveIssued)
            {
                // Wait for the stop from SLOW_CROSS to actually take effect,
                // otherwise the position read below is taken mid-deceleration.
                if (_motionControl.isBusy())
                    break;
                if (!Raft::isTimeout(millis(), _stateEntryTimeMs, _settleDelayMs))
                    break;
                AxisStepsDataType delta = _midPos - axisPos(axis);
                LOG_I(MODULE_PREFIX, "axis %d at %d, moving %d steps to midpoint %d",
                      axis, (int)axisPos(axis), (int)delta, (int)_midPos);
                // At the fast rate this short move overshoots the midpoint -
                // there is not enough distance to decelerate cleanly. It is a
                // few hundred steps, so the seek rate costs well under a second
                // and lands cleanly.
                if (delta != 0)
                    moveSteps(axis, delta, _slowStepsPerSec[axis]);
                _moveIssued = true;
                _moveIssuedMs = millis();
                break;
            }
            // A freshly queued move does not report isBusy() immediately, so
            // waiting on !isBusy() alone returns instantly and the end-stop
            // then gets read at the PRE-move position. Give the move time to
            // be picked up before believing that it has finished.
            if (!Raft::isTimeout(millis(), _moveIssuedMs, MOVE_PICKUP_MS))
                break;
            if (_motionControl.isBusy())
                break;
            enterState(State::VERIFY);
            break;
        }

        case State::VERIFY:
        {
            if (_motionControl.isBusy())
                break;
            if (!Raft::isTimeout(millis(), _stateEntryTimeMs, _settleDelayMs))
                break;

            // Acceptance test: the end-stop MUST be triggered at the midpoint.
            // If it is not, the edges were mismeasured and setting an origin
            // here would silently corrupt every subsequent move.
            if (!endStopTriggered(axis))
            {
                setError("end-stop NOT triggered at computed midpoint");
                return;
            }
            AxisStepsDataType err = axisPos(axis) - _midPos;
            // Park stays on the sensor midpoint; the geometric zero is
            // homeOffsetSteps away, so REPORT that offset here rather than
            // driving to it. Reported angles then agree with forward
            // kinematics while home remains the most repeatable point on the
            // sensor. Measured offsets: axis0 -307, axis1 -167 steps (fitted
            // from 103 camera poses; residual 5.27 -> 0.73 mm).
            // Origin is plain zero: the parked midpoint reports (0, 180) as
            // every test and the UI expect. The geometric offset lives in the
            // KINEMATICS (KinematicsSingleArmSCARA::calculateAnglesFromSteps),
            // not in the reported position - putting it here instead made home
            // report (-11.51, 173.74) and broke every (0,180) assertion.
            _motionControl.setAxisOrigin(axis);
            _motionControl.setAxisHomed(axis, true);
            LOG_I(MODULE_PREFIX, "axis %d HOMED (pos err %d steps, %.3f deg) endstop=TRIGGERED",
                  axis, (int)err, err * 360.0 / _fullRotationSteps);
            enterState(State::NEXT_AXIS);
            break;
        }

        case State::NEXT_AXIS:
        {
            _motionControl.disarmEndStopEdgeCapture();
            _currentAxis++;
            if (_currentAxis >= _numAxes)
            {
                // Homing necessarily stops mid-move to catch edges, and
                // stopAll() reacts to that by clearing every axis's homed flag
                // AND setting _positionUncertain. Both must be undone here or
                // the first ramped move after homing is rejected with
                // HOMING_REQUIRED (seen as an intermittent failCmdFailed -
                // intermittent because it depends on whether motion happened to
                // still be running at the last stop).
                //
                // Every axis is sitting on its own origin at this point, so
                // setCurPositionAsOrigin is positionally a no-op; it is called
                // for its markPositionCertain side effect, which is the only
                // thing that clears _positionUncertain. setAxisOrigin() does not.
                for (int a = _startAxis; a < _numAxes; a++)
                    _motionControl.setAxisHomed(a, true);

                // ORDER MATTERS. setCurPositionAsOrigin() zeroes the step
                // position of EVERY axis, so calling it after the per-axis
                // origins have been set silently wipes the home offsets (home
                // then reported 0.00/180.00 instead of the geometric zero, and
                // the position field stayed distorted at 5.37 mm rms). Clear
                // the uncertain flag FIRST, then re-apply the offsets.
                _motionControl.setCurPositionAsOrigin(true);
                for (int a = _startAxis; a < _numAxes; a++)
                    _motionControl.setAxisOrigin(a);
                LOG_I(MODULE_PREFIX, "ALL AXES HOMED");
                enterState(State::COMPLETE);
                _motionControl.stopPattern();
                break;
            }
            startAxis(_currentAxis);
            break;
        }

        default:
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Helpers
////////////////////////////////////////////////////////////////////////////////

void HomingSeekCenter::enterState(State s)
{
    _state = s;
    _stateEntryTimeMs = millis();
}

bool HomingSeekCenter::endStopTriggered(int axis) const
{
    bool isFresh = false;
    bool trig = _motionControl.getEndStopState(axis, false, isFresh);
    return isFresh && trig;
}

AxisStepsDataType HomingSeekCenter::axisPos(int axis) const
{
    return _motionControl.getAxisTotalSteps().getVal(axis);
}

void HomingSeekCenter::moveSteps(int axis, AxisStepsDataType steps, uint32_t stepsPerSec)
{
    MotionArgs args;
    args.clear();
    args.setMode("pos-rel-steps");
    args.setSpeed(String(stepsPerSec) + "sps");
    args.setDoNotSplitMove(true);
    args.getAxesPos().setVal(axis, steps);
    args.getAxesSpecified().setVal(axis, true);
    _motionControl.moveTo(args);
}

void HomingSeekCenter::stopMotion()
{
    _motionControl.stopAndClear();
}

void HomingSeekCenter::setError(const char* msg)
{
    LOG_E(MODULE_PREFIX, "axis %d HOMING FAILED: %s (pos=%d)", _currentAxis, msg, (int)axisPos(_currentAxis));
    _motionControl.disarmEndStopEdgeCapture();
    _state = State::ERROR;
    _motionControl.stopPattern();
}

////////////////////////////////////////////////////////////////////////////////
MotionPatternBase* HomingSeekCenter::create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
{
    return new HomingSeekCenter(pNamedValueProvider, motionControl);
}
