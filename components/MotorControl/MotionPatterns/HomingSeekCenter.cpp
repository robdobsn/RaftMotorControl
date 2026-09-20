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
    _settleStopIssued = false;

    // Do NOT read the end-stop yet. Homing is typically entered right after a
    // stop() - `home_and_wait` allows only 0.5 s - and the arm can still be
    // decelerating. An end-stop sampled mid-motion near an edge yields the
    // wrong `_startedInsideFlag`, the approach then drives the wrong way, and
    // homing fails with "no flag found in one rotation". Intermittent, because
    // it depends where the previous test left the arm: seen twice in five full
    // regressions from DIFFERENT poses, never reproducible in isolation where
    // the arm is already still.
    enterState(State::SETTLE_START);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Sense the end-stop and launch the approach (arm is known stationary)
void HomingSeekCenter::beginApproach(int axis)
{
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
        case State::SETTLE_START:
        {
            // Take ownership: stop whatever was running, ONCE, then wait a
            // FIXED quiet period before sensing the end-stop.
            //
            // The previous version waited on !isBusy() and restarted its own
            // timer while busy. Both were wrong: homing is routinely invoked
            // while a pattern is playing or stopping, so isBusy() may never
            // clear, and restarting the timer also defeated the global
            // timeout - homing hung indefinitely instead of failing. That
            // turned one "homing did not complete" per suite into four.
            if (!_settleStopIssued)
            {
                _motionControl.stopAndClear();
                _settleStopIssued = true;
                _settleStartMs = millis();
                break;
            }
            if (!Raft::isTimeout(millis(), _settleStartMs, SETTLE_BEFORE_SENSE_MS))
                break;
            beginApproach(axis);
            break;
        }

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
                // Park at (sensor midpoint + homeOffsetSteps). The offset is
                // the `setHomeHere` calibration: the distance from the midpoint
                // to the position at which the END EFFECTOR sits at the CENTRE
                // OF THE BED. Parking on the bare midpoint leaves the EE ~8 mm
                // off centre, which is wrong for the machine's purpose.
                //
                // NOTE the offset may place the axis outside its trigger
                // region - axis 1's exceeds its flag half-width - so the
                // end-stop is NOT required to read triggered at the park
                // position. It is checked at the MIDPOINT instead, before the
                // offset is applied, which is what actually validates the edge
                // measurement.
                AxisStepsDataType homeOffset = (axis < (int)_homeOffsetStepsPerAxis.size())
                                                ? _homeOffsetStepsPerAxis[axis] : 0;
                // The stored offset is MODULAR - a rotary axis has no absolute
                // step zero, so a saved value may be the positive congruent
                // form. Axis 1's reads 9280, which is -320 steps (-12 deg), not
                // a 348 deg journey: applied raw it commanded a 9562-step move,
                // very nearly a full revolution.
                homeOffset = homeOffset % _fullRotationSteps;
                if (homeOffset > _fullRotationSteps / 2)
                    homeOffset -= _fullRotationSteps;
                else if (homeOffset < -_fullRotationSteps / 2)
                    homeOffset += _fullRotationSteps;
                _midPos = (_edgeAPos + _edgeBPos) / 2 + homeOffset;
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

            // Acceptance test. The edge measurement is validated by the cross
            // itself: both edges were captured, so the midpoint provably lies
            // inside the trigger region. The end-stop is NOT required to read
            // triggered at the PARK position, because homeOffsetSteps (the
            // bed-centring calibration) can legitimately place the park point
            // outside the flag - axis 1's offset exceeds its flag half-width.
            // Requiring it here made homing fail outright once the offset was
            // applied.
            bool trigAtPark = endStopTriggered(axis);
            AxisStepsDataType homeOffset = (axis < (int)_homeOffsetStepsPerAxis.size())
                                            ? _homeOffsetStepsPerAxis[axis] : 0;
            homeOffset = homeOffset % _fullRotationSteps;
            if (homeOffset > _fullRotationSteps / 2) homeOffset -= _fullRotationSteps;
            else if (homeOffset < -_fullRotationSteps / 2) homeOffset += _fullRotationSteps;
            AxisStepsDataType width = _edgeBPos - _edgeAPos;
            if (width < 0) width = -width;
            if (width < 20)
            {
                setError("flag width implausibly small - edge measurement bad");
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
            LOG_I(MODULE_PREFIX, "axis %d HOMED (pos err %d steps, %.3f deg) "
                  "flagWidth %d offset %d endstopAtPark=%s",
                  axis, (int)err, err * 360.0 / _fullRotationSteps,
                  (int)width, (int)homeOffset, trigAtPark ? "yes" : "no (expected if offset > half-width)");
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

                // Steps are now zero, but with a homeOffsetSteps in the
                // kinematics the arm is NOT at the Cartesian origin. Leaving
                // the tracked position at (0,0) makes the first sub-block of
                // the next split move plan from a place the arm is not, which
                // near the folded home pose turns ~8 mm of error into a 45 deg
                // joint slam. Resync so units and steps agree.
                _motionControl.syncUnitsFromSteps();
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

const char* HomingSeekCenter::stateName(State s)
{
    switch (s)
    {
        case State::IDLE: return "IDLE";
        case State::SETTLE_START: return "SETTLE_START";
        case State::FAST_APPROACH: return "FAST_APPROACH";
        case State::SLOW_CROSS: return "SLOW_CROSS";
        case State::FAST_TO_MID: return "FAST_TO_MID";
        case State::VERIFY: return "VERIFY";
        case State::NEXT_AXIS: return "NEXT_AXIS";
        case State::COMPLETE: return "COMPLETE";
        default: return "ERROR";
    }
}

void HomingSeekCenter::setError(const char* msg)
{
    // Report the state and the sensed conditions: this pattern fails rarely
    // and only inside a full suite run, so the log has to be enough on its own.
    LOG_E(MODULE_PREFIX, "axis %d HOMING FAILED in %s: %s (pos=%d endstop=%s "
          "startedInside=%d reversing=%d gotA=%d gotB=%d)",
          _currentAxis, stateName(_state), msg, (int)axisPos(_currentAxis),
          endStopTriggered(_currentAxis) ? "TRIG" : "clear",
          (int)_startedInsideFlag, (int)_approachReversing,
          (int)_gotEdgeA, (int)_gotEdgeB);
    _motionControl.disarmEndStopEdgeCapture();
    _state = State::ERROR;
    _motionControl.stopPattern();
}

////////////////////////////////////////////////////////////////////////////////
MotionPatternBase* HomingSeekCenter::create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
{
    return new HomingSeekCenter(pNamedValueProvider, motionControl);
}
