////////////////////////////////////////////////////////////////////////////////
//
// EndStopScan
// Characterise an end-stop sensor region by repeated crossings in one frame
//
// Rob Dobson 2026
//
////////////////////////////////////////////////////////////////////////////////

#include "EndStopScan.h"
#include "MotionArgs.h"
#include "RaftUtils.h"
#include "RaftJson.h"
#include "Logger.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
EndStopScan::EndStopScan(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
    : MotionPatternBase(pNamedValueProvider, motionControl)
{
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
EndStopScan::~EndStopScan()
{
    _motionControl.disarmEndStopEdgeCapture();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Factory
MotionPatternBase* EndStopScan::create(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl)
{
    return new EndStopScan(pNamedValueProvider, motionControl);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Setup the scan
/// @param pParamsJson axis, stepsPerSec, passes, clearSteps, timeoutMs, startDir
void EndStopScan::setup(const char* pParamsJson)
{
    if (pParamsJson)
    {
        RaftJson config(pParamsJson);
        _axis = config.getInt("axis", _axis);
        _stepsPerSec = config.getInt("stepsPerSec", _stepsPerSec);
        _passes = config.getInt("passes", _passes);
        _clearSteps = config.getInt("clearSteps", _clearSteps);
        _timeoutMs = config.getInt("timeoutMs", _timeoutMs);
        _dir = config.getInt("startDir", _dir);
    }

    _passesDone = 0;
    _transitionsLogged = 0;
    _sawEnter = false;
    _startTimeMs = millis();

    // NOTE: deliberately NOT calling setCurPositionAsOrigin - the whole point is
    // that every transition in this run shares one coordinate frame.
    _motionControl.armEndStopEdgeCapture(_axis, false); // min endstop

    LOG_I(MODULE_PREFIX, "SCAN START axis=%d sps=%u passes=%d clearSteps=%d startDir=%d "
                         "(frame NOT reset - positions comparable across the whole run)",
          _axis, (unsigned)_stepsPerSec, _passes, (int)_clearSteps, _dir);

    sendRotate(_dir);
    setState(State::SEEKING);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Drain any captured transitions and log them
void EndStopScan::drainTransitions()
{
    AxisStepsDataType pos = 0;
    bool newState = false;
    while (_motionControl.popEndStopEdge(pos, newState))
    {
        _transitionsLogged++;
        LOG_I(MODULE_PREFIX, "SCAN EDGE n=%d pass=%d dir=%d sps=%u state=%s pos=%d",
              _transitionsLogged, _passesDone, _dir, (unsigned)_stepsPerSec,
              newState ? "ENTER" : "LEAVE", (int)pos);

        if (newState)
        {
            // Entered the region
            _sawEnter = true;
            _enterPos = pos;
        }
        else if (_sawEnter)
        {
            // Left the region - one complete crossing
            _sawEnter = false;
            _passesDone++;
            LOG_I(MODULE_PREFIX, "SCAN PASS done=%d dir=%d sps=%u enter=%d leave=%d width=%d",
                  _passesDone, _dir, (unsigned)_stepsPerSec,
                  (int)_enterPos, (int)pos, (int)abs(_enterPos - pos));
            // Continue a margin past the region, then reverse
            _clearStartPos = pos;
            setState(State::CLEARING);
            return;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Service loop
void EndStopScan::loop()
{
    if ((_state == State::IDLE) || (_state == State::COMPLETE) || (_state == State::ERROR))
        return;

    if (Raft::isTimeout(millis(), _startTimeMs, _timeoutMs))
    {
        LOG_E(MODULE_PREFIX, "SCAN TIMEOUT after %d passes (%d transitions)",
              _passesDone, _transitionsLogged);
        stopMotion();
        _motionControl.disarmEndStopEdgeCapture();
        setState(State::ERROR);
        _motionControl.stopPattern();
        return;
    }

    switch (_state)
    {
        case State::SEEKING:
        {
            drainTransitions();
            break;
        }

        case State::CLEARING:
        {
            AxesValues<AxisStepsDataType> posVals = _motionControl.getAxisTotalSteps();
            AxisStepsDataType p = posVals.getVal(_axis);
            if (abs(p - _clearStartPos) < _clearSteps)
                break;

            stopMotion();
            if (_passesDone >= _passes)
            {
                uint32_t overflow = _motionControl.getEndStopEdgeOverflowCount();
                LOG_I(MODULE_PREFIX, "SCAN COMPLETE passes=%d transitions=%d queueOverflow=%u",
                      _passesDone, _transitionsLogged, (unsigned)overflow);
                _motionControl.disarmEndStopEdgeCapture();
                setState(State::COMPLETE);
                _motionControl.stopPattern();
                break;
            }
            // Reverse and cross the region again from the other side
            _dir = -_dir;
            _sawEnter = false;
            sendRotate(_dir);
            setState(State::SEEKING);
            break;
        }

        default:
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Rotate the axis in the given direction
void EndStopScan::sendRotate(int dir)
{
    int steps = _fullRotationSteps * _maxRotations * dir;
    MotionArgs args;
    args.clear();
    args.setMode("pos-rel-steps");
    args.setSpeed(String(_stepsPerSec) + "sps");
    args.setDoNotSplitMove(true);
    AxesValues<AxisPosDataType>& axisVals = args.getAxesPos();
    axisVals.setVal(_axis, steps);
    args.getAxesSpecified().setVal(_axis, true);
    _motionControl.moveTo(args);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Stop motion and clear pending moves
void EndStopScan::stopMotion()
{
    _motionControl.stopAndClear();
}
