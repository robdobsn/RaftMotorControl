/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionControlIF
// Interface for motion control to be used by motion patterns
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include "AxesValues.h"
#include "AxesParams.h"
#include "RaftRetCode.h"

class MotionArgs;

class MotionControlIF
{
public:
    virtual ~MotionControlIF() {}

    /// @brief Move to a specific location (flat or ramped and relative or absolute)
    /// @param args MotionArgs specify the motion to be performed
    /// @param respMsg Optional pointer to string for error message (default nullptr)
    /// @return RaftRetCode
    virtual RaftRetCode moveTo(MotionArgs& args, String* respMsg = nullptr) = 0;

    /// @brief Pause (or resume) all motion
    /// @param pauseIt true to pause, false to resume
    virtual void pause(bool pauseIt) = 0;

    /// @brief Check if the motion controller is paused
    /// @return true if paused
    virtual bool isPaused() const = 0;

    /// @brief Check if the motion controller is busy
    /// @return true if any motion is in the pipeline
    virtual bool isBusy() const = 0;

    /// @brief Set current position as home (all axes)
    /// @param markPositionCertain If true, treat the resulting origin as physically
    ///        known (clears any position-uncertain flag from a prior mid-motion stop).
    ///        Pass false when the call is only being used to reset step counters
    ///        without asserting that the arm is really at the origin — e.g. at the
    ///        START of a homing pattern where the arm's actual pose is unknown until
    ///        the switches have been located.
    virtual void setCurPositionAsOrigin(bool markPositionCertain = true) = 0;

    /// @brief Set the current position of a specific axis as its origin (zero)
    /// @param axisIdx Axis index to set as origin
    virtual void setAxisOrigin(uint32_t axisIdx) = 0;

    /// @brief Get axes parameters
    /// @return AxesParams structure
    virtual AxesParams getAxesParams() const = 0;

    /// @brief Get last commanded position
    /// @return AxesValues of last commanded position
    virtual AxesValues<AxisPosDataType> getLastCommandedPos() const = 0;

    /// @brief Get last monitored position
    /// @return AxesValues of last monitored position
    virtual AxesValues<AxisPosDataType> getLastMonitoredPos() const = 0;

    /// @brief Get total step counts for all axes
    /// @return AxesValues of total step counts for all axes
    virtual AxesValues<AxisStepsDataType> getAxisTotalSteps() const = 0;

    /// @brief Get end-stop state for an axis (min or max)
    /// @param axisIdx axis index
    /// @param max true for max end-stop, false for min end-stop
    /// @param isFresh (out) set to true if valid, false if not configured
    /// @return true if triggered, false otherwise
    virtual bool getEndStopState(uint32_t axisIdx, bool max, bool& isFresh) const = 0;

    /// @brief Arm step-rate capture of end-stop transitions for one axis
    /// @param axisIdx axis index
    /// @param isMax true for max end-stop, false for min end-stop
    /// @note Gives homing the axis position at the instant the end-stop changes
    ///       state, rather than at the next service-loop poll. See
    ///       RampGenerator::armEndStopEdgeCapture for why this matters.
    virtual void armEndStopEdgeCapture(uint32_t axisIdx, bool isMax) = 0;

    /// @brief Disarm end-stop transition capture
    virtual void disarmEndStopEdgeCapture() = 0;

    /// @brief Get the most recent captured end-stop transition
    /// @param seq (out) capture sequence number - increments on each transition
    /// @param posSteps (out) axis total step position at the transition
    /// @param newState (out) end-stop state after the transition
    /// @return true if a transition has been captured since arming
    virtual bool getEndStopEdgeCapture(uint32_t& seq, AxisStepsDataType& posSteps, bool& newState) const = 0;

    /// @brief Pop the oldest unread end-stop transition from the capture queue
    /// @param posSteps (out) axis total step position at the transition
    /// @param newState (out) end-stop state after the transition
    /// @return true if a transition was available
    virtual bool popEndStopEdge(AxisStepsDataType& posSteps, bool& newState) = 0;

    /// @brief Number of transitions dropped because the capture queue was full
    virtual uint32_t getEndStopEdgeOverflowCount() const = 0;

    /// @brief Stop current motion pattern
    virtual void stopPattern() = 0;

    /// @brief Stop all motion and clear the queue
    virtual void stopAndClear() = 0;

    /// @brief Set homing status for a specific axis
    /// @param axisIdx Axis index
    /// @param homed true if axis has been homed
    virtual void setAxisHomed(uint32_t axisIdx, bool homed) = 0;

    /// @brief Check if a specific axis has been homed
    /// @param axisIdx Axis index
    /// @return true if axis has been homed since last reset
    virtual bool isAxisHomed(uint32_t axisIdx) const = 0;

    /// @brief Check if all axes have been homed
    /// @return true if all axes have been homed since last reset
    virtual bool isAllAxesHomed() const = 0;

    /// @brief Apply (and optionally persist) per-axis home offsets in steps (calibration)
    /// @param offsetStepsPerAxis One entry per axis (extra entries ignored)
    /// @param persist If true, also write the offsets to non-volatile storage
    /// @note Default no-op so non-motion implementers are unaffected
    virtual void applyHomeOffsetsSteps(const std::vector<AxisStepsDataType>& offsetStepsPerAxis, bool persist)
    {
        (void)offsetStepsPerAxis; (void)persist;
    }
};
