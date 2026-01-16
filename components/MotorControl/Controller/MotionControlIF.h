/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionControlIF
// Interface for motion control to be used by motion patterns
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesValues.h"
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

    /// @brief Set current position as home
    /// @param allAxes true to set all axes, false to set only specified axis
    /// @param axisIdx axis index (only used if allAxes is false)
    virtual void setCurPositionAsOrigin(bool allAxes = true, uint32_t axisIdx = 0) = 0;

    /// @brief Get last commanded position
    /// @return AxesValues of last commanded position
    virtual AxesValues<AxisPosDataType> getLastCommandedPos() const = 0;

    /// @brief Get last monitored position
    /// @return AxesValues of last monitored position
    virtual AxesValues<AxisPosDataType> getLastMonitoredPos() const = 0;

    /// @brief Get end-stop state for an axis (min or max)
    /// @param axisIdx axis index
    /// @param max true for max end-stop, false for min end-stop
    /// @param isFresh (out) set to true if valid, false if not configured
    /// @return true if triggered, false otherwise
    virtual bool getEndStopState(uint32_t axisIdx, bool max, bool& isFresh) const = 0;

    /// @brief Stop current motion pattern
    virtual void stopPattern() = 0;
};
