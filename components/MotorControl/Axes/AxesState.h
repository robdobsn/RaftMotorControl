/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxesState
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesValues.h"

class AxesState
{
public:
    AxesState()
    {
        clear();
    }
    void clear()
    {
        unitsFromOrigin.clear();
        stepsFromOrigin.clear();
        _unitsFromOriginValid = false;
    }

    void setOrigin()
    {
        unitsFromOrigin.clear();
        stepsFromOrigin.clear();
        _unitsFromOriginValid = true;
    }

    /// @brief Set a single axis to origin (zero) without affecting other axes
    /// @param axisIdx Axis index to set as origin
    /// @param offsetSteps Position to report AT the origin point. Normally 0,
    ///        but homing parks at the end-stop MIDPOINT while the arm's
    ///        geometric zero is a fixed calibration distance away
    ///        (homeOffsetSteps). Reporting that offset here keeps the parked
    ///        position on the most repeatable point of the sensor while making
    ///        the reported angle agree with forward kinematics.
    void setAxisOrigin(uint32_t axisIdx, AxisStepsDataType offsetSteps = 0,
                       AxisPosDataType offsetUnits = 0)
    {
        unitsFromOrigin.setVal(axisIdx, offsetUnits);
        stepsFromOrigin.setVal(axisIdx, offsetSteps);
        _unitsFromOriginValid = true;
    }

    /// @brief Set position
    /// @param unitsFromOrigin units from origin
    /// @param steps steps (maybe relative to previous value or from origin)
    /// @param stepsAreRelativeToPreviousValue true if steps are relative to previous value
    void setPosition(const AxesValues<AxisPosDataType>& unitsFromOrigin,
                        const AxesValues<AxisStepsDataType>& steps,
                        bool stepsAreRelativeToPreviousValue)
    {
        this->unitsFromOrigin = unitsFromOrigin;
        if (!stepsAreRelativeToPreviousValue)
            this->stepsFromOrigin = steps;
        else
            this->stepsFromOrigin += steps;
        _unitsFromOriginValid = true;
    }
    AxesValues<AxisStepsDataType> getStepsFromOrigin() const
    {
        return stepsFromOrigin;
    }
    AxesValues<AxisPosDataType> getUnitsFromOrigin() const
    {
        return unitsFromOrigin;
    }
    AxisStepsDataType getStepsFromOrigin(uint32_t axisIdx) const
    {
        return stepsFromOrigin.getVal(axisIdx);
    }
    AxisPosDataType getUnitsFromOrigin(uint32_t axisIdx) const
    {
        return unitsFromOrigin.getVal(axisIdx);
    }
    bool isValid() const
    {
        return _unitsFromOriginValid;
    }
    void setStepsFromOriginAndInvalidateUnits(const AxesValues<AxisStepsDataType>& steps)
    {
        stepsFromOrigin = steps;
        _unitsFromOriginValid = false;
    }

private:
    // Axis positions in axes units and steps
    AxesValues<AxisPosDataType> unitsFromOrigin;
    AxesValues<AxisStepsDataType> stepsFromOrigin;

    // Units from origin values validity
    bool _unitsFromOriginValid = false;
};
