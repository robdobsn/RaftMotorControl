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
