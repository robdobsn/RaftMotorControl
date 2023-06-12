/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxesPosition
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxisValues.h"
#include "AxesPosValues.h"
#include "AxesParamVals.h"

class AxesPosition
{
public:
    AxesPosition()
    {
        clear();
    }
    void clear()
    {
        unitsFromHome.clear();
        stepsFromHome.clear();
        _unitsFromHomeValid = false;
    }
    bool unitsFromHomeValid()
    {
        return _unitsFromHomeValid;
    }
    void setUnitsFromHomeValidity(bool valid)
    {
        _unitsFromHomeValid = valid;
    }
    // Axis position in axes units and steps
    AxesPosValues unitsFromHome;
    AxesParamVals<AxisStepsDataType> stepsFromHome;
private:
    bool _unitsFromHomeValid;
};
