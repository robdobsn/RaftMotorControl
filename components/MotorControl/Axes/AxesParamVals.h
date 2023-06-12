/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxesParamVals
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <ArduinoOrAlt.h>
#include "esp_attr.h"
#include <Logger.h>

template <typename T>
class AxesParamVals
{
public:
    AxesParamVals()
    {
        clear();
    }
    AxesParamVals(const AxesParamVals &other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] = other._vals[i];
    }
    AxesParamVals& operator=(const AxesParamVals& other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] = other._vals[i];
        return *this;
    }

    AxesParamVals(T x, T y)
    {
        _vals[0] = x;
        _vals[1] = y;
        _vals[2] = 0;
    }
    AxesParamVals(T x, T y, T z)
    {
        _vals[0] = x;
        _vals[1] = y;
        _vals[2] = z;
    }
    void clear()
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] = 0;
    }
    void setVal(uint32_t axisIdx, T val)
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
        {
            _vals[axisIdx] = val;
        }
    }
    T getVal(uint32_t axisIdx) const
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
            return _vals[axisIdx];
        return 0;
    }
    T vectorMultSum(AxesParamVals<T> other)
    {
        T result = 0;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result += _vals[i] * other._vals[i];
        return result;
    }
    // Debug
    String getDebugStr()
    {
        char debugStr[40];
        snprintf(debugStr, sizeof(debugStr), "X%0.2f Y%0.2f Z%0.2f", 
                (double)_vals[0],
                (double)_vals[1],
                (double)_vals[2]);
        return debugStr;
    }    
    String toJSON()
    {
        String jsonStr = "[";
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            if (axisIdx != 0)
                jsonStr += ",";
            jsonStr += String(_vals[axisIdx]);
        }
        jsonStr += "]";
        return jsonStr;
    }

private:
    T _vals[AXIS_VALUES_MAX_AXES];
};
