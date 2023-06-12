/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisInt32s
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>

class AxisInt32s
{
public:
    int32_t vals[AXIS_VALUES_MAX_AXES];

    AxisInt32s()
    {
        clear();
    }
    AxisInt32s(const AxisInt32s &u32s)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            vals[i] = u32s.vals[i];
    }
    AxisInt32s &operator=(const AxisInt32s &u32s)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            vals[i] = u32s.vals[i];
        return *this;
    }
    AxisInt32s(int32_t xVal, int32_t yVal, int32_t zVal)
    {
        vals[0] = xVal;
        vals[1] = yVal;
        vals[2] = zVal;
    }
    bool operator==(const AxisInt32s& other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            if (vals[i] != other.vals[i])
                return false;
        return true;
    }
    bool operator!=(const AxisInt32s& other)
    {
        return !(*this == other);
    }
    void clear()
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            vals[i] = 0;
    }
    void set(int32_t val0, int32_t val1, int32_t val2 = 0)
    {
        vals[0] = val0;
        vals[1] = val1;
        vals[2] = val2;
    }
    int32_t X()
    {
        return vals[0];
    }
    int32_t Y()
    {
        return vals[1];
    }
    int32_t Z()
    {
        return vals[2];
    }
    int32_t getVal(uint32_t axisIdx)
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
            return vals[axisIdx];
        return 0;
    }
    void setVal(uint32_t axisIdx, int32_t val)
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
            vals[axisIdx] = val;
    }
    String toJSON()
    {
        String jsonStr = "[";
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            if (axisIdx != 0)
                jsonStr += ",";
            jsonStr += String(vals[axisIdx]);
        }
        jsonStr += "]";
        return jsonStr;
    }
};
