/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxesValues
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "Logger.h"
#include "RaftArduino.h"
#include "esp_attr.h"

// Max axes and endstops supported
static const uint32_t AXIS_VALUES_MAX_AXES = 3;

// Data types
typedef float AxisStepRateDataType;
typedef float AxisSpeedDataType;
typedef float AxisAccDataType;
typedef float AxisPosDataType;
typedef double AxisCalcDataType;
typedef float AxisPosFactorDataType;
typedef float AxisRPMDataType;
typedef float AxisStepsFactorDataType;
typedef int32_t AxisStepsDataType;
typedef int32_t AxisStepsDataType;
typedef float AxisUnitVectorDataType;
typedef float AxisDistDataType;

class AxisPosAndValidDataType
{
public:
    AxisPosAndValidDataType()
    {
        _pos = 0;
        _valid = false;
    }
    AxisPosAndValidDataType(AxisPosDataType pos, bool valid = true)
    {
        _pos = pos;
        _valid = valid;
    }
    void clear()
    {
        _pos = 0;
        _valid = false;
    }
    bool isValid() const
    {
        return _valid;
    }
    AxisPosDataType getVal() const
    {
        return _pos;
    }
    void setVal(AxisPosDataType pos)
    {
        _pos = pos;
        _valid = true;
    }
private:
    AxisPosDataType _pos;
    bool _valid;
};

/// @brief Templated class for axis values
/// @tparam T Type of value
template <typename T>
class AxesValues
{
public:
    AxesValues()
    {
        clear();
    }
    AxesValues(const AxesValues &other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] = other._vals[i];
    }
    AxesValues& operator=(const AxesValues& other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] = other._vals[i];
        return *this;
    }
    AxesValues operator+(const AxesValues& other) const
    {
        AxesValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._vals[i] = _vals[i] + other._vals[i];
        return result;
    }
    AxesValues& operator+=(const AxesValues& other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] += other._vals[i];
        return *this;
    }
    AxesValues operator-(const AxesValues& other) const
    {
        AxesValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._vals[i] = _vals[i] - other._vals[i];
        return result;
    }
    AxesValues& operator-=(const AxesValues& other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] -= other._vals[i];
        return *this;
    }
    AxesValues operator*(T val) const
    {
        AxesValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._vals[i] = _vals[i] * val;
        return result;
    }
    AxesValues& operator*=(T val)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] *= val;
        return *this;
    }
    AxesValues operator/(T val) const
    {
        // Check for divide by zero
        if (val == 0)
        {
            AxesValues result;
            for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
                result._vals[i] = 0;
            return result;
        }
        AxesValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._vals[i] = _vals[i] / val;
        return result;
    }
    AxesValues& operator/=(T val)
    {
        // Check for divide by zero
        if (val == 0)
        {
            for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
                _vals[i] = 0;
            return *this;
        }
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] /= val;
        return *this;
    }
    T& operator[](uint32_t idx) {
        if (idx >= AXIS_VALUES_MAX_AXES) {
            static T dummy = 0;
            return dummy;
        }
        return _vals[idx];
    }
    const T& operator[](uint32_t idx) const {
        if (idx >= AXIS_VALUES_MAX_AXES) {
            static T dummy = 0;
            return dummy;
        }
        return _vals[idx];
    }
    uint32_t numAxes() const
    {
        return AXIS_VALUES_MAX_AXES;
    }

    AxesValues(T x, T y)
    {
        _vals[0] = x;
        _vals[1] = y;
        _vals[2] = 0;
    }
    AxesValues(T x, T y, T z)
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
    T vectorMultSum(AxesValues<T> other) const
    {
        T result = 0;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result += _vals[i] * other._vals[i];
        return result;
    }
    // Debug
    String getDebugStr() const
    {
        char debugStr[40];
        snprintf(debugStr, sizeof(debugStr), "X%0.2f Y%0.2f Z%0.2f", 
                (double)_vals[0],
                (double)_vals[1],
                (double)_vals[2]);
        return debugStr;
    }    
    String toJSON() const
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

    // Convert to AxesValus<AxesPosDataType>
    AxesValues<AxisPosDataType> toAxesPos() const
    {
        AxesValues<AxisPosDataType> result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result.setVal(i, _vals[i].getVal());
        return result;
    }
    void fromAxesPos(const AxesValues<AxisPosDataType>& axesPos)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _vals[i] = (T) axesPos.getVal(i);
    }

private:
    T _vals[AXIS_VALUES_MAX_AXES];
};

