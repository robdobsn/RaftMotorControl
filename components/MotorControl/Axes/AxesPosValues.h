/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxesPosValues
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <ArduinoOrAlt.h>
#include <esp_attr.h>
#include <Logger.h>
#include <math.h>

class AxesPosValues
{
public:
    typedef float AxisPosStoreType;
    static const uint32_t STORE_TO_POS_FACTOR = 1;
    AxisPosStoreType _pt[AXIS_VALUES_MAX_AXES];
    uint8_t _validityFlags;

public:
    AxesPosValues()
    {
        clear();
    }
    AxesPosValues(const AxesPosValues &other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _pt[i] = other._pt[i];
        _validityFlags = other._validityFlags;
    }
    AxesPosValues(AxisPosDataType x, AxisPosDataType y)
    {
        _pt[0] = x * STORE_TO_POS_FACTOR;
        _pt[1] = y * STORE_TO_POS_FACTOR;
        _pt[2] = 0;
        _validityFlags = 0x03;
    }
    AxesPosValues(AxisPosDataType x, AxisPosDataType y, AxisPosDataType z)
    {
        _pt[0] = x * STORE_TO_POS_FACTOR;
        _pt[1] = y * STORE_TO_POS_FACTOR;
        _pt[2] = z * STORE_TO_POS_FACTOR;
        _validityFlags = 0x07;
    }
    AxesPosValues(AxisPosDataType x, AxisPosDataType y, AxisPosDataType z, bool xValid, bool yValid, bool zValid)
    {
        _pt[0] = x * STORE_TO_POS_FACTOR;
        _pt[1] = y * STORE_TO_POS_FACTOR;
        _pt[2] = z * STORE_TO_POS_FACTOR;
        _validityFlags = xValid ? 0x01 : 0;
        _validityFlags |= yValid ? 0x02 : 0;
        _validityFlags |= zValid ? 0x04 : 0;
    }
    uint32_t numAxes() const
    {
        return AXIS_VALUES_MAX_AXES;
    }
    bool operator==(const AxesPosValues& other) const
    {
        if (_validityFlags != other._validityFlags)
            return false;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            if ((_validityFlags & (0x01 << i)) && (_pt[i] != other._pt[i]))
                return false;
        return true;
    }
    bool operator!=(const AxesPosValues& other) const
    {
        return !(*this == other);
    }
    void clear()
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _pt[i] = 0;
        _validityFlags = 0;
    }
    inline AxisPosDataType IRAM_ATTR getVal(uint32_t axisIdx) const
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
            return _pt[axisIdx] / STORE_TO_POS_FACTOR;
        return 0;
    }
    void setVal(uint32_t axisIdx, AxisPosDataType val)
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
        {
            _pt[axisIdx] = val * STORE_TO_POS_FACTOR;
            uint32_t axisMask = 0x01 << axisIdx;
            _validityFlags |= axisMask;
        }
    }
    void set(AxisPosDataType val0, AxisPosDataType val1, AxisPosDataType val2 = 0)
    {
        _pt[0] = val0 * STORE_TO_POS_FACTOR;
        _pt[1] = val1 * STORE_TO_POS_FACTOR;
        _pt[2] = val2 * STORE_TO_POS_FACTOR;
        _validityFlags = 0x07;
    }
    void setValid(uint32_t axisIdx, bool isValid)
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
        {
            uint32_t axisMask = 0x01 << axisIdx;
            if (isValid)
                _validityFlags |= axisMask;
            else
                _validityFlags &= ~axisMask;
        }
    }
    bool isValid(uint32_t axisIdx) const
    {
        if (axisIdx < AXIS_VALUES_MAX_AXES)
        {
            uint32_t axisMask = 0x01 << axisIdx;
            return (_validityFlags & axisMask) != 0;
        }
        return false;
    }
    bool anyValid() const
    {
        return (_validityFlags != 0);
    }
    AxisPosDataType X() const
    {
        return _pt[0] / STORE_TO_POS_FACTOR;
    }
    void X(AxisPosDataType val)
    {
        _pt[0] = val * STORE_TO_POS_FACTOR;
        _validityFlags |= 0x01;
    }
    AxisPosDataType Y() const
    {
        return _pt[1] / STORE_TO_POS_FACTOR;
    }
    void Y(AxisPosDataType val)
    {
        _pt[1] = val * STORE_TO_POS_FACTOR;
        _validityFlags |= 0x02;
    }
    AxisPosDataType Z() const
    {
        return _pt[2] / STORE_TO_POS_FACTOR;
    }
    void Z(AxisPosDataType val)
    {
        _pt[2] = val * STORE_TO_POS_FACTOR;
        _validityFlags |= 0x04;
    }
    AxesPosValues &operator=(const AxesPosValues &other)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            _pt[i] = other._pt[i];
        _validityFlags = other._validityFlags;
        return *this;
    }
    AxesPosValues operator-(const AxesPosValues &pt) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._pt[i] = _pt[i] - (pt.isValid(i) ? pt._pt[i] : 0);
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator-(AxisPosDataType val) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._pt[i] = _pt[i] - (val * STORE_TO_POS_FACTOR);
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator+(const AxesPosValues &pt) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._pt[i] = _pt[i] + (pt.isValid(i) ? pt._pt[i] : 0);
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator+(AxisPosDataType val) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            result._pt[i] = _pt[i] + (val * STORE_TO_POS_FACTOR);
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator/(const AxesPosValues &pt) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
        {
            if (pt._pt[i] != 0)
                result._pt[i] = _pt[i] / (pt.isValid(i) ? pt._pt[i] : 1);
        }
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator/(AxisPosDataType val) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
        {
            if (val != 0)
                result._pt[i] = _pt[i] / (val * STORE_TO_POS_FACTOR);
        }
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator*(const AxesPosValues &pt) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
        {
            result._pt[i] = _pt[i] * (pt.isValid(i) ? pt._pt[i] : 1);
        }
        result._validityFlags = _validityFlags;
        return result;
    }
    AxesPosValues operator*(AxisPosDataType val) const
    {
        AxesPosValues result;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
        {
            result._pt[i] = _pt[i] * (val * STORE_TO_POS_FACTOR);
        }
        result._validityFlags = _validityFlags;
        return result;
    }

    // Calculate distance between points including only axes that are indicated
    // true in the optional includeDist argument
    AxisPosDataType distanceTo(const AxesPosValues &pt, bool includeDist[] = NULL) const
    {
        double distSum = 0;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
        {
            if (isValid(i) && ((includeDist == NULL) || includeDist[i]))
            {
                double sq = _pt[i] - pt._pt[i];
                sq = sq * sq;
                distSum += sq;
            }
        }
        return sqrt(distSum) / STORE_TO_POS_FACTOR;
    }

    // Debug
    String getDebugStr()
    {
        char debugStr[40];
        snprintf(debugStr, sizeof(debugStr), "X%0.2f%s Y%0.2f%s Z%0.2f%s", 
                _pt[0],
                _validityFlags & 0x01 ? "" : "(INV)",
                _pt[1], 
                _validityFlags & 0x02 ? "" : "(INV)",
                _pt[2],
                _validityFlags & 0x04 ? "" : "(INV)");
        return debugStr;
    }

    // String toJSON()
    // {
    //     String jsonStr = "[";
    //     for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    //     {
    //         if (axisIdx != 0)
    //             jsonStr += ",";
    //         jsonStr += String((double)(_pt[axisIdx]) / STORE_TO_POS_FACTOR, 2);
    //     }
    //     jsonStr += "]";
    //     return jsonStr;
    // }
};