/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisValidBools
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "Logger.h"
#include "RaftArduino.h"
#include "esp_attr.h"
class AxisValidBools
{
public:
    struct BoolBitValues
    {
        bool bX : 1;
        bool bY : 1;
        bool bZ : 1;
    };
    union {
        BoolBitValues _bits;
        uint16_t _uint;
    };

    AxisValidBools()
    {
        _uint = 0;
    }
    AxisValidBools(const AxisValidBools &other)
    {
        _uint = other._uint;
    }
    AxisValidBools &operator=(const AxisValidBools &other)
    {
        _uint = other._uint;
        return *this;
    }
    bool operator==(const AxisValidBools& other)
    {
        return _uint == other._uint;
    }
    bool operator!=(const AxisValidBools& other)
    {
        return !(*this == other);
    }
    bool isValid(uint32_t axisIdx)
    {
        return _uint & (1 << axisIdx);
    }
    AxisValidBools(bool xValid, bool yValid, bool zValid)
    {
        _uint = 0;
        _bits.bX = xValid;
        _bits.bY = yValid;
        _bits.bZ = zValid;
    }
    bool XValid()
    {
        return _bits.bX;
    }
    bool YValid()
    {
        return _bits.bY;
    }
    bool ZValid()
    {
        return _bits.bZ;
    }
    bool operator[](uint32_t boolIdx)
    {
        return isValid(boolIdx);
    }
    void setVal(uint32_t boolIdx, bool val)
    {
        if (val)
            _uint |= (1 << boolIdx);
        else
            _uint &= (0xffff ^ (1 << boolIdx));
    }
};
