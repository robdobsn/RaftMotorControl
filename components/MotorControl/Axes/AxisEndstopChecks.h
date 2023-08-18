/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisEndstopChecks
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <RaftArduino.h>
#include "esp_attr.h"
#include <Logger.h>
#include <JSONParams.h>

static const uint32_t AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS = 2;

class AxisEndstopChecks
{
public:
    static constexpr uint32_t MIN_MAX_VALID_BIT = 31;
    static constexpr uint32_t MIN_MAX_VALUES_MASK = 0x3fffffff;
    static constexpr uint32_t MIN_VAL_IDX = 0;
    static constexpr uint32_t MAX_VAL_IDX = 1;
    static constexpr uint32_t BITS_PER_VAL = 2;
    static constexpr uint32_t BITS_PER_VAL_MASK = 0x03;
    static constexpr uint32_t MAX_AXIS_INDEX = (32 / (AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS * BITS_PER_VAL)) - 1;

    enum AxisMinMaxEnum
    {
        END_STOP_NOT_HIT = 0,
        END_STOP_HIT = 1,
        END_STOP_TOWARDS = 2,
        END_STOP_NONE = 3
    };

    AxisEndstopChecks();
    AxisEndstopChecks(const AxisEndstopChecks &other)
    {
        _uint = other._uint;
    }
    AxisEndstopChecks &operator=(const AxisEndstopChecks &other)
    {
        _uint = other._uint;
        return *this;
    }
    bool operator==(const AxisEndstopChecks& other) const
    {
        return _uint == other._uint;
    }
    bool operator!=(const AxisEndstopChecks& other) const
    {
        return !(*this == other);
    }
    bool isValid() const
    {
        return _uint & (1 << MIN_MAX_VALID_BIT);
    }
    void set(uint32_t axisIdx, uint32_t endStopIdx, AxisMinMaxEnum checkType);
    void set(uint32_t axisIdx, uint32_t endStopIdx, const String& minMaxStr);
    AxisMinMaxEnum get(uint32_t axisIdx, uint32_t endStopIdx) const;
    // Reverse endstop direction for endstops that are set
    void reverse();
    // Clear endstops on all axes
    void clear()
    {
        _uint = 0;
    }
    // Set endstop on all axes when moving towards
    void all();
    inline bool IRAM_ATTR any() const
    {
        return (_uint & (1 << MIN_MAX_VALID_BIT)) && (_uint & MIN_MAX_VALUES_MASK);
    }
    uint32_t debugGetRawValue() const
    {
        return _uint;
    }
    String getStr(AxisMinMaxEnum minMax) const;
    void fromJSON(const JSONParams& jsonData, const char* elemName);
    String toJSON(const char* elemName) const;

private:
    uint32_t _uint;
};