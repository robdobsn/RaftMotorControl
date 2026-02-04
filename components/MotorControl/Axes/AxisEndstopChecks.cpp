/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisEndstopChecks
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "AxisEndstopChecks.h"
#include "AxesValues.h"

static const char* AxisEndstopMinMaxEnumStrs[] = {"0", "1", "T", "X"};

AxisEndstopChecks::AxisEndstopChecks()
{
    _uint = 0;
    for (uint32_t axisIdx = 0; axisIdx < MAX_AXIS_INDEX; axisIdx++)
    {
        for (uint32_t endStopIdx = 0; endStopIdx < AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS; endStopIdx++)
        {
            uint32_t valIdx = (axisIdx * AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS + endStopIdx) * BITS_PER_VAL;
            _uint |= (END_STOP_NONE << valIdx);
        }
    }
}

AxisEndstopChecks::AxisMinMaxEnum MOTOR_TICK_FN_DECORATOR AxisEndstopChecks::get(uint32_t axisIdx, uint32_t endStopIdx) const
{
    if ((axisIdx > MAX_AXIS_INDEX) || (endStopIdx >= AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS))
        return END_STOP_NONE;
    uint32_t valIdx = (axisIdx * AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS + endStopIdx) * BITS_PER_VAL;
    return (AxisMinMaxEnum)((_uint >> valIdx) & BITS_PER_VAL_MASK);
}

void AxisEndstopChecks::set(uint32_t axisIdx, uint32_t endStopIdx, AxisMinMaxEnum checkType)
{
    if ((axisIdx > MAX_AXIS_INDEX) || (endStopIdx >= AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS))
        return;
    uint32_t valIdx = (axisIdx * AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS + endStopIdx) * BITS_PER_VAL;
    uint32_t valMask = (BITS_PER_VAL_MASK << valIdx);
    _uint &= (valMask ^ 0xffffffff);
    _uint |= checkType << valIdx;
    _uint |= (1 << MIN_MAX_VALID_BIT);
}
void AxisEndstopChecks::set(uint32_t axisIdx, uint32_t endStopIdx, const String& minMaxStr)
{
    AxisMinMaxEnum setTo = END_STOP_NONE;
    for (uint32_t i = 0; i < sizeof(AxisEndstopMinMaxEnumStrs)/sizeof(AxisEndstopMinMaxEnumStrs[0]); i++)
    {
        if (minMaxStr.equalsIgnoreCase(AxisEndstopMinMaxEnumStrs[i]))
        {
            setTo = (AxisMinMaxEnum)i;
            break;
        }
    }
    set(axisIdx, endStopIdx, setTo);
}
void AxisEndstopChecks::reverse()
{
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS; i++)
        {
            AxisMinMaxEnum esEnum = get(axisIdx, i);
            if (esEnum == END_STOP_HIT)
                esEnum = END_STOP_NOT_HIT;
            else if (esEnum == END_STOP_NOT_HIT)
                esEnum = END_STOP_HIT;
            set(axisIdx, i, esEnum);
        }
    }
}
void AxisEndstopChecks::all()
{
    uint32_t newUint = 0;
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        newUint = newUint << (AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS * BITS_PER_VAL);
        // Stop when endstop is hit and axis is moving towards this endstop
        for (uint32_t valIdx = 0; valIdx < AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS; valIdx++)
            newUint |= END_STOP_TOWARDS << (valIdx * BITS_PER_VAL);
    }
    _uint = newUint |= (1 << MIN_MAX_VALID_BIT);
}
String AxisEndstopChecks::getStr(AxisMinMaxEnum minMax) const
{
    return AxisEndstopMinMaxEnumStrs[minMax];
}
void AxisEndstopChecks::fromJSON(const RaftJsonIF& jsonData, const char* elemName)
{
    // Extract array
    std::vector<String> endpointList;
    jsonData.getArrayElems(elemName, endpointList);
    uint32_t axisIdx = 0;
    for (const RaftJson endpoint : endpointList)
    {
        if (axisIdx >= MAX_AXIS_INDEX)
            break;
        for (uint32_t endstopIdx = 0; endstopIdx < AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS; endstopIdx++)
        {
            char endstopIdxStr[20];
            snprintf(endstopIdxStr, sizeof(endstopIdxStr), "[%d]", (int)endstopIdx);
            set(axisIdx, endstopIdx, endpoint.getString(endstopIdxStr, ""));
        }
        axisIdx++;
    }
}
String AxisEndstopChecks::toJSON(const char* elemName) const
{
    String jsonStr = "[";
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        if (axisIdx != 0)
            jsonStr += ",";
        jsonStr += "[";
        for (uint32_t endstopIdx = 0; endstopIdx < AXIS_VALUES_MAX_ENDSTOPS_PER_AXIS; endstopIdx++)
        {
            if (endstopIdx != 0)
                jsonStr += ",";
            jsonStr += "\"" + getStr(get(axisIdx, endstopIdx)) + "\"";
        }
        jsonStr += "]";
    }
    jsonStr += "]";
    return "\"" + String(elemName) + "\":" + jsonStr;
}
