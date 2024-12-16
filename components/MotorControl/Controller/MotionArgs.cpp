/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionArgs
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionArgs.h"

// #define DEBUG_MOTION_ARGS

/// @brief Get field definitions for JSON serialization
/// @return std::vector<MotionArgs::FieldDefType>
std::vector<MotionArgs::FieldDefType> MotionArgs::getFieldDefs()
{
    std::vector<FieldDefType> fieldDefs;
    fieldDefs.push_back(FieldDefType("rel", &_isRelative, "bool"));
    fieldDefs.push_back(FieldDefType("ramped", &_rampedMotion, "bool"));
    fieldDefs.push_back(FieldDefType("steps", &_unitsAreSteps, "bool"));
    fieldDefs.push_back(FieldDefType("nosplit", &_dontSplitMove, "bool"));
    fieldDefs.push_back(FieldDefType("exDistOk", &_extrudeValid, "bool"));
    fieldDefs.push_back(FieldDefType("speedOk", &_targetSpeedValid, "bool"));
    fieldDefs.push_back(FieldDefType("cw", &_moveClockwise, "bool"));
    fieldDefs.push_back(FieldDefType("rapid", &_moveRapid, "bool"));
    fieldDefs.push_back(FieldDefType("more", &_moreMovesComing, "bool"));
    fieldDefs.push_back(FieldDefType("homing", &_isHoming, "bool"));
    fieldDefs.push_back(FieldDefType("idxOk", &_motionTrackingIndexValid, "bool"));
    fieldDefs.push_back(FieldDefType("feedPerMin", &_feedrateUnitsPerMin, "bool"));
    fieldDefs.push_back(FieldDefType("speed", &_targetSpeed, "double"));
    fieldDefs.push_back(FieldDefType("exDist", &_extrudeDistance, "double"));
    fieldDefs.push_back(FieldDefType("feedrate", &_feedrate, "double"));
    fieldDefs.push_back(FieldDefType("idx", &_motionTrackingIdx, "int"));
    fieldDefs.push_back(FieldDefType("en", &_enableMotors, "bool"));
    fieldDefs.push_back(FieldDefType("ampsPCofMax", &_ampsPercentOfMax, "double"));
    fieldDefs.push_back(FieldDefType("clearQ", &_preClearMotionQueue, "bool"));
    fieldDefs.push_back(FieldDefType("stop", &_stopMotion, "bool"));
    fieldDefs.push_back(FieldDefType("constrain", &_constrainToBounds, "bool"));
    fieldDefs.push_back(FieldDefType("minMotion", &_minimizeMotion, "bool"));
    return fieldDefs;
}

void MotionArgs::fromJSON(const char* jsonStr)
{
    // Get json
    RaftJson cmdJson(jsonStr);
    clear();

    // Get field definitions
    std::vector<FieldDefType> fieldDefs = getFieldDefs();

    // Get fields
    for (auto &fieldDef : fieldDefs)
    {
        // Check if value is present
        if (!cmdJson.contains(fieldDef._name.c_str()))
            continue;

        // Get value
        if (fieldDef._dataType.equalsIgnoreCase("bool"))
        {
            bool fieldVal = cmdJson.getBool(fieldDef._name.c_str(), 0);
            *((bool*)fieldDef._pValue) = fieldVal;
        }
        else if (fieldDef._dataType.equalsIgnoreCase("int"))
        {
            int fieldVal = cmdJson.getLong(fieldDef._name.c_str(), 0);
            *((int*)fieldDef._pValue) = fieldVal;
        }
        else
        {
            double fieldVal = cmdJson.getDouble(fieldDef._name.c_str(), 0);
            *((double*)fieldDef._pValue) = fieldVal;
        }
    }

    // Extract endstops
    _endstops.fromJSON(cmdJson, "endstops");

    // Extract position
    std::vector<String> posList;
    cmdJson.getArrayElems("pos", posList);
    _axesPos.clear();
    for (const RaftJson pos : posList)
    {
        int32_t axisIdx = pos.getLong("a", -1);
        double axisPos = pos.getDouble("p", 0);        
#ifdef DEBUG_MOTION_ARGS
        LOG_I(MODULE_PREFIX, "fromJSON %s pos %s axisIdx: %d, axisPos: %.2f", cmdJson.getJsonDoc(), pos.getJsonDoc(), axisIdx, axisPos);
#endif
        _axesPos.setVal(axisIdx, axisPos);
        _axesSpecified.setVal(axisIdx, true);
    }
}

String MotionArgs::toJSON()
{
    // Get field definitions
    std::vector<FieldDefType> fieldDefs = getFieldDefs();

    // Output string
    String jsonStr;

    // Expand fields
    for (auto &fieldDef : fieldDefs)
    {
        char tmpStr[100];
        if (fieldDef._dataType.equalsIgnoreCase("bool"))
        {
            snprintf(tmpStr, sizeof(tmpStr), "\"%s\":%d,", fieldDef._name.c_str(), (*((bool*)fieldDef._pValue)));
        }
        else if (fieldDef._dataType.equalsIgnoreCase("int"))
        {
            snprintf(tmpStr, sizeof(tmpStr), "\"%s\":%d,", fieldDef._name.c_str(), (*((int*)fieldDef._pValue)));
        }
        else
        {
            snprintf(tmpStr, sizeof(tmpStr), "\"%s\":%.2f,", fieldDef._name.c_str(), (*((double*)fieldDef._pValue)));
        }
        jsonStr += tmpStr;
    }

    // Expand endstops
    jsonStr += _endstops.toJSON("endstops");

    // Expand position
    jsonStr += ",\"pos\":[";
    bool firstAxis = true;
    for (int32_t axisIdx = 0; axisIdx < MULTISTEPPER_MAX_AXES; axisIdx++)
    {
        if (!firstAxis)
        {
            jsonStr += ",";
        }
        firstAxis = false;
        jsonStr += "{\"a\":" + String(axisIdx) + ",\"p\":" + String(_axesPos.getVal(axisIdx)) + "}";
    }
    jsonStr += "]";
    return "{" + jsonStr + "}";
}
