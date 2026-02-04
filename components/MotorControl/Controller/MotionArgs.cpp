/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionArgs
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionArgs.h"

// #define DEBUG_MOTION_ARGS

void MotionArgs::fromJSON(const char* jsonStr)
{
    // Get json
    RaftJson cmdJson(jsonStr);
    clear();

    // Get mode (default to "abs" if not specified)
    _mode = cmdJson.getString("mode", "abs");

    // Get speed (can be numeric or string)
    int arrayLen = 0;
    if (cmdJson.getType("speed", arrayLen) == RaftJson::RAFT_JSON_STRING)
    {
        _speed = cmdJson.getString("speed", "");
    }
    else if (cmdJson.getType("speed", arrayLen) == RaftJson::RAFT_JSON_NUMBER)
    {
        // Numeric speed is interpreted as percentage
        double speedVal = cmdJson.getDouble("speed", 100.0);
        _speed = String(speedVal);
    }

    // Get motor current
    _motorCurrent = cmdJson.getDouble("motorCurrent", 0);

    // Get boolean flags
    _dontSplitMove = cmdJson.getBool("nosplit", false);
    _moveClockwise = cmdJson.getBool("cw", false);
    _moveRapid = cmdJson.getBool("rapid", false);
    _moreMovesComing = cmdJson.getBool("more", false);
    
    // Parse outOfBounds string value
    String oobStr = cmdJson.getString("outOfBounds", "");
    if (oobStr == "allow" || oobStr == "ok")
        _outOfBoundsAction = OutOfBoundsAction::ALLOW;
    else if (oobStr == "clamp" || oobStr == "constrain")
        _outOfBoundsAction = OutOfBoundsAction::CLAMP;
    else if (oobStr == "discard" || oobStr == "reject")
        _outOfBoundsAction = OutOfBoundsAction::DISCARD;
    // If not specified, USE_DEFAULT (inherits from SysType)
    
    _immediateExecution = cmdJson.getBool("imm", false);

    // Get extrude distance (if present)
    if (cmdJson.contains("exDist"))
    {
        _extrudeDistance = cmdJson.getDouble("exDist", 0);
        _extrudeValid = true;
    }

    // Get motion tracking index (if present)
    if (cmdJson.contains("idx"))
    {
        _motionTrackingIdx = cmdJson.getLong("idx", 0);
        _motionTrackingIndexValid = true;
    }

    // Extract endstops
    _endstops.fromJSON(cmdJson, "endstops");

    // Extract position or velocity array
    const char* arrayName = isVelocityMode() ? "vel" : "pos";
    std::vector<String> valueList;
    cmdJson.getArrayElems(arrayName, valueList);
    
    _axesPos.clear();
    _velocities.clear();
    _axesSpecified.clear();
    
    // Parse array format: [100.0, 50.0, null, 25.0]
    for (size_t axisIdx = 0; axisIdx < valueList.size() && axisIdx < MULTISTEPPER_MAX_AXES; axisIdx++)
    {
#ifdef DEBUG_MOTION_ARGS
        LOG_I(MODULE_PREFIX, "fromJSON %s[%d] valueString='%s'", arrayName, axisIdx, valueList[axisIdx].c_str());
#endif
        RaftJson valueJson(valueList[axisIdx].c_str());
        int arrayLen = 0;
        if (valueJson.getType("", arrayLen) == RaftJson::RAFT_JSON_NUMBER)
        {
            double axisValue = valueJson.getDouble("", 0);
            if (isVelocityMode())
            {
                _velocities.setVal(axisIdx, axisValue);
            }
            else
            {
                _axesPos.setVal(axisIdx, axisValue);
            }
            _axesSpecified.setVal(axisIdx, true);
#ifdef DEBUG_MOTION_ARGS
            LOG_I(MODULE_PREFIX, "fromJSON %s[%d] = %.2f", arrayName, axisIdx, axisValue);
#endif
        }
        // null or non-numeric values mean axis not specified
    }
}

String MotionArgs::toJSON()
{
    // Output string - reserve space to avoid heap fragmentation
    String jsonStr;
    jsonStr.reserve(256);
    jsonStr = "\"mode\":\"" + _mode + "\"";

    // Speed (if specified)
    if (_speed.length() > 0)
    {
        // Check if speed is pure numeric (percentage)
        bool isNumeric = true;
        for (size_t i = 0; i < _speed.length(); i++)
        {
            if (!isdigit(_speed[i]) && _speed[i] != '.' && _speed[i] != '-')
            {
                isNumeric = false;
                break;
            }
        }
        
        if (isNumeric)
        {
            jsonStr += ",\"speed\":" + _speed;
        }
        else
        {
            jsonStr += ",\"speed\":\"" + _speed + "\"";
        }
    }

    // Motor current (if non-zero)
    if (_motorCurrent > 0)
    {
        jsonStr += ",\"motorCurrent\":" + String(_motorCurrent);
    }

    // Boolean flags (only output if true)
    if (_dontSplitMove)
        jsonStr += ",\"nosplit\":true";
    if (_moveClockwise)
        jsonStr += ",\"cw\":true";
    if (_moveRapid)
        jsonStr += ",\"rapid\":true";
    if (_moreMovesComing)
        jsonStr += ",\"more\":true";
    if (_outOfBoundsAction != OutOfBoundsAction::USE_DEFAULT)
    {
        jsonStr += ",\"outOfBounds\":";
        if (_outOfBoundsAction == OutOfBoundsAction::ALLOW)
            jsonStr += "\"allow\"";
        else if (_outOfBoundsAction == OutOfBoundsAction::CLAMP)
            jsonStr += "\"clamp\"";
        else
            jsonStr += "\"discard\"";
    }
    if (_immediateExecution)
        jsonStr += ",\"imm\":true";

    // Extrude distance (if valid)
    if (_extrudeValid)
    {
        jsonStr += ",\"exDist\":" + String(_extrudeDistance);
    }

    // Motion tracking index (if valid)
    if (_motionTrackingIndexValid)
    {
        jsonStr += ",\"idx\":" + String(_motionTrackingIdx);
    }

    // Endstops
    String endstopsStr = _endstops.toJSON("endstops");
    if (endstopsStr.length() > 0)
    {
        jsonStr += "," + endstopsStr;
    }

    // Position or velocity array (simplified format)
    const char* arrayName = isVelocityMode() ? "vel" : "pos";
    AxesValues<AxisPosDataType>& values = isVelocityMode() ? _velocities : _axesPos;
    
    jsonStr += ",\"" + String(arrayName) + "\":[";
    bool firstAxis = true;
    for (int32_t axisIdx = 0; axisIdx < MULTISTEPPER_MAX_AXES; axisIdx++)
    {
        if (!firstAxis)
            jsonStr += ",";
        firstAxis = false;
        
        if (_axesSpecified.getVal(axisIdx))
        {
            jsonStr += String(values.getVal(axisIdx));
        }
        else
        {
            jsonStr += "null";
        }
    }
    jsonStr += "]";
    
    return "{" + jsonStr + "}";
}
