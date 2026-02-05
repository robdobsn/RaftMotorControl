/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionArgs
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include "AxesValues.h"
#include "MotorControlMsgFormats.h"
#include "AxisEndstopChecks.h"
#include "MotorControlTypes.h"

// This must be packed as it is used for binary communication
#pragma pack(push, 1)

class MotionArgs
{
public:
    MotionArgs()
    {
        clear();
    }
    
    void clear()
    {
        // Versioning
        _motionArgsStructVersion = MULTISTEPPER_MOTION_ARGS_BINARY_FORMAT_1;

        // Motion mode (default to absolute position in units)
        _mode = "abs";

        // Speed (empty means use 100% of config max)
        _speed = "";

        // Motor current (0 means use default)
        _motorCurrent = 0;

        // Flags
        _dontSplitMove = false;
        _extrudeValid = false;
        _moveClockwise = false;
        _moveRapid = false;
        _moreMovesComing = false;
        _motionTrackingIndexValid = false;
        _outOfBoundsAction = OutOfBoundsAction::USE_DEFAULT;
        _immediateExecution = false;

        // Reset values
        _extrudeDistance = 1;
        _motionTrackingIdx = 0;
        _axesPos.clear();
        _velocities.clear();
        _axesSpecified.clear();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motion Mode
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setMode(const String& mode) { _mode = mode; }
    String getMode() const { return _mode; }
    
    bool isRelative() const
    {
        return _mode == "rel" || _mode == "pos-rel-steps" || _mode == "pos-rel-steps-noramp";
    }
    
    bool areUnitsSteps() const
    {
        return _mode.startsWith("pos-") && _mode.indexOf("steps") >= 0;
    }
    
    bool isRamped() const
    {
        return _mode.indexOf("noramp") < 0;  // Ramped unless mode contains "noramp"
    }
    
    bool isVelocityMode() const
    {
        return _mode == "vel" || _mode.startsWith("vel-");
    }
    
    bool areVelocityUnitsSteps() const
    {
        return _mode == "vel-steps";
    }
    
    bool isProportionate() const
    {
        return _mode == "prop" || _mode.startsWith("prop-");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Speed Control
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setSpeed(const String& speed) { _speed = speed; }
    String getSpeed() const { return _speed; }

    /// @brief Parse speed string and calculate actual speed value
    /// @param configMaxSpeedUps Maximum configured speed in units/sec for the axis
    /// @return Speed in units/sec, automatically capped by configMaxSpeedUps
    double getSpeedUps(double configMaxSpeedUps) const
    {
        // Empty speed means use 100% of config max
        if (_speed.length() == 0)
            return configMaxSpeedUps;

        // Check if numeric (percentage)
        bool isNumeric = true;
        for (size_t i = 0; i < _speed.length(); i++)
        {
            if (!isdigit(_speed[i]) && _speed[i] != '.' && _speed[i] != '-')
            {
                isNumeric = false;
                break;
            }
        }

        double requestedSpeed;
        
        if (isNumeric)
        {
            // Numeric value is percentage
            double percent = _speed.toDouble();
            requestedSpeed = configMaxSpeedUps * (percent / 100.0);
        }
        else
        {
            // Parse string with units
            double value = _speed.toDouble();  // Extract numeric part
            
            if (_speed.endsWith("pc") || _speed.endsWith("percent"))
            {
                requestedSpeed = configMaxSpeedUps * (value / 100.0);
            }
            else if (_speed.endsWith("ups") || _speed.endsWith("unitsps"))
            {
                requestedSpeed = value;
            }
            else if (_speed.endsWith("upm") || _speed.endsWith("unitspm"))
            {
                requestedSpeed = value / 60.0;
            }
            else if (_speed.endsWith("mmps"))
            {
                requestedSpeed = value;  // Assumes axis units are mm
            }
            else if (_speed.endsWith("mmpm"))
            {
                requestedSpeed = value / 60.0;  // Assumes axis units are mm
            }
            else if (_speed.endsWith("sps"))
            {
                requestedSpeed = value;  // Steps per second (caller handles conversion)
            }
            else
            {
                // Unknown suffix, treat as percentage
                requestedSpeed = configMaxSpeedUps * (value / 100.0);
            }
        }
        
        // Always cap by configuration maximum (safety limit)
        if (requestedSpeed > configMaxSpeedUps)
            requestedSpeed = configMaxSpeedUps;
            
        return requestedSpeed;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motor Current
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setMotorCurrent(double current) { _motorCurrent = current; }
    double getMotorCurrent() const { return _motorCurrent; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motion Flags
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setDoNotSplitMove(bool flag) { _dontSplitMove = flag; }
    bool dontSplitMove() const { return _dontSplitMove; }
    
    void setMoveRapid(bool flag) { _moveRapid = flag; }
    bool isMoveRapid() const { return _moveRapid; }
    
    void setClockwise(bool flag) { _moveClockwise = flag; }
    bool isMoveClockwise() const { return _moveClockwise; }
    
    void setOutOfBoundsAction(OutOfBoundsAction action) { _outOfBoundsAction = action; }
    OutOfBoundsAction getOutOfBoundsAction() const { return _outOfBoundsAction; }
    
    OutOfBoundsAction getEffectiveOutOfBoundsAction(OutOfBoundsAction defaultAction) const
    {
        return (_outOfBoundsAction == OutOfBoundsAction::USE_DEFAULT) ? 
            defaultAction : _outOfBoundsAction;
    }
    
    void setImmediateExecution(bool flag) { _immediateExecution = flag; }
    bool isImmediateExecution() const { return _immediateExecution; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Axis Positions
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    AxesValues<AxisPosDataType>& getAxesPos() { return _axesPos; }
    const AxesValues<AxisPosDataType>& getAxesPosConst() const { return _axesPos; }
    
    AxesValues<AxisSpecifiedDataType>& getAxesSpecified() { return _axesSpecified; }
    
    void setAxesPositions(const AxesValues<AxisPosDataType>& axisPositions)
    {
        _axesPos = axisPositions;
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
            _axesSpecified.setVal(axisIdx, true);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Velocities (for velocity mode)
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    AxesValues<AxisPosDataType>& getVelocities() { return _velocities; }
    const AxesValues<AxisPosDataType>& getVelocitiesConst() const { return _velocities; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Extrusion
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setExtrudeDist(AxisDistDataType extrude)
    {
        _extrudeDistance = extrude;
        _extrudeValid = true;
    }
    bool isExtrudeValid() const { return _extrudeValid; }
    AxisDistDataType getExtrudeDist() const { return _extrudeDistance; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motion Tracking
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setMotionTrackingIndex(uint32_t motionTrackingIdx)
    {
        _motionTrackingIdx = motionTrackingIdx;
        _motionTrackingIndexValid = true;
    }
    bool isMotionTrackingIndexValid() const { return _motionTrackingIndexValid; }
    uint32_t getMotionTrackingIndex() const { return _motionTrackingIdx; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // More Moves Coming Hint
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setMoreMovesComing(bool moreMovesComing) { _moreMovesComing = moreMovesComing; }
    bool getMoreMovesComing() const { return _moreMovesComing; }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // End Stops
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setEndStops(AxisEndstopChecks endstops) { _endstops = endstops; }
    const AxisEndstopChecks& getEndstopCheck() const { return _endstops; }
    
    void setTestAllEndStops()
    {
        _endstops.all();
        LOG_I("MotionArgs", "Test all endstops");
    }
    void setTestNoEndStops() { _endstops.clear(); }
    void setTestEndStopsDefault() { _endstops.clear(); }
    void setTestEndStop(int axisIdx, int endStopIdx, AxisEndstopChecks::AxisMinMaxEnum checkType)
    {
        _endstops.set(axisIdx, endStopIdx, checkType);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // JSON Serialization
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void fromJSON(const char* jsonStr);
    String toJSON();

private:
    // Version of this structure
    uint8_t _motionArgsStructVersion = 0;

    // Motion mode: "abs", "rel", "pos-abs-steps", "pos-rel-steps", 
    // "pos-abs-steps-noramp", "pos-rel-steps-noramp", "vel", "vel-steps",
    // "prop", "prop-rel" (proportionate 0-1 mapping to axis min/max)
    String _mode = "abs";

    // Speed: numeric (percentage) or string with units ("10mmps", "500upm", "80pc", etc.)
    String _speed = "";

    // Motor current as percentage of max
    double _motorCurrent = 0;

    // Flags
    bool _dontSplitMove = false;
    bool _extrudeValid = false;
    bool _moveClockwise = false;
    bool _moveRapid = false;
    bool _moreMovesComing = false;
    bool _motionTrackingIndexValid = false;
    OutOfBoundsAction _outOfBoundsAction = OutOfBoundsAction::USE_DEFAULT;
    bool _immediateExecution = false;  // Stop, clear queue, then execute this motion

    // Extrude distance
    double _extrudeDistance = 0;

    // Motion tracking index
    uint32_t _motionTrackingIdx = 0;

    // End stops
    AxisEndstopChecks _endstops;

    // Position coordinates (for position modes)
    AxesValues<AxisPosDataType> _axesPos;

    // Velocities (for velocity modes)
    AxesValues<AxisPosDataType> _velocities;

    // Axes specified flags
    AxesValues<AxisSpecifiedDataType> _axesSpecified;

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionArgs";
};

#pragma pack(pop)
