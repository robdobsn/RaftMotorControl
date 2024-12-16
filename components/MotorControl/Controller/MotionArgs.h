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
        // Versioning - as this structure might be used in a binary
        // fashion
        _motionArgsStructVersion = MULTISTEPPER_MOTION_ARGS_BINARY_FORMAT_1;

        // Flags
        _isRelative = false;
        _rampedMotion = true;
        _unitsAreSteps = false;
        _dontSplitMove = false;
        _extrudeValid = false;
        _targetSpeedValid = false;
        _moveClockwise = false;
        _moveRapid = false;
        _moreMovesComing = false;
        _motionTrackingIndexValid = false;
        _isHoming = false;
        _feedrateUnitsPerMin = false;
        _enableMotors = true;
        _preClearMotionQueue = false;
        _stopMotion = false; 
        _constrainToBounds = false;
        _minimizeMotion = true;

        // Reset values to sensible levels
        _targetSpeed = 0;
        _feedrate = 100.0;
        _extrudeDistance = 1;
        _motionTrackingIdx = 0;
        _axesPos.clear();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motion Flags
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setRamped(bool flag)
    {
        _rampedMotion = flag;
    }
    bool isRamped() const
    {
        return _rampedMotion;
    }
    void setRelative(bool flag)
    {
        _isRelative = flag;
    }
    bool isRelative() const
    {
        return _isRelative;
    }
    void setDoNotSplitMove(bool flag)
    {
        _dontSplitMove = flag;
    }
    bool dontSplitMove() const
    {
        return _dontSplitMove;
    }
    void setMoveRapid(bool flag)
    {
        _moveRapid = flag;
    }
    bool isMoveRapid()
    {
        return _moveRapid;
    }
    void setClockwise(bool flag)
    {
        _moveClockwise = flag;
    }
    bool isMoveClockwise()
    {
        return _moveClockwise;
    }
    void setUnitsSteps(bool flag)
    {
        _unitsAreSteps = flag;
    }
    bool areUnitsSteps()
    {
        return _unitsAreSteps;
    }
    bool isEnableMotors() const
    {
        return _enableMotors;
    }
    bool isClearQueue() const
    {
        return _preClearMotionQueue;
    }
    bool isStopMotion() const
    {
        return _stopMotion;
    }
    bool constrainToBounds() const
    {
        return _constrainToBounds;
    }
    bool minimizeMotion() const
    {
        return _minimizeMotion;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Get axis positions
    /// @return AxesValues<AxisPosDataType>
    /// @note The values may be modified by the kinematics so this cannot be const
    AxesValues<AxisPosDataType>& getAxesPos()
    {
        return _axesPos;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Get Axis positions const
    /// @return const AxesValues<AxisPosDataType>&
    const AxesValues<AxisPosDataType>& getAxesPosConst() const
    {
        return _axesPos;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Get axes specified (which axes are specified in the command)
    /// @return AxesValues<AxisSpecifiedDataType>
    /// @note The values may be modified by the kinematics so this cannot be const
    AxesValues<AxisSpecifiedDataType>& getAxesSpecified()
    {
        return _axesSpecified;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Set axes positions
    /// @param axisPositions AxesValues<AxisPosDataType>
    void setAxesPositions(const AxesValues<AxisPosDataType>& axisPositions)
    {
        _axesPos = axisPositions;
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
            _axesSpecified.setVal(axisIdx, true);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Target speed
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setTargetSpeed(AxisSpeedDataType targetSpeed)
    {
        _targetSpeed = targetSpeed;
        _targetSpeedValid = true;
    }
    bool isTargetSpeedValid() const
    {
        return _targetSpeedValid;
    }
    AxisSpeedDataType getTargetSpeed() const
    {
        return _targetSpeed;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Feedrate percent
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setFeedratePercent(double feedrate)
    {
        _feedrate = feedrate;
        _feedrateUnitsPerMin = false;
    }
    void setFeedrateUnitsPerMin(double feedrate)
    {
        _feedrate = feedrate;
        _feedrateUnitsPerMin = true;
    }
    double getFeedrate() const
    {
        return _feedrate;
    }
    bool isFeedrateUnitsPerMin() const
    {
        return _feedrateUnitsPerMin;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Extrusion
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setExtrudeDist(AxisDistDataType extrude)
    {
        _extrudeDistance = extrude;
        _extrudeValid = true;
    }
    bool isExtrudeValid() const
    {
        return _extrudeValid;
    }
    AxisDistDataType getExtrudeDist() const
    {
        return _extrudeDistance;
    }    

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motion tracking
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setMotionTrackingIndex(uint32_t motionTrackingIdx)
    {
        _motionTrackingIdx = motionTrackingIdx;
        _motionTrackingIndexValid = true;
    }
    bool isMotionTrackingIndexValid() const
    {
        return _motionTrackingIndexValid;
    }
    uint32_t getMotionTrackingIndex() const
    {
        return _motionTrackingIdx;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Hint that more movement is expected (allows optimization of pipeline processing)
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setMoreMovesComing(bool moreMovesComing)
    {
        _moreMovesComing = moreMovesComing;
    }
    bool getMoreMovesComing() const
    {
        return _moreMovesComing;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // End stops
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void setEndStops(AxisEndstopChecks endstops)
    {
        _endstops = endstops;
    }
    void setTestAllEndStops()
    {
        _endstops.all();
        LOG_I("MotionArgs", "Test all endstops");
    }

    void setTestNoEndStops()
    {
        _endstops.clear();
    }

    void setTestEndStopsDefault()
    {
        _endstops.clear();
    }

    void setTestEndStop(int axisIdx, int endStopIdx, AxisEndstopChecks::AxisMinMaxEnum checkType)
    {
        _endstops.set(axisIdx, endStopIdx, checkType);
    }

    const AxisEndstopChecks &getEndstopCheck() const
    {
        return _endstops;
    }

    void fromJSON(const char* jsonStr);
    String toJSON();

private:

    // Version of this structure
    uint8_t _motionArgsStructVersion = 0;

    // Flags
    bool _isRelative = false;
    bool _rampedMotion = true;
    bool _unitsAreSteps = false;
    bool _dontSplitMove = false;
    bool _extrudeValid = false;
    bool _targetSpeedValid = false;
    bool _moveClockwise = false;
    bool _moveRapid = false;
    bool _moreMovesComing = false;
    bool _isHoming = false;
    bool _motionTrackingIndexValid = false;
    bool _feedrateUnitsPerMin = false;
    bool _enableMotors = false;
    bool _preClearMotionQueue = false;
    bool _stopMotion = false;
    bool _constrainToBounds = false;
    bool _minimizeMotion = true;

    // Boolean flags
    class FieldDefType {
    public:
        FieldDefType(const char* name, void* pValue, const char* dataType)
        {
            _name = name;
            _pValue = pValue;
            _dataType = dataType;
        }
        String _name;
        void* _pValue;
        String _dataType;
    };
    std::vector<FieldDefType> getFieldDefs();

    // Target speed (like an absolute feedrate)
    double _targetSpeed = 0;

    // Extrude distance
    double _extrudeDistance = 0;

    // Feedrate is a percentage (unless _feedrateUnitsPerMin is set)
    double _feedrate = 100;

    // Current as percentage of max current
    double _ampsPercentOfMax = 0;

    // Motion tracking index - used to track execution of motion requests
    int _motionTrackingIdx = 0;

    // End stops
    AxisEndstopChecks _endstops;

    // Coords
    // When _unitsAreSteps flag is true these represent the position in steps
    // When _unitsAreSteps flag is false units are axes units (defined in axes config)
    AxesValues<AxisPosDataType> _axesPos;

    // When used as a command argument some axes may not be specified
    AxesValues<AxisSpecifiedDataType> _axesSpecified;

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionArgs";
};

#pragma pack(pop)
