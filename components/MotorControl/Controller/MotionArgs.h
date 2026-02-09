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

#define DEBUG_MOTION_ARGS_SPEED_PARSING

/// @brief Motion mode enumeration
enum class MotionModeEnum
{
    ABS,            // Absolute position in units (cartesian)
    REL,            // Relative position in units (cartesian)
    ABS_STEPS,  // Absolute position in steps (bypasses kinematics)
    REL_STEPS,  // Relative position in steps (bypasses kinematics)
    VEL,            // Velocity in units
    VEL_STEPS,      // Velocity in steps
    PROP_ABS,           // Proportionate (0-1 mapping to axis min/max)
    PROP_REL,       // Proportionate relative
    UNKNOWN         // Unknown/invalid mode
};

/// @brief Speed unit type enumeration
enum class SpeedUnitType
{
    NONE,            // Speed not specified
    PERCENTAGE,      // 0-100 percentage of max speed
    UNITS_PER_SEC,   // Absolute speed in axis units/sec (degrees, mm, etc.)
    STEPS_PER_SEC    // Absolute speed in steps/sec
};

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
        _mode = MotionModeEnum::ABS;

        // Speed (NONE means not specified - use default from config)
        _speedValue = 0.0;
        _speedUnitType = SpeedUnitType::NONE;

        // Motor current (0 means use default)
        _motorCurrent = 0;

        // Flags
        _dontSplitMove = false;
        _homingMove = false;
        _moreMovesComing = false;
        _motionTrackingIndexValid = false;
        _outOfBoundsAction = OutOfBoundsAction::USE_DEFAULT;
        _immediateExecution = false;

        // Reset values
        _motionTrackingIdx = 0;
        _axesPos.clear();
        _velocities.clear();
        _axesSpecified.clear();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Motion Mode
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /// @brief Set motion mode from string (e.g. "abs", "rel", "pos-rel-steps", etc.)
    /// @param mode Mode string
    void setMode(const String& mode) 
    { 
        _mode = stringToMode(mode);
    }
    
    /// @brief Set motion mode from enum
    /// @param mode Mode enum value
    void setMode(MotionModeEnum mode) 
    { 
        _mode = mode;
    }
    
    /// @brief Get motion mode as enum
    /// @return Mode enum value
    MotionModeEnum getMode() const 
    { 
        return _mode; 
    }
    
    /// @brief Get motion mode as string
    /// @return Mode string
    String getModeString() const 
    { 
        return modeToString(_mode); 
    }
    
    bool isRelative() const
    {
        return _mode == MotionModeEnum::REL || _mode == MotionModeEnum::REL_STEPS;
    }
    
    bool areUnitsSteps() const
    {
        return _mode == MotionModeEnum::ABS_STEPS || 
               _mode == MotionModeEnum::REL_STEPS ||
               _mode == MotionModeEnum::VEL_STEPS;
    }
    
    bool isRamped() const
    {
        // Step-based modes are never ramped (ramping requires cartesian coords for kinematics)
        return !areUnitsSteps();
    }
    
    bool isVelocityMode() const
    {
        return _mode == MotionModeEnum::VEL || _mode == MotionModeEnum::VEL_STEPS;
    }
    
    bool areVelocityUnitsSteps() const
    {
        return _mode == MotionModeEnum::VEL_STEPS;
    }
    
    bool isProportionate() const
    {
        return _mode == MotionModeEnum::PROP_ABS || _mode == MotionModeEnum::PROP_REL;
    }
    
    /// @brief Convert mode string to enum
    /// @param modeStr Mode string
    /// @return Mode enum value
    static MotionModeEnum stringToMode(const String& modeStr)
    {
        if (modeStr == "abs")
            return MotionModeEnum::ABS;
        else if (modeStr == "rel")
            return MotionModeEnum::REL;
        else if (modeStr == "pos-abs-steps")
            return MotionModeEnum::ABS_STEPS;
        else if (modeStr == "pos-rel-steps")
            return MotionModeEnum::REL_STEPS;
        else if (modeStr == "vel")
            return MotionModeEnum::VEL;
        else if (modeStr == "vel-steps")
            return MotionModeEnum::VEL_STEPS;
        else if (modeStr == "prop" || modeStr == "prop-abs")
            return MotionModeEnum::PROP_ABS;
        else if (modeStr == "prop-rel")
            return MotionModeEnum::PROP_REL;
        else
            return MotionModeEnum::UNKNOWN;
    }
    
    /// @brief Convert mode enum to string
    /// @param mode Mode enum value
    /// @return Mode string
    static String modeToString(MotionModeEnum mode)
    {
        switch (mode)
        {
            case MotionModeEnum::ABS:           return "abs";
            case MotionModeEnum::REL:           return "rel";
            case MotionModeEnum::ABS_STEPS: return "abs-steps";
            case MotionModeEnum::REL_STEPS: return "rel-steps";
            case MotionModeEnum::VEL:           return "vel";
            case MotionModeEnum::VEL_STEPS:     return "vel-steps";
            case MotionModeEnum::PROP_ABS:          return "prop-abs";
            case MotionModeEnum::PROP_REL:      return "prop-rel";
            default:                            return "unknown";
        }
    }

    /// @brief Set motion speed as a string (e.g. "10mmps", "500upm", "80pc", etc.)
    /// @param speed Speed string - parsed and stored as value + unit type
    void setSpeed(const String& speed) 
    { 
        parseSpeedString(speed);
    }
    
    /// @brief Set speed directly as units per second
    /// @param speedUps Speed in units per second
    void setSpeedUps(double speedUps)
    {
        _speedValue = speedUps;
        _speedUnitType = SpeedUnitType::UNITS_PER_SEC;
    }
    
    /// @brief Set speed directly as steps per second
    /// @param speedSps Speed in steps per second
    void setSpeedStepsPerSec(double speedSps)
    {
        _speedValue = speedSps;
        _speedUnitType = SpeedUnitType::STEPS_PER_SEC;
    }
    
    /// @brief Set speed as percentage of max
    /// @param percent Percentage (0-100)
    void setSpeedPercent(double percent)
    {
        _speedValue = percent;
        _speedUnitType = SpeedUnitType::PERCENTAGE;
    }

    /// @brief Get motion speed as string (for JSON serialization)
    /// @return Speed string
    String getSpeedString() const 
    { 
        switch (_speedUnitType)
        {
            case SpeedUnitType::UNITS_PER_SEC:
                return String(_speedValue) + "ups";
            case SpeedUnitType::STEPS_PER_SEC:
                return String(_speedValue) + "sps";
            case SpeedUnitType::PERCENTAGE:
            default:
                return String(_speedValue);
        }
    }
    
    /// @brief Get speed unit type
    /// @return Speed unit type enum
    SpeedUnitType getSpeedUnitType() const { return _speedUnitType; }
    
    /// @brief Get raw speed value (without conversion)
    /// @return Speed value in its native units
    double getSpeedValue() const { return _speedValue; }

    /// @brief Get speed in units/second (for ramped/cartesian motion)
    /// @param configMaxSpeedUps Maximum configured speed in units/sec for capping
    /// @return Speed in units/sec, automatically capped by configMaxSpeedUps
    double getSpeedUps(double configMaxSpeedUps) const
    {
        double requestedSpeed;
        
        switch (_speedUnitType)
        {
            case SpeedUnitType::PERCENTAGE:
                requestedSpeed = configMaxSpeedUps * (_speedValue / 100.0);
                break;
            case SpeedUnitType::UNITS_PER_SEC:
                requestedSpeed = _speedValue;
                break;
            case SpeedUnitType::STEPS_PER_SEC:
                // Steps/sec requested but we need units/sec - caller should use getSpeedStepsPerSec instead
                // For safety, return the value directly (caller may know the conversion)
                requestedSpeed = _speedValue;
                break;
            default:
                requestedSpeed = configMaxSpeedUps;
                break;
        }
        
        // Always cap by configuration maximum (safety limit)
        if (requestedSpeed > configMaxSpeedUps)
            requestedSpeed = configMaxSpeedUps;
            
#ifdef DEBUG_MOTION_ARGS_SPEED_PARSING
        LOG_I(MODULE_PREFIX, "getSpeedUps: value=%f type=%d -> %f ups (max %f)", 
              _speedValue, (int)_speedUnitType, requestedSpeed, configMaxSpeedUps);
#endif

        return requestedSpeed;
    }
    
    /// @brief Get speed in steps/second (for step-based motion)
    /// @param configMaxStepsPerSec Maximum configured speed in steps/sec for capping
    /// @param stepsPerUnit Conversion factor from units to steps (e.g., steps per degree)
    /// @return Speed in steps/sec, automatically capped by configMaxStepsPerSec
    double getSpeedStepsPerSec(double configMaxStepsPerSec, double stepsPerUnit = 1.0) const
    {
        double requestedSpeed;
        
        switch (_speedUnitType)
        {
            case SpeedUnitType::PERCENTAGE:
                requestedSpeed = configMaxStepsPerSec * (_speedValue / 100.0);
                break;
            case SpeedUnitType::UNITS_PER_SEC:
                // Convert units/sec to steps/sec
                requestedSpeed = _speedValue * stepsPerUnit;
                break;
            case SpeedUnitType::STEPS_PER_SEC:
                requestedSpeed = _speedValue;
                break;
            default:
                requestedSpeed = configMaxStepsPerSec;
                break;
        }
        
        // Always cap by configuration maximum (safety limit)
        if (requestedSpeed > configMaxStepsPerSec)
            requestedSpeed = configMaxStepsPerSec;
            
#ifdef DEBUG_MOTION_ARGS_SPEED_PARSING
        LOG_I(MODULE_PREFIX, "getSpeedStepsPerSec: value=%f type=%d stepsPerUnit=%f -> %f sps (max %f)", 
              _speedValue, (int)_speedUnitType, stepsPerUnit, requestedSpeed, configMaxStepsPerSec);
#endif

        return requestedSpeed;
    }

private:
    /// @brief Parse speed string and store as value + unit type
    /// @param speed Speed string to parse
    void parseSpeedString(const String& speed)
    {
        // Empty speed means use 100% of config max
        if (speed.length() == 0)
        {
            _speedValue = 100.0;
            _speedUnitType = SpeedUnitType::PERCENTAGE;
            return;
        }

        // Check if numeric (percentage)
        bool isNumeric = true;
        for (size_t i = 0; i < speed.length(); i++)
        {
            if (!isdigit(speed[i]) && speed[i] != '.' && speed[i] != '-')
            {
                isNumeric = false;
                break;
            }
        }

        double value = speed.toDouble();  // Extract numeric part
        
        if (isNumeric)
        {
            // Numeric value is percentage
            _speedValue = value;
            _speedUnitType = SpeedUnitType::PERCENTAGE;
        }
        else if (speed.endsWith("pc") || speed.endsWith("percent"))
        {
            _speedValue = value;
            _speedUnitType = SpeedUnitType::PERCENTAGE;
        }
        else if (speed.endsWith("ups") || speed.endsWith("unitsps"))
        {
            // Units per second (degrees per second, etc.)
            _speedValue = value;
            _speedUnitType = SpeedUnitType::UNITS_PER_SEC;
        }
        else if (speed.endsWith("upm") || speed.endsWith("unitspm"))
        {
            // Units per minute -> convert to per second
            _speedValue = value / 60.0;
            _speedUnitType = SpeedUnitType::UNITS_PER_SEC;
        }
        else if (speed.endsWith("mmps"))
        {
            // mm per second (units per second)
            _speedValue = value;
            _speedUnitType = SpeedUnitType::UNITS_PER_SEC;
        }
        else if (speed.endsWith("mmpm"))
        {
            // mm per minute -> convert to per second
            _speedValue = value / 60.0;
            _speedUnitType = SpeedUnitType::UNITS_PER_SEC;
        }
        else if (speed.endsWith("sps"))
        {
            // Steps per second
            _speedValue = value;
            _speedUnitType = SpeedUnitType::STEPS_PER_SEC;
        }
        else
        {
            // Unknown suffix, treat as percentage
            _speedValue = value;
            _speedUnitType = SpeedUnitType::PERCENTAGE;
        }
        
#ifdef DEBUG_MOTION_ARGS_SPEED_PARSING
        LOG_I(MODULE_PREFIX, "parseSpeedString '%s' -> value=%f type=%d", 
              speed.c_str(), _speedValue, (int)_speedUnitType);
#endif
    }

public:

    /// @brief Set motor current
    /// @param current 
    void setMotorCurrent(double current) 
    { 
        _motorCurrent = current; 
    }

    /// @brief Get motor current
    /// @return Motor current as percentage of max (0 means use default)
    double getMotorCurrent() const 
    { 
        return _motorCurrent; 
    }

    /// @brief Set do not split move flag
    /// @param flag True to set do not split move
    void setDoNotSplitMove(bool flag) 
    { 
        _dontSplitMove = flag; 
    }

    /// @brief Get do not split move flag
    /// @return True if move should not be split into smaller blocks
    bool isDoNotSplitMove() const { return _dontSplitMove; }
    
    /// @brief Set homing move flag
    /// @param flag True to set homing move
    void setHomingMove(bool flag) 
    { 
        _homingMove = flag; 
    }

    /// @brief Get homing move flag
    /// @return True if this is a homing move
    bool isHomingMove() const 
    { 
        return _homingMove; 
    }
    
    /// @brief Set out of bounds action for this move (overrides default for SysType)
    /// @param action Out of bounds action enum value
    void setOutOfBoundsAction(OutOfBoundsAction action) 
    { 
        _outOfBoundsAction = action; 
    }

    /// @brief Get out of bounds action for this move
    /// @return Out of bounds action enum value
    OutOfBoundsAction getOutOfBoundsAction() const 
    { 
        return _outOfBoundsAction; 
    }

    /// @brief Get effective out of bounds action for this move, using the default if motion args action is USE_DEFAULT
    /// @param defaultAction Default out of bounds action to use if motion args action is USE_DEFAULT
    /// @return Effective out of bounds action enum value
    OutOfBoundsAction getEffectiveOutOfBoundsAction(OutOfBoundsAction defaultAction) const
    {
        return (_outOfBoundsAction == OutOfBoundsAction::USE_DEFAULT) ? 
            defaultAction : _outOfBoundsAction;
    }
    
    /// @brief Set immediate execution flag (if true, stop, clear queue, then execute this motion)
    /// @param flag True to set immediate execution
    void setImmediateExecution(bool flag) 
    { 
        _immediateExecution = flag; 
    }

    /// @brief Get immediate execution flag
    /// @return True if this motion should be executed immediately (stop, clear queue, then execute this motion)
    bool isImmediateExecution() const 
    { 
        return _immediateExecution; 
    }

    /// @brief Get axes positions (for position modes)
    /// @return Reference to axes positions values
    AxesValues<AxisPosDataType>& getAxesPos() 
    { 
        return _axesPos; 
    }

    /// @brief Get axes positions (for position modes) as const reference
    /// @return Const reference to axes positions values
    const AxesValues<AxisPosDataType>& getAxesPosConst() const 
    { 
        return _axesPos; 
    }
   
    /// @brief Get axes specified flags (indicates which axes are specified in this move)
    /// @return Reference to axes specified flags
    AxesValues<AxisSpecifiedDataType>& getAxesSpecified() 
    { 
        return _axesSpecified; 
    }
    
    /// @brief Set axes positions and mark all specified (for position modes)
    /// @param axisPositions Const reference to axes positions values
    void setAxesPositions(const AxesValues<AxisPosDataType>& axisPositions)
    {
        _axesPos = axisPositions;
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
            _axesSpecified.setVal(axisIdx, true);
    }

    /// @brief Get velocities (for velocity modes)
    /// @return Reference to velocities values
    AxesValues<AxisPosDataType>& getVelocities() 
    { 
        return _velocities; 
    }

    /// @brief Get velocities (for velocity modes) as const reference
    /// @return Const reference to velocities values
    const AxesValues<AxisPosDataType>& getVelocitiesConst() const 
    { 
        return _velocities; 
    }

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

    // Motion mode enum
    MotionModeEnum _mode = MotionModeEnum::ABS;

    // Speed value and unit type (parsed from string)
    double _speedValue = 0.0;
    SpeedUnitType _speedUnitType = SpeedUnitType::NONE;

    // Motor current as percentage of max
    double _motorCurrent = 0;

    // Flags
    bool _dontSplitMove: 1 = false;
    bool _homingMove: 1 = false;
    bool _moreMovesComing: 1 = false;
    bool _motionTrackingIndexValid: 1 = false;
    bool _immediateExecution: 1 = false;  // Stop, clear queue, then execute this motion
    OutOfBoundsAction _outOfBoundsAction = OutOfBoundsAction::USE_DEFAULT;

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
