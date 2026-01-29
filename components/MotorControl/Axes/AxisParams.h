/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisParams
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftJson.h"
#include "Logger.h"
#include "RaftUtils.h"
#include "AxesValues.h"
#include "MotorControlTypes.h"

// This class holds the parameters for a single axis of a machine which may be driven by a stepper or servo
// The parameters are used to convert between machine units and steps using kinematics which are specific to
// the machine configuration and may include cartesian, SCARA, etc.
// Some parameters are specific to particular types of machine (e.g. activeLenMM for a SCARA arm)
// Units may be mm or degrees, etc. depending on the machine configuration
// For a cartesian machine the units are typically mm and for a SCARA machine the units are typically degrees
// The term Ups means units per second and Ups2 means units per second squared (acceleration)

class AxisParams
{
public:
    static constexpr AxisSpeedDataType maxVelocityUps_default = 100.0f;
    static constexpr AxisSpeedDataType minVelocityUps_default = 0.0f;
    static constexpr AxisAccDataType maxAccUps2_default = 100.0f;
    static constexpr AxisStepsFactorDataType stepsPerRot_default = 1.0f;
    static constexpr AxisPosFactorDataType posUnitsPerRot_default = 1.0f;
    static constexpr AxisRPMDataType maxRPM_default = 300.0f;
    static constexpr AxisPosDataType originOffsetUnits_default = 0.0f;
    static constexpr AxisStepsDataType stepsForAxisHoming_default = 100000;

    // Max and min speed in units per second
    AxisSpeedDataType _maxSpeedUps;
    AxisSpeedDataType _minSpeedUps;

    // Max acceleration in units per second squared
    AxisAccDataType _maxAccelUps2;

    // Steps per rotation and units per rotation
    AxisStepsFactorDataType _stepsPerRot;
    AxisPosFactorDataType _unitsPerRot;

    // Max RPM
    AxisRPMDataType _maxRPM;

    // Min and max values for the axis in units
    AxisPosDataType _minUnits;
    AxisPosDataType _maxUnits;
    
    // Flags indicating if bounds were explicitly set
    bool _minUnitsSet;
    bool _maxUnitsSet;

    // Axis type
    bool _isPrimaryAxis;
    bool _isDominantAxis;
    // A servo axis is one which does not require blockwise stepping to a destination
    bool _isServoAxis;

public:
    AxisParams()
    {
        clear();
    }

    void clear()
    {
        _maxSpeedUps = maxVelocityUps_default;
        _minSpeedUps = minVelocityUps_default;
        _maxAccelUps2 = maxAccUps2_default;
        _stepsPerRot = stepsPerRot_default;
        _unitsPerRot = posUnitsPerRot_default;
        _maxRPM = maxRPM_default;
        _minUnits = 0;
        _maxUnits = 0;
        _minUnitsSet = false;
        _maxUnitsSet = false;
        _isPrimaryAxis = true;
        _isDominantAxis = false;
        _isServoAxis = false;
    }

    AxisStepsFactorDataType stepsPerUnit() const
    {
        if (_unitsPerRot != 0)
            return _stepsPerRot / _unitsPerRot;
        return 1;
    }

    bool ptInBounds(const AxisPosDataType &val) const
    {
        // Only check bounds that were explicitly set
        if (_minUnitsSet && val < _minUnits)
            return false;
        if (_maxUnitsSet && val > _maxUnits)
            return false;
        return true;  // No bounds set or within bounds
    }

    AxisPosDataType getNearestInBoundsValue(AxisPosDataType val) const
    {
        if (_minUnitsSet && val < _minUnits)
            return _minUnits;
        if (_maxUnitsSet && val > _maxUnits)
            return _maxUnits;
        return val;
    }

    void setFromJSON(const char *axisJSON)
    {
        RaftJson config(axisJSON);
        _maxSpeedUps = AxisSpeedDataType(config.getDouble("maxSpeedUps", AxisParams::maxVelocityUps_default));
        _maxAccelUps2 = AxisAccDataType(config.getDouble("maxAccUps2", AxisParams::maxAccUps2_default));
        _stepsPerRot = AxisStepsFactorDataType(config.getDouble("stepsPerRot", AxisParams::stepsPerRot_default));
        _unitsPerRot = AxisPosFactorDataType(config.getDouble("unitsPerRot", AxisParams::posUnitsPerRot_default));
        _maxRPM = AxisRPMDataType(config.getDouble("maxRPM", AxisParams::maxRPM_default));
        
        // Check if bounds were explicitly set and parse them
        _minUnitsSet = config.contains("minUnits");
        _maxUnitsSet = config.contains("maxUnits");
        if (_minUnitsSet)
            _minUnits = AxisPosDataType(config.getDouble("minUnits", 0));
        if (_maxUnitsSet)
            _maxUnits = AxisPosDataType(config.getDouble("maxUnits", 0));
            
        _isDominantAxis = config.getBool("isDominantAxis", 0);
        _isPrimaryAxis = config.getBool("isPrimaryAxis", 1);
        _isServoAxis = config.getBool("isServoAxis", 0);
    }

    void debugLog(int axisIdx)
    {
        static const char* MODULE_PREFIX = "AxisParams";
        LOG_I(MODULE_PREFIX, "Axis%d params maxSpeed %0.2f acceleration %0.2f stepsPerRot %0.2f unitsPerRot %0.2f maxRPM %0.2f",
                   axisIdx, _maxSpeedUps, _maxAccelUps2, _stepsPerRot, _unitsPerRot, _maxRPM);
        LOG_I(MODULE_PREFIX, "Axis%d params minVal %0.2f maxVal %0.2f isDominant %d isServo %d",
                   axisIdx, _minUnits, _maxUnits, _isDominantAxis, _isServoAxis);
    }
};
