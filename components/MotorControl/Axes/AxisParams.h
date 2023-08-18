/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisParams
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftJson.h"
#include <Logger.h>
#include <RaftUtils.h>
#include "AxisValues.h"

class AxisParams
{
public:
    static constexpr AxisVelocityDataType maxVelocity_default = 100.0f;
    static constexpr AxisVelocityDataType minVelocity_default = 0.0f;
    static constexpr AxisAccDataType acceleration_default = 100.0f;
    static constexpr AxisStepsFactorDataType stepsPerRot_default = 1.0f;
    static constexpr AxisPosFactorDataType posUnitsPerRot_default = 1.0f;
    static constexpr AxisRPMDataType maxRPM_default = 300.0f;
    static constexpr AxisPosDataType homeOffsetVal_default = 0.0f;
    static constexpr AxisStepsDataType homeOffSteps_default = 0;
    static constexpr AxisStepsDataType stepsForAxisHoming_default = 100000;

    // Parameters
    AxisVelocityDataType _maxVelocityUnitsPerSec;
    AxisVelocityDataType _minVelocityUnitsPerSec;
    AxisAccDataType _maxAccelUnitsPerSec2;
    AxisStepsFactorDataType _stepsPerRot;
    AxisPosFactorDataType _unitsPerRot;
    AxisRPMDataType _maxRPM;
    bool _minValValid;
    AxisPosDataType _minVal;
    bool _maxValValid;
    AxisPosDataType _maxVal;
    bool _isPrimaryAxis;
    bool _isDominantAxis;
    // A servo axis is one which does not require blockwise stepping to a destination
    bool _isServoAxis;
    AxisPosDataType _homeOffsetVal;
    AxisStepsDataType _homeOffSteps;

  public:
    AxisParams()
    {
        clear();
    }

    void clear()
    {
        _maxVelocityUnitsPerSec = maxVelocity_default;
        _minVelocityUnitsPerSec = minVelocity_default;
        _maxAccelUnitsPerSec2 = acceleration_default;
        _stepsPerRot = stepsPerRot_default;
        _unitsPerRot = posUnitsPerRot_default;
        _maxRPM = maxRPM_default;
        _minValValid = false;
        _minVal = 0;
        _maxValValid = false;
        _maxVal = 0;
        _isPrimaryAxis = true;
        _isDominantAxis = false;
        _isServoAxis = false;
        _homeOffsetVal = homeOffsetVal_default;
        _homeOffSteps = homeOffSteps_default;
    }

    AxisStepsFactorDataType stepsPerUnit() const
    {
        if (_unitsPerRot != 0)
            return _stepsPerRot / _unitsPerRot;
        return 1;
    }

    bool ptInBounds(AxisPosDataType &val, bool correctValueInPlace) const
    {
        bool wasValid = true;
        if (_minValValid && val < _minVal)
        {
            wasValid = false;
            if (correctValueInPlace)
                val = _minVal;
        }
        if (_maxValValid && val > _maxVal)
        {
            wasValid = false;
            if (correctValueInPlace)
                val = _maxVal;
        }
        return wasValid;
    }

    void setFromJSON(const char *axisJSON)
    {
        // Stepper motor
        _maxVelocityUnitsPerSec = AxisVelocityDataType(RaftJson::getDouble("maxSpeed", AxisParams::maxVelocity_default, axisJSON));
        _maxAccelUnitsPerSec2 = AxisAccDataType(RaftJson::getDouble("maxAcc", AxisParams::acceleration_default, axisJSON));
        _stepsPerRot = AxisStepsFactorDataType(RaftJson::getDouble("stepsPerRot", AxisParams::stepsPerRot_default, axisJSON));
        _unitsPerRot = AxisPosFactorDataType(RaftJson::getDouble("unitsPerRot", AxisParams::posUnitsPerRot_default, axisJSON));
        _maxRPM = AxisRPMDataType(RaftJson::getDouble("maxRPM", AxisParams::maxRPM_default, axisJSON));
        _minVal = AxisPosDataType(RaftJson::getDouble("minVal", 0, _minValValid, axisJSON));
        _maxVal = AxisPosDataType(RaftJson::getDouble("maxVal", 0, _maxValValid, axisJSON));
        _homeOffsetVal = AxisPosDataType(RaftJson::getDouble("homeOffsetVal", 0, axisJSON));
        _homeOffSteps = AxisStepsDataType(RaftJson::getDouble("homeOffSteps", 0, axisJSON));
        _isDominantAxis = RaftJson::getBool("isDominantAxis", 0, axisJSON);
        _isPrimaryAxis = RaftJson::getBool("isPrimaryAxis", 1, axisJSON);
        _isServoAxis = RaftJson::getBool("isServoAxis", 0, axisJSON);
    }

    void debugLog(int axisIdx)
    {
        static const char* MODULE_PREFIX = "AxisParams";
        LOG_I(MODULE_PREFIX, "Axis%d params maxSpeed %0.2f, acceleration %0.2f, stepsPerRot %0.2f, unitsPerRot %0.2f, maxRPM %0.2f",
                   axisIdx, _maxVelocityUnitsPerSec, _maxAccelUnitsPerSec2, _stepsPerRot, _unitsPerRot, _maxRPM);
        LOG_I(MODULE_PREFIX, "Axis%d params minVal %0.2f (%d), maxVal %0.2f (%d), isDominant %d, isServo %d, homeOffVal %0.2f, homeOffSteps %d",
                   axisIdx, _minVal, _minValValid, _maxVal, _maxValValid, _isDominantAxis, _isServoAxis, _homeOffsetVal, _homeOffSteps);
    }
};
