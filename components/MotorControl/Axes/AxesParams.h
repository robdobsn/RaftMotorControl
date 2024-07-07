/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxesParams
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxisParams.h"
#include "AxesValues.h"
#include <vector>

#define DEBUG_AXES_PARAMS

class AxesParams
{
public:
    AxesParams()
    {
        clearAxes();
    }

    void clearAxes()
    {
        _masterAxisIdx = -1;
        _masterAxisMaxAccUps2 = AxisParams::maxAccUps2_default;
        _axisParams.clear();
    }

    // Note that this should not be in the normal steps data type as it may be a fraction and
    // step counts may be integers
    double getStepsPerUnit(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::stepsPerRot_default / AxisParams::posUnitsPerRot_default;
        return _axisParams[axisIdx].stepsPerUnit();
    }

    AxisStepsDataType getStepsPerRot(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::stepsPerRot_default;
        return _axisParams[axisIdx]._stepsPerRot;
    }

    AxisPosDataType getunitsPerRot(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::posUnitsPerRot_default;
        return _axisParams[axisIdx]._unitsPerRot;
    }

    AxisPosDataType getMaxUnits(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return 0;
        return _axisParams[axisIdx]._maxUnits;
    }

    AxisPosDataType getMinUnits(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return false;
        return _axisParams[axisIdx]._minUnits;
    }

    AxisSpeedDataType getMaxSpeedUps(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::maxVelocityUps_default;
        return _axisParams[axisIdx]._maxSpeedUps;
    }

    AxisSpeedDataType getMinSpeedUps(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::minVelocityUps_default;
        return _axisParams[axisIdx]._minSpeedUps;
    }

    AxisStepRateDataType getMaxStepRatePerSec(uint32_t axisIdx, bool forceRecalc = false) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::maxRPM_default * AxisParams::stepsPerRot_default / 60;
        if (forceRecalc)
            return _axisParams[axisIdx]._maxRPM * _axisParams[axisIdx]._stepsPerRot / 60;
        return _maxStepRatesPerSec.getVal(axisIdx);
    }

    AxisAccDataType getMaxAccelUps2(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::maxAccUps2_default;
        return _axisParams[axisIdx]._maxAccelUps2;
    }

    String getGeometry() const
    {
        return _geometry;
    }

    AxisPosDataType getMaxJunctionDeviationMM() const
    {
        return _maxJunctionDeviationMM;
    }

    bool isPrimaryAxis(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return false;
        return _axisParams[axisIdx]._isPrimaryAxis;
    }

    bool ptInBounds(const AxesValues<AxisPosDataType>& pt) const
    {
        bool isValid = true;
        for (uint32_t axisIdx = 0; (axisIdx < _axisParams.size()) && (axisIdx < pt.numAxes()); axisIdx++)
            isValid = isValid && _axisParams[axisIdx].ptInBounds(pt[axisIdx]);
        return isValid;
    }

    bool setupAxes(const RaftJsonIF& config)
    {
        // Clear existing
        _axisParams.clear();

        // Get params related to kinematics
        _geometry = config.getString("motion/geom", "XYZ");
        _maxBlockDistMM = config.getDouble("motion/blockDistMM", _maxBlockDistanceMM_default);
        _maxJunctionDeviationMM = config.getDouble("motion/maxJunctionDeviationMM", maxJunctionDeviationMM_default);
        _homingNeededBeforeAnyMove = config.getBool("motion/homeBeforeMove", true);

#ifdef DEBUG_AXES_PARAMS
        // Debug
        LOG_I(MODULE_PREFIX, "setupAxes geom %s blockDistMM %0.2f (0=no-max) homeBefMove %s jnDev %0.2fmm",
               _geometry.c_str(), _maxBlockDistMM,
               _homingNeededBeforeAnyMove ? "Y" : "N",
               _maxJunctionDeviationMM);
#endif
        
        // Extract sub-system elements
        std::vector<String> axesVec;
        if (config.getArrayElems("axes", axesVec))
        {
            // Check index ok
            uint32_t numAxesToAdd = axesVec.size();
            if (numAxesToAdd >= AXIS_VALUES_MAX_AXES)
                numAxesToAdd = AXIS_VALUES_MAX_AXES;

            // Resize appropriately
            _axisParams.resize(numAxesToAdd);
            uint32_t axisIdx = 0;
            for (RaftJson axisConfig : axesVec)
            {
                // Get params
                String paramsJson = axisConfig.getString("params", "{}");

                // Set the axis parameters
                _axisParams[axisIdx].setFromJSON(paramsJson.c_str());

                // Find the master axis (dominant one, or first primary - or just first)
                setMasterAxis(axisIdx);

                // Cache axis max step rate
                for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
                {
                    _maxStepRatesPerSec.setVal(i, getMaxStepRatePerSec(i, true));
                }

                // Next
                axisIdx++;
            }
        }
        return true;
    }

    void debugLog()
    {
        for (uint32_t axisIdx = 0; axisIdx < _axisParams.size(); axisIdx++)
            _axisParams[axisIdx].debugLog(axisIdx);
    }
    // Set the master axis either to the dominant axis (if there is one)
    // or just the first one found
    void setMasterAxis(int fallbackAxisIdx)
    {
        int dominantIdx = -1;
        int primaryIdx = -1;
        for (uint32_t i = 0; i < _axisParams.size(); i++)
        {
            if (_axisParams[i]._isDominantAxis)
            {
                dominantIdx = i;
                break;
            }
            if (_axisParams[i]._isPrimaryAxis)
            {
                if (primaryIdx == -1)
                    primaryIdx = i;
            }
        }
        if (dominantIdx != -1)
            _masterAxisIdx = dominantIdx;
        else if (primaryIdx != -1)
            _masterAxisIdx = primaryIdx;
        else if (_masterAxisIdx == -1)
            _masterAxisIdx = fallbackAxisIdx;

        // Cache values for master axis
        _masterAxisMaxAccUps2 = getMaxAccelUps2(_masterAxisIdx);
    }

    AxisAccDataType masterAxisMaxAccel() const
    {
        return _masterAxisMaxAccUps2;
    }

    AxisSpeedDataType masterAxisMaxSpeed() const
    {
        if (_masterAxisIdx != -1)
            return getMaxSpeedUps(_masterAxisIdx);
        return getMaxSpeedUps(0);
    }

    // Defaults
    static constexpr double _maxBlockDistanceMM_default = 0.0f;
    static constexpr double maxJunctionDeviationMM_default = 0.05f;

private:

    static constexpr const char* MODULE_PREFIX = "AxesParams";

    // Kinematics
    String _geometry;
    double _maxBlockDistMM = _maxBlockDistanceMM_default;
    bool _homingNeededBeforeAnyMove = true;
    double _maxJunctionDeviationMM = maxJunctionDeviationMM_default;

    // Axis parameters
    std::vector<AxisParams> _axisParams;

    // Master axis
    int _masterAxisIdx = 0;

    // Cache values for master axis as they are used frequently in the planner
    AxisAccDataType _masterAxisMaxAccUps2 = AxisParams::maxAccUps2_default;

    // Cache max step rate
    AxesValues<AxisStepRateDataType> _maxStepRatesPerSec;
};
