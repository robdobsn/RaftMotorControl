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

// #define DEBUG_AXES_PARAMS

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

    /// @brief Get number of axes defined
    /// @return Number of axes
    uint32_t getNumAxes() const
    {
        return _axisParams.size();
    }

    /// @brief Get steps per unit for an axis
    /// @param axisIdx Axis index
    /// @return Steps per unit for the axis
    /// @note this should not be in the normal steps data type as it may be a fraction and step counts may be integers
    double getStepsPerUnit(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::stepsPerRot_default / AxisParams::posUnitsPerRot_default;
        return _axisParams[axisIdx].stepsPerUnit();
    }

    /// @brief Get steps per rotation for an axis
    /// @param axisIdx Axis index
    /// @return Steps per rotation for the axis
    AxisStepsDataType getStepsPerRot(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::stepsPerRot_default;
        return _axisParams[axisIdx]._stepsPerRot;
    }

    /// @brief Get units per rotation for an axis
    /// @param axisIdx Axis index
    /// @return Units per rotation for the axis
    AxisPosDataType getunitsPerRot(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::posUnitsPerRot_default;
        return _axisParams[axisIdx]._unitsPerRot;
    }

    /// @brief Get max units for an axis
    /// @param axisIdx Axis index
    /// @return Max units for the axis
    AxisPosDataType getMaxUnits(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return 0;
        return _axisParams[axisIdx]._maxUnits;
    }

    /// @brief Get min units for an axis
    /// @param axisIdx Axis index
    /// @return Min units for the axis
    AxisPosDataType getMinUnits(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return false;
        return _axisParams[axisIdx]._minUnits;
    }

    /// @brief Get max speed for an axis in units per second
    /// @param axisIdx Axis index
    /// @return Max speed for the axis in units per second
    AxisSpeedDataType getMaxSpeedUps(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::maxVelocityUps_default;
        return _axisParams[axisIdx]._maxSpeedUps;
    }

    /// @brief Get min speed for an axis in units per second
    /// @param axisIdx Axis index
    /// @return Min speed for the axis in units per second
    AxisSpeedDataType getMinSpeedUps(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::minVelocityUps_default;
        return _axisParams[axisIdx]._minSpeedUps;
    }

    /// @brief Get max step rate for an axis in steps per second
    /// @param axisIdx Axis index
    /// @param forceRecalc If true, forces recalculation of the max step rate based on current axis parameters instead of using cached value
    /// @return Max step rate for the axis in steps per second
    AxisStepRateDataType getMaxStepRatePerSec(uint32_t axisIdx, bool forceRecalc = false) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::maxDegreesPerSecond_default * AxisParams::stepsPerRot_default / 360;
        if (forceRecalc)
            return _axisParams[axisIdx]._maxDegreesPerSec * _axisParams[axisIdx]._stepsPerRot / 360;
        return _maxStepRatesPerSec.getVal(axisIdx);
    }

    /// @brief Get homing step rate per second for an axis in steps per second
    /// @param axisIdx Axis index
    /// @return Homing step rate for the axis in steps per second
    AxisStepRateDataType getHomingStepRatePerSec(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::homingDegreesPerSecond_default * AxisParams::stepsPerRot_default / 360;
        return _axisParams[axisIdx]._homingDegreesPerSec * _axisParams[axisIdx]._stepsPerRot / 360;
    }

    /// @brief Get max acceleration for an axis in units per second squared
    /// @param axisIdx Axis index
    /// @return Max acceleration for the axis in units per second squared
    AxisAccDataType getMaxAccelUps2(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return AxisParams::maxAccUps2_default;
        return _axisParams[axisIdx]._maxAccelUps2;
    }

    /// @brief Get max block distance in millimeters
    /// @return Max block distance in millimeters
    double getMaxBlockDistMM() const
    {
        return _maxBlockDistMM;
    }

    /// @brief Get geometry type
    /// @return Geometry type string (e.g. "XY" - as registered with factory)
    String getGeometry() const
    {
        return _geometry;
    }

    /// @brief Check if out-of-bounds points are allowed
    /// @return True if out-of-bounds points are allowed, false otherwise
    bool allowOutOfBounds() const
    {
        return _outOfBoundsDefault == OutOfBoundsAction::ALLOW;
    }
    
    /// @brief Get default action for out-of-bounds points
    /// @return Default action for out-of-bounds points
    OutOfBoundsAction getOutOfBoundsDefault() const
    {
        return _outOfBoundsDefault;
    }

    /// @brief Get max junction deviation in millimeters
    /// @return Max junction deviation in millimeters
    AxisPosDataType getMaxJunctionDeviationMM() const
    {
        return _maxJunctionDeviationMM;
    }

    /// @brief Check if homing is needed before any move
    /// @param axisIdx Axis index
    /// @return True if homing is needed before any move, false otherwise
    bool isPrimaryAxis(uint32_t axisIdx) const
    {
        if (axisIdx >= _axisParams.size())
            return false;
        return _axisParams[axisIdx]._isPrimaryAxis;
    }

    /// @brief Check if point is within bounds for all axes
    /// @param pt 
    /// @return True if point is within bounds for all axes, false otherwise
    bool ptInBounds(const AxesValues<AxisPosDataType>& pt) const
    {
        bool isValid = true;
        for (uint32_t axisIdx = 0; (axisIdx < _axisParams.size()) && (axisIdx < pt.numAxes()); axisIdx++)
            isValid = isValid && _axisParams[axisIdx].ptInBounds(pt[axisIdx]);
        return isValid;
    }

    /// @brief Constrain point to bounds for all axes
    /// @param pt Point to constrain (modified in place)
    void constrainPtToBounds(AxesValues<AxisPosDataType>& pt) const
    {
        for (uint32_t axisIdx = 0; (axisIdx < _axisParams.size()) && (axisIdx < pt.numAxes()); axisIdx++)
            pt.setVal(axisIdx, _axisParams[axisIdx].getNearestInBoundsValue(pt[axisIdx]));
    }

    /// @brief Set up axes parameters from JSON configuration
    /// @param config 
    /// @return True if setup was successful, false otherwise
    bool setupAxes(const RaftJsonIF& config)
    {
        // Clear existing
        _axisParams.clear();

        // Get params related to kinematics
        _geometry = config.getString("motion/geom", "XYZ");
        _maxBlockDistMM = config.getDouble("motion/blockDistMM", _maxBlockDistanceMM_default);
        _maxJunctionDeviationMM = config.getDouble("motion/maxJunctionDeviationMM", maxJunctionDeviationMM_default);
        _homingNeededBeforeAnyMove = config.getBool("motion/homeBeforeMove", true);
        
        // Parse outOfBounds string
        String oobStr = config.getString("motion/outOfBounds", "discard");
        if (oobStr == "allow" || oobStr == "ok")
            _outOfBoundsDefault = OutOfBoundsAction::ALLOW;
        else if (oobStr == "clamp" || oobStr == "constrain")
            _outOfBoundsDefault = OutOfBoundsAction::CLAMP;
        else
            _outOfBoundsDefault = OutOfBoundsAction::DISCARD;

#ifdef DEBUG_AXES_PARAMS
        // Debug
        LOG_I(MODULE_PREFIX, "setupAxes geom %s blockDistMM %0.2f (0=no-max) homeBefMove %s jnDev %0.2fmm outOfBounds %s",
               _geometry.c_str(), _maxBlockDistMM,
               _homingNeededBeforeAnyMove ? "Y" : "N",
               _maxJunctionDeviationMM,
                _outOfBoundsDefault == OutOfBoundsAction::ALLOW ? "allow" :
                _outOfBoundsDefault == OutOfBoundsAction::CLAMP ? "clamp" : "discard");
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
                    LOG_I(MODULE_PREFIX, "Axis %d max step rate: %0.2f steps/sec", i, _maxStepRatesPerSec.getVal(i));
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

    /// @brief Set master axis index based on dominant or primary axis flags, or fallback to first axis
    /// @param fallbackAxisIdx Axis index to use if no dominant or primary axis is found
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

    /// @brief Get master axis index
    /// @return Master axis index
    int getMasterAxisIdx() const
    {
        return _masterAxisIdx;
    }

    /// @brief Get the maximum acceleration of the master axis
    /// @return Maximum acceleration of the master axis in units per second squared
    AxisAccDataType masterAxisMaxAccel() const
    {
        return _masterAxisMaxAccUps2;
    }

    /// @brief Get the maximum speed of the master axis
    /// @return Maximum speed of the master axis in units per second
    AxisSpeedDataType masterAxisMaxSpeed() const
    {
        if (_masterAxisIdx != -1)
            return getMaxSpeedUps(_masterAxisIdx);
        return getMaxSpeedUps(0);
    }

    // Defaults
    static constexpr double _maxBlockDistanceMM_default = 10.0f;
    static constexpr double maxJunctionDeviationMM_default = 0.05f;

private:

    static constexpr const char* MODULE_PREFIX = "AxesParams";

    // Kinematics
    String _geometry;
    double _maxBlockDistMM = _maxBlockDistanceMM_default;
    bool _homingNeededBeforeAnyMove = true;
    double _maxJunctionDeviationMM = maxJunctionDeviationMM_default;
    OutOfBoundsAction _outOfBoundsDefault = OutOfBoundsAction::DISCARD;

    // Axis parameters
    std::vector<AxisParams> _axisParams;

    // Master axis
    int _masterAxisIdx = 0;

    // Cache values for master axis as they are used frequently in the planner
    AxisAccDataType _masterAxisMaxAccUps2 = AxisParams::maxAccUps2_default;

    // Cache max step rate
    AxesValues<AxisStepRateDataType> _maxStepRatesPerSec;
};
