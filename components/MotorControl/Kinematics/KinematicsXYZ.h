/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// KinematicsXYZ
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftKinematics.h"
#include "AxesParams.h"

#define DEBUG_KINEMATICS_XYZ

class KinematicsXYZ : public RaftKinematics
{
public:

    /// @brief create method for factory
    /// @param config Configuration
    /// @return new instance of this class
    static RaftKinematics *create(const RaftJsonIF& config)
    {
        return new KinematicsXYZ(config);
    }

    /// @brief Constructor
    /// @param config Configuration
    KinematicsXYZ(const RaftJsonIF& config)
    {
    }

    /// @brief Convert a point in cartesian to actuator steps
    /// @param targetPt Target point cartesian from origin
    /// @param outActuator Output actuator in absolute steps from origin
    /// @param curAxesState Current position (in both units and steps from origin)
    /// @param axesParams Axes parameters
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                              AxesValues<AxisStepsDataType>& outActuator,
                              const AxesState& curAxesState,
                              const AxesParams& axesParams) const override final
    {
        // Check machine bounds
        bool pointIsValid = axesParams.ptInBounds(targetPt);
        if (!pointIsValid)
        {
            LOG_I(MODULE_PREFIX, "ptToActuator FAIL out of bounds");
            return false;
        }

        // Perform conversion
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            // Axis val from home point
            float axisValFromHome = targetPt.getVal(axisIdx);

            // Convert to steps
            outActuator.setVal(axisIdx, round(axisValFromHome * axesParams.getStepsPerUnit(axisIdx)));

            // TODO - decide if this origin steps needed
            // outActuator.setVal(axisIdx, round(axisValFromHome * axesParams.getStepsPerUnit(axisIdx) + axesParams.gethomeOffsetSteps(axisIdx)));

#ifdef DEBUG_KINEMATICS_XYZ
            LOG_I(MODULE_PREFIX, "ptToActuator axis%d %.2f-> %d",
                  axisIdx,
                  targetPt.getVal(axisIdx),
                  outActuator.getVal(axisIdx));
#endif
        }
        return true;
    }

    /// @brief Convert actuator steps to a point in cartesian
    /// @param targetActuator Target actuator steps
    /// @param outPt Output point in cartesian
    /// @param curAxesState Current axes state (axes position and origin status)
    /// @param axesParams Axes parameters
    /// @return true if successful    
    virtual bool actuatorToPt(const AxesValues<AxisStepsDataType>& targetActuator,
                              AxesValues<AxisPosDataType>& outPt,
                              const AxesState &curAxesState,
                              const AxesParams& axesParams) const override final
    {
        // Perform conversion
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            // TODO - decide if this origin steps needed
            // double ptVal = targetActuator.getVal(axisIdx) - axesParams.gethomeOffsetSteps(axisIdx);
            double ptVal = targetActuator.getVal(axisIdx);
            ptVal = ptVal / axesParams.getStepsPerUnit(axisIdx);
            outPt.setVal(axisIdx, ptVal);
#ifdef DEBUG_KINEMATICS_XYZ
            LOG_I(MODULE_PREFIX, "actuatorToPt axis%d %d -> %.2f (perunit %.2f)",
                            axisIdx, 
                            targetActuator.getVal(axisIdx),
                            ptVal, 
                            axesParams.getStepsPerUnit(axisIdx));
#endif
        }
        return true;        
    }

private:
    static constexpr const char *MODULE_PREFIX = "KinematicsXYZ";
};
