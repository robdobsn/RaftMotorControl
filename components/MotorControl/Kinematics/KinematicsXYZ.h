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

    static RaftKinematics *create()
    {
        return new KinematicsXYZ();
    }

    virtual bool ptToActuator(AxesPosValues targetPt,
                              AxesParamVals<AxisStepsDataType> &outActuator,
                              const AxesPosition &curPos,
                              const AxesParams &axesParams,
                              bool allowOutOfBounds) const override final
    {
        // Check machine bounds and fix the value if required
        bool ptWasValid = axesParams.ptInBounds(targetPt, !allowOutOfBounds);

        // Perform conversion
        for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            // Axis val from home point
            float axisValFromHome = targetPt.getVal(axisIdx) - axesParams.getHomeOffsetVal(axisIdx);
            // Convert to steps and add offset to home in steps
            outActuator.setVal(axisIdx, round(axisValFromHome * axesParams.getStepsPerUnit(axisIdx) + axesParams.gethomeOffSteps(axisIdx)));

#ifdef DEBUG_KINEMATICS_XYZ
            LOG_I(MODULE_PREFIX, "ptToActuator axis%d %f%s-> %d (homeOffVal %f, homeOffSteps %d)",
                  axisIdx,
                  targetPt.getVal(axisIdx),
                  ptWasValid ? " " : "(OOB) ",
                  outActuator.getVal(axisIdx),
                  axesParams.getHomeOffsetVal(axisIdx),
                  axesParams.gethomeOffSteps(axisIdx));
#endif
        }
        return ptWasValid;
    }
    virtual bool actuatorToPt(const AxesParamVals<AxisStepsDataType> &targetActuator,
                              AxesPosValues &outPt,
                              const AxesPosition &curPos,
                              const AxesParams &axesParams) const override final
    {
        // Perform conversion
        for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            double ptVal = targetActuator.getVal(axisIdx) - axesParams.gethomeOffSteps(axisIdx);
            ptVal = ptVal / axesParams.getStepsPerUnit(axisIdx) + axesParams.getHomeOffsetVal(axisIdx);
            outPt.setVal(axisIdx, ptVal);
#ifdef DEBUG_KINEMATICS_XYZ
            LOG_I(MODULE_PREFIX, "actuatorToPt axis%d %d -> %f (perunit %f, homeOffSteps %d, homeOffVal %f)",
                            axisIdx, 
                            targetActuator.getVal(axisIdx),
                            ptVal, 
                            axesParams.getStepsPerUnit(axisIdx), 
                            axesParams.gethomeOffSteps(axisIdx),
                            axesParams.getHomeOffsetVal(axisIdx));
#endif
        }
        return true;        
    }

private:
    static constexpr const char *MODULE_PREFIX = "KinematicsXYZ";
};
