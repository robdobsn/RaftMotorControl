/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisGeomBase
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxisValues.h"
#include "AxesPosValues.h"
#include "AxesParamVals.h"

class AxesPosition;
class AxesParams;

class AxisGeomBase
{
public:
    virtual ~AxisGeomBase()
    {
    }

    // Coord transform from real-world coords to actuator
    virtual bool ptToActuator(AxesPosValues targetPt, 
                AxesParamVals<AxisStepsDataType> &outActuator, 
                const AxesPosition &curPos, 
                const AxesParams &axesParams, 
                bool allowOutOfBounds) const = 0;

    // Coord transform from actuator to real-world coords
    virtual bool actuatorToPt(const AxesParamVals<AxisStepsDataType> &targetActuator, 
                AxesPosValues &outPt, 
                const AxesPosition &curPos, 
                const AxesParams &axesParams) const = 0;

    // Correct step overflow (necessary in continuous rotation bots)
    virtual void correctStepOverflow(AxesPosition &curPos, 
                const AxesParams &axesParams) const
    {
    }

    // Convert coordinates (used for coordinate systems like Theta-Rho which are position dependent)
    // This doesn't convert coords - just checks for things like wrap around in circular coordinate systems
    // Note that values are modified in-place
    virtual void preProcessCoords(AxesPosValues& axisPositions, const AxesParams& axesParams) const
    {
    }

// typedef bool (*ptToActuatorFnType)(AxisFloats &targetPt, AxisFloats &outActuator, AxesPosition &curPos, AxesParams &axesParams, bool allowOutOfBounds);
// typedef void (*actuatorToPtFnType)(AxisInt32s &targetActuator, AxisFloats &outPt, AxesPosition &curPos, AxesParams &axesParams);
// typedef void (*correctStepOverflowFnType)(AxesPosition &curPos, AxesParams &axesParams);
// typedef void (*convertCoordsFnType)(RobotCommandArgs& cmdArgs, AxesParams &axesParams);
// typedef void (*setRobotAttributesFnType)(AxesParams& axesParams, String& robotAttributes);


};
