/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftKinematics
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxisValues.h"
#include "AxesPosValues.h"
#include "AxesParamVals.h"

class AxesPosition;
class AxesParams;

class RaftKinematics
{
public:
    virtual ~RaftKinematics()
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
};
