/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPlanner
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesPosition.h"
#include "MotionPipeline.h"
#include "MotionArgs.h"

class MotionPlanner
{
public:
    MotionPlanner();

    void setup(double junctionDeviation, uint32_t );

    // Add a linear (no ramp) motion block (used for homing, etc)
    AxesParamVals<AxisStepsDataType> moveToLinear(const MotionArgs &args,
                      AxesParamVals<AxisStepsDataType> curAxesStepsFromHome,
                      const AxesParams &axesParams, 
                      MotionPipeline &motionPipeline);

    // Add a regular ramped (variable acceleration) motion block
    bool moveToRamped(const MotionArgs &args,
                      const AxesParamVals<AxisStepsDataType> &destActuatorCoords,
                      AxesPosition &curAxisPositions,
                      const AxesParams &axesParams,
                      MotionPipeline &motionPipeline);

    // Debug
    void debugShowPipeline(MotionPipeline &motionPipeline, unsigned int minQLen);

private:
    // Minimum planner speed mm/s
    float _minimumPlannerSpeedMMps = 0.0f;
    // Junction deviation
    float _junctionDeviation = 0.0f;
    // Step generation timer period ns
    uint32_t _stepGenPeriodNs = 0;

    // Structure to store details on last processed block
    struct MotionBlockSequentialData
    {
        AxesParamVals<AxisUnitVectorDataType> _unitVectors;
        float _maxParamSpeedMMps = 0.0f;
    };
    // Data on previously processed block
    bool _prevMotionBlockValid = false;
    MotionBlockSequentialData _prevMotionBlock;

    // Recalculate
    void recalculatePipeline(MotionPipeline &motionPipeline, const AxesParams &axesParams);
};
