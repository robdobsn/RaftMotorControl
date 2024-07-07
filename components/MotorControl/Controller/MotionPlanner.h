/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPlanner
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "MotionPipeline.h"
#include "MotionArgs.h"
#include "AxesParams.h"
#include "AxesState.h"

class MotionPlanner
{
public:
    MotionPlanner();

    void setup(double maxJunctionDeviationMM, uint32_t );

    // Add a linear (no ramp) motion block (used for homing, etc)
    AxesValues<AxisStepsDataType> moveToLinear(const MotionArgs& args,
                    AxesState& axesState,
                    const AxesParams& axesParams, 
                    MotionPipelineIF& motionPipeline);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Add a ramped (variable acceleration) motion block
    /// @param args MotionArgs define the parameters for motion
    /// @param destActuatorCoords Destination actuator coordinates
    /// @param curAxesState Current state of the axes including position and origin status
    /// @param axesParams Parameters for the axes
    /// @param motionPipeline Motion pipeline to add the block to
    /// @return true if a block was added
    bool moveToRamped(const MotionArgs& args,
                    const AxesValues<AxisStepsDataType>& destActuatorCoords,
                    AxesState& curAxesState,
                    const AxesParams& axesParams,
                    MotionPipelineIF& motionPipeline);

    // Debug
    void debugShowPipeline(MotionPipelineIF& motionPipeline, unsigned int minQLen);

private:
    // Minimum planner speed mm/s
    float _minimumPlannerSpeedMMps = 0.0f;
    // Max junction deviation (mm)
    float _maxJunctionDeviationMM = 0.0f;
    // Step generation timer period ns
    uint32_t _stepGenPeriodNs = 0;

    // Structure to store details on last processed block
    struct MotionBlockSequentialData
    {
        AxesValues<AxisUnitVectorDataType> _unitVectors;
        float _maxParamSpeedMMps = 0.0f;
    };
    // Data on previously processed block
    bool _prevMotionBlockValid = false;
    MotionBlockSequentialData _prevMotionBlock;

    // Recalculate
    void recalculatePipeline(MotionPipelineIF& motionPipeline, const AxesParams& axesParams);
};
