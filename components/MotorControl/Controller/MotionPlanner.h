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
#include "RampGenConsts.h"

class MotionPlanner
{
public:
    MotionPlanner(const AxesParams& axesParams);

    /// @brief Setup
    /// @param stepGenPeriodUs Step generation period in microseconds
    void setup(uint32_t stepGenPeriodUs);

    /// @brief Add a non-ramped motion block (used for homing, etc)
    /// @param args MotionArgs define the parameters for motion
    /// @param axesState Current state of the axes including position and origin status
    /// @param axesParams Parameters for the axes
    /// @param motionPipeline Motion pipeline to add the block to
    /// @return AxesValues<AxisStepsDataType> containing the destination actuator coordinates
    /// @note This function is used for non-ramped motion such as homing where the actuator moves at a constant speed
    ///       args may be modified by this function
    AxesValues<AxisStepsDataType> moveToNonRamped(MotionArgs& args,
                    AxesState& axesState,
                    const AxesParams& axesParams, 
                    MotionPipelineIF& motionPipeline);

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

    /// @brief Debug show pipeline contents
    /// @param motionPipeline Motion pipeline to show
    /// @param minQLen Minimum queue length to show
    void debugShowPipeline(MotionPipelineIF& motionPipeline, unsigned int minQLen);

private:

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionPlanner";

    // Minimum planner speed mm/s
    float _minimumPlannerSpeedMMps = 0.0f;
    // Step generation timer period ns
    uint32_t _stepGenPeriodNs = RAMP_GEN_PERIOD_US_DEFAULT * 1000;

    // Axes parameters
    const AxesParams& _axesParams;

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
