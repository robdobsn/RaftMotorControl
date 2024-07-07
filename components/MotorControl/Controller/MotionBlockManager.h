/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlockManager
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesParams.h"
#include "MotionArgs.h"
#include "MotorEnabler.h"
#include "MotionPlanner.h"
#include "RaftKinematics.h"

class MotionPipelineIF;

class MotionBlockManager
{
public:
    // Constructor / Destructor
    MotionBlockManager(MotorEnabler& motorEnabler, AxesParams& axesParams);
    virtual ~MotionBlockManager();

    // Clear
    void clear();

    // Setup
    void setup(uint32_t stepGenPeriodUs, const RaftJsonIF& motionConfig);

    // pumpBlockSplitter - should be called regularly 
    // A single moveTo command can be split into blocks - this function checks if such
    // splitting is in progress and adds the split-up motion blocks accordingly
    void pumpBlockSplitter(MotionPipelineIF& motionPipeline);

    // Check is busy
    bool isBusy() const
    {
        return _numBlocks != 0;
    }

    // Add non-ramped motion block
    bool addNonRampedBlock(const MotionArgs& args, MotionPipelineIF& motionPipeline);

    // Add rampled block (which may be split up)
    bool addRampedBlock(const MotionArgs& args, 
                const AxesValues<AxisPosDataType>& targetPosition, 
                uint32_t numBlocks);

    // Get current state of axes
    const AxesState& getAxesState() const
    {
        return _axesState;
    }

    // Check last commanded position is valid
    bool isAxesStateValid() const
    {
        return _axesState.isValid();
    }

    // Convert actuator coords to real-world coords
    void coordsActuatorToRealWorld(const AxesValues<AxisStepsDataType> &targetActuator, 
                AxesValues<AxisPosDataType> &outPt) const;

    // Convert coordinates (used for coordinate systems like Theta-Rho which are position dependent)
    // This doesn't convert coords - just checks for things like wrap around in circular coordinate systems
    // Note that values are modified in-place
    void preProcessCoords(AxesValues<AxisPosAndValidDataType>& axesPositions, const AxesParams& axesParams) const
    {
        // Get kinematics
        if (!_pRaftKinematics)
        {
            LOG_W(MODULE_PREFIX, "preProcessCoords no kinematics set");
            return;
        }    
        _pRaftKinematics->preProcessCoords(axesPositions, axesParams);
    }

    // Set current position as home
    void setCurPositionAsOrigin(uint32_t axisIdx);

    // Homing needed before any move
    bool isHomingNeededBeforeMove() const
    {
        return _homingNeededBeforeAnyMove;
    }
    
private:
    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionBlockManager";

    // Args for motion
    MotionArgs _blockMotionArgs;

    // State of axes (including current position and origin status)
    AxesState _axesState;

    // Target position
    AxesValues<AxisPosDataType> _targetPosition;

    // Block motion as a vector
    AxesValues<AxisPosDataType> _blockMotionVector;

    // Num blocks to split over
    uint32_t _numBlocks = 0;

    // Next block to return
    uint32_t _nextBlockIdx = 0;

    // Planner used to plan the pipeline of motion
    MotionPlanner _motionPlanner;

    // Motor enabler
    MotorEnabler& _motorEnabler;

    // Kinematics
    RaftKinematics* _pRaftKinematics = nullptr;

    // Axes parameters
    AxesParams& _axesParams;

    // // Last commanded axes positions
    // AxesState _lastCommandedAxesPositions;

    // Homing is needed before any movement
    bool _homingNeededBeforeAnyMove = false;

    // Helpers
    bool addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline);
};
