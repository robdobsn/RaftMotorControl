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
    /// @brief Constructor
    MotionBlockManager(MotorEnabler& motorEnabler, AxesParams& axesParams);

    /// @brief Destructor
    virtual ~MotionBlockManager();

    /// @brief Clear all blocks
    void clear();

    /// @brief Setup
    /// @param stepGenPeriodUs Period of the step generator in microseconds
    /// @param motionConfig JSON configuration
    void setup(uint32_t stepGenPeriodUs, const RaftJsonIF& motionConfig);

    /// @brief Check if the motion block manager is busy
    /// @return true if busy
    bool isBusy() const
    {
        return _numBlocks != 0;
    }

    /// @brief Add non-ramped motion block (used for homing, etc)
    /// @param args MotionArgs define the parameters for motion including target position, speed, etc
    /// @param motionPipeline Motion pipeline to add the block to
    /// @return true if the block was added
    /// @note args may be modified by this function
    bool addNonRampedBlock(MotionArgs& args, MotionPipelineIF& motionPipeline);

    /// @brief Add a ramped motion block (which may be split up)
    /// @param args MotionArgs define the parameters for motion including target position, speed, etc
    /// @param numBlocks Number of blocks to split the motion into
    /// @return true if the block was added
    bool addRampedBlock(const MotionArgs& args, uint32_t numBlocks);

    /// @brief Get current state of axes
    /// @return AxesState
    const AxesState& getAxesState() const
    {
        return _axesState;
    }

    /// @brief Check last commanded position is valid
    /// @return true if valid
    bool isAxesStateValid() const
    {
        return _axesState.isValid();
    }

    /// @brief Convert actuator coords to real-world coords
    /// @param targetActuator Target actuator coordinates
    /// @param outPt Output real-world coordinates
    void actuatorToPt(const AxesValues<AxisStepsDataType> &targetActuator, 
                AxesValues<AxisPosDataType> &outPt) const
    {
        // Get kinematics
        if (!_pRaftKinematics)
        {
            LOG_W(MODULE_PREFIX, "actuatorToPt no kinematics set");
            return;
        }
        _pRaftKinematics->actuatorToPt(targetActuator, outPt, _axesState, _axesParams);
    }

    /// @brief Pre-process coordinates
    /// @param args Motion arguments (may be modified) including target position
    /// @return Distance to move in MM
    /// @note This is used to manage unspecified axes and for coordinate systems like Theta-Rho 
    ///       which are curret-position dependent
    AxisDistDataType preProcessCoords(MotionArgs& args) const
    {
        // Get kinematics
        if (!_pRaftKinematics)
        {
            LOG_W(MODULE_PREFIX, "preProcessCoords no kinematics set");
            return 0;
        }    
        return _pRaftKinematics->preProcessCoords(args, _axesState, _axesParams);
    }

    /// @brief Set current position as origin
    void setCurPositionAsOrigin();

    /// @brief Check if homing needed before any move
    /// @return true if homing is needed
    bool isHomingNeededBeforeMove() const
    {
        return _homingNeededBeforeAnyMove;
    }

    /// @brief Add to planner
    /// @param args MotionArgs define the parameters for motion
    /// @param motionPipeline Motion pipeline to add the block to
    /// @param respMsg Optional pointer to string for error message (default nullptr)
    /// @return RaftRetCode
    RaftRetCode addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline, String* respMsg = nullptr);

    /// @brief Pump block splitter
    /// @param motionPipeline Motion pipeline to add blocks to
    /// @param respMsg Optional pointer to string for error message (default nullptr)
    /// @return RaftRetCode
    RaftRetCode pumpBlockSplitter(MotionPipelineIF& motionPipeline, String* respMsg = nullptr);
    
private:
    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionBlockManager";

    // Args for motion
    MotionArgs _blockMotionArgs;

    // Final target position (this is a copy of the requested position because _blockMotionArgs may be modified)
    AxesValues<AxisPosDataType> _finalTargetPos;

    // State of axes (including current position and origin status)
    AxesState _axesState;

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

    /// @brief Add to planner
    /// @param args MotionArgs define the parameters for motion
    /// @param motionPipeline Motion pipeline to add the block to
    /// @return true if successful
    /// @note The planner is responsible for computing suitable motion
    bool addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline);
};
