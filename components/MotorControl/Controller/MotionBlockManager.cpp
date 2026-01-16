/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlockManager
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionBlockManager.h"
#include "RaftKinematicsSystem.h"

// #define DEBUG_RAMPED_BLOCK
// #define DEBUG_COORD_UPDATES
// #define DEBUG_BLOCK_SPLITTER

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
/// @param motorEnabler object to enable/disable motors
/// @param axesParams parameters for the axes
MotionBlockManager::MotionBlockManager(MotorEnabler& motorEnabler, AxesParams& axesParams)
                :   _motionPlanner(axesParams),
                    _motorEnabler(motorEnabler), 
                    _axesParams(axesParams)
{
    clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
MotionBlockManager::~MotionBlockManager()
{
    if (_pRaftKinematics)
        delete _pRaftKinematics;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Clear
void MotionBlockManager::clear()
{
    _numBlocks = 0;
    _nextBlockIdx = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup
/// @param stepGenPeriodUs Period of the step generator in microseconds
/// @param motionConfig JSON configuration
void MotionBlockManager::setup(uint32_t stepGenPeriodUs, const RaftJsonIF& motionConfig)
{
    // Motion Pipeline and Planner
    _motionPlanner.setup(stepGenPeriodUs);

    // Set geometry
    if (_pRaftKinematics)
        delete _pRaftKinematics;
    _pRaftKinematics = RaftKinematicsSystem::createKinematics(motionConfig);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add a non-ramped (constant speed) block to the pipeline (for homing etc)
/// @param args Motion arguments
/// @param motionPipeline Motion pipeline to add the block to
/// @return true if successful
/// @note args may be modified by this function
bool MotionBlockManager::addNonRampedBlock(MotionArgs& args, MotionPipelineIF& motionPipeline)
{
    AxesValues<AxisStepsDataType> curPosStepsFromOrigin = _motionPlanner.moveToNonRamped(args, 
                    _axesState, 
                    _axesParams, 
                    motionPipeline);

    // Since this was a non-ramped move units from home is now invalid
    _axesState.setStepsFromOriginAndInvalidateUnits(curPosStepsFromOrigin);

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add a ramped block (which may be split up) to the pipeline
/// @param args Motion arguments including target position, speed, etc
/// @param numBlocks Number of blocks to split the move into
/// @return true if successful
bool MotionBlockManager::addRampedBlock(const MotionArgs& args, uint32_t numBlocks)
{
    _blockMotionArgs = args;
    _numBlocks = numBlocks;
    _nextBlockIdx = 0;
    _finalTargetPos = args.getAxesPosConst();
    _blockMotionVector = (_finalTargetPos - _axesState.getUnitsFromOrigin()) / double(numBlocks);

    // Smart solution selection for kinematics that support alternate IK solutions
    // If splitting into multiple blocks, validate the path with both IK solutions
    if (numBlocks > 1 && _pRaftKinematics && _pRaftKinematics->supportsAlternateSolutions())
    {
        // Test primary solution (normal behavior)
        _pRaftKinematics->setPreferAlternateSolution(false);
        bool primaryPathValid = _pRaftKinematics->validateLinearPath(
            _axesState.getUnitsFromOrigin(), 
            _finalTargetPos, 
            numBlocks, 
            _axesState, 
            _axesParams);

        if (!primaryPathValid)
        {
            // Primary path has invalid points, try alternate solution
            LOG_I(MODULE_PREFIX, "Primary IK solution creates invalid intermediate points, testing alternate...");
            
            _pRaftKinematics->setPreferAlternateSolution(true);
            bool alternatePathValid = _pRaftKinematics->validateLinearPath(
                _axesState.getUnitsFromOrigin(), 
                _finalTargetPos, 
                numBlocks, 
                _axesState, 
                _axesParams);

            if (alternatePathValid)
            {
                LOG_I(MODULE_PREFIX, "Alternate IK solution valid, using alternate configuration");
                // Keep alternate solution preference set
            }
            else
            {
                // Both solutions fail - disable splitting, move directly
                LOG_W(MODULE_PREFIX, "Both IK solutions have invalid points, disabling block splitting (nosplit)");
                _numBlocks = 1;
                _pRaftKinematics->setPreferAlternateSolution(false);  // Reset to normal
            }
        }
        else
        {
            // Primary path is valid, use it
            _pRaftKinematics->setPreferAlternateSolution(false);
        }
    }

#ifdef DEBUG_RAMPED_BLOCK
    LOG_I(MODULE_PREFIX, "addRampedBlock curUnits %s curSteps %s targetPosUnits %s numBlocks %d blockMotionVector %s)",
                _axesState.getUnitsFromOrigin().getDebugJSON("unFrOr").c_str(),
                _axesState.getStepsFromOrigin().getDebugJSON("stFrOr").c_str(),
                _finalTargetPos.getDebugJSON("targ").c_str(),
                _numBlocks, 
                _blockMotionVector.getDebugJSON("vec").c_str());
#endif

    return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Pump the block splitter - should be called regularly 
/// @param motionPipeline Motion pipeline to add the block to
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
/// @note This is used to manage splitting of a single moveTo command into multiple blocks
RaftRetCode MotionBlockManager::pumpBlockSplitter(MotionPipelineIF& motionPipeline, String* respMsg)
{
    // Check if we can add anything to the pipeline
    while (motionPipeline.canAccept())
    {
        // Check if any blocks remain to be expanded out
        if (_numBlocks <= 0)
        {
            // Reset solution preference when move sequence completes
            if (_pRaftKinematics && _pRaftKinematics->supportsAlternateSolutions())
            {
                _pRaftKinematics->setPreferAlternateSolution(false);
            }
            return RAFT_OK;
        }

        // Add to pipeline any blocks that are waiting to be expanded out
        AxesValues<AxisPosDataType> nextBlockDest = _axesState.getUnitsFromOrigin() + _blockMotionVector;

        // Bump position
        _nextBlockIdx++;

        // Check if done, use final target coords if so to ensure cumulative errors don't creep in
        if (_nextBlockIdx >= _numBlocks)
        {
            _numBlocks = 0;
            nextBlockDest = _finalTargetPos;
        }

        // Prepare add to planner
        _blockMotionArgs.setAxesPositions(nextBlockDest);
        _blockMotionArgs.setMoreMovesComing(_numBlocks != 0);

#ifdef DEBUG_BLOCK_SPLITTER
        LOG_I(MODULE_PREFIX, "pumpBlockSplitter last %s + delta %s => dest %s (%s) nextBlockIdx %d, numBlocks %d", 
                    _axesState.getUnitsFromOrigin().getDebugJSON("unFrOr").c_str(),
                    _blockMotionVector.getDebugJSON("vec").c_str(),
                    nextBlockDest.getDebugJSON("dst").c_str(),
                    _blockMotionArgs.getAxesPos().getDebugJSON("cur").c_str(), 
                    _nextBlockIdx,
                    _numBlocks);
#endif

        // Add to planner
        RaftRetCode rc = addToPlanner(_blockMotionArgs, motionPipeline, respMsg);
        if (rc != RAFT_OK)
            return rc;

        // Enable motors
        _motorEnabler.enableMotors(true, false);
    }
    return RAFT_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add to planner
/// @param args MotionArgs define the parameters for motion
/// @param motionPipeline Motion pipeline to add the block to
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
/// @note The planner is responsible for computing suitable motion
///       and args may be modified by this function
RaftRetCode MotionBlockManager::addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline, String* respMsg)
{
    // Get kinematics
    if (!_pRaftKinematics)
    {
        if (respMsg)
            *respMsg = "No kinematics geometry configured";
        LOG_W(MODULE_PREFIX, "addToPlanner no geometry set");
        return RAFT_INVALID_OBJECT;
    }

    // Convert the move to actuator coordinates
    AxesValues<AxisStepsDataType> actuatorCoords;
    _pRaftKinematics->ptToActuator(args.getAxesPosConst(), 
            actuatorCoords, 
            _axesState, 
            _axesParams,
            args.constrainToBounds());

    // Plan the move
    RaftRetCode rc = _motionPlanner.moveToRamped(args, actuatorCoords, 
                        _axesState, _axesParams, motionPipeline, respMsg);
#ifdef DEBUG_COORD_UPDATES
    LOG_I(MODULE_PREFIX, "addToPlanner rc %s pt %s actuator %s", 
            Raft::getRetCodeStr(rc),
            args.getAxesPosConst().getDebugJSON("cur").c_str(),
            actuatorCoords.toJSON().c_str());
#endif

    // Correct overflows if necessary
    if (rc == RAFT_OK)
    {
        // TODO check that this is already done in moveToRamped - it updates _lastCommandedAxisPos
        // // Update axisMotion
        // _lastCommandedAxisPos.curPosUnitsFromOrigin = args.getPointMM();

        // Correct overflows
        // TODO re-implement
        // if (_correctStepOverflowFn)
        // {
        //     _correctStepOverflowFn(_lastCommandedAxesPositions, _axesParams);
        // }            
#ifdef DEBUG_COORD_UPDATES
        LOG_I(MODULE_PREFIX, "addToPlanner updatedAxisPos %s",
            _axesState.getUnitsFromOrigin().getDebugJSON("unFrOr").c_str());
#endif
    }
    else
    {
        LOG_W(MODULE_PREFIX, "addToPlanner moveToRamped failed: %s", 
              respMsg && respMsg->length() > 0 ? respMsg->c_str() : Raft::getRetCodeStr(rc));
    }
    return rc;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set current position as origin
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::setCurPositionAsOrigin(uint32_t axisIdx)
{
    if (axisIdx >= AXIS_VALUES_MAX_AXES)
        return;

    // TODO 
    // _lastCommandedAxesPositions.curPosUnitsFromOrigin.setVal(axisIdx, 0);
    // _lastCommandedAxesPositions.setUnitsFromOriginValidity(true);
    // _lastCommandedAxesPositions.curPosStepsFromOrigin.setVal(axisIdx, 0);
// #ifdef DEBUG_COORD_UPDATES
//         LOG_I(MODULE_PREFIX, "setCurPosAsOrigin axisIdx %d curMM %0.2f steps %d", axisIdx,
//                     _lastCommandedAxesPositions.curPosUnitsFromOrigin.getVal(axisIdx),
//                     _lastCommandedAxesPositions.curPosStepsFromOrigin.getVal(axisIdx));
// #endif
}
