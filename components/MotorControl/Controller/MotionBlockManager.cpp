/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlockManager
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionBlockManager.h"
#include "RaftKinematicsSystem.h"

#define DEBUG_RAMPED_BLOCK
#define DEBUG_COORD_UPDATES
#define DEBUG_BLOCK_SPLITTER

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionBlockManager::MotionBlockManager(MotorEnabler& motorEnabler, AxesParams& axesParams)
                :   _motorEnabler(motorEnabler), 
                    _axesParams(axesParams)
{
    clear();
}

MotionBlockManager::~MotionBlockManager()
{
    if (_pRaftKinematics)
        delete _pRaftKinematics;
}

void MotionBlockManager::clear()
{
    _numBlocks = 0;
    _nextBlockIdx = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::setup(uint32_t stepGenPeriodUs, const RaftJsonIF& motionConfig)
{
    // Motion Pipeline and Planner
    _motionPlanner.setup(_axesParams.getMaxJunctionDeviationMM(), stepGenPeriodUs);

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
bool MotionBlockManager::addNonRampedBlock(const MotionArgs& args, MotionPipelineIF& motionPipeline)
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
/// @param args Motion arguments
/// @param targetPosition Target position
/// @param numBlocks Number of blocks to split the move into
/// @return true if successful
bool MotionBlockManager::addRampedBlock(const MotionArgs& args, 
                const AxesValues<AxisPosDataType>& targetPosition, 
                uint32_t numBlocks)
{
    _blockMotionArgs = args;
    _targetPosition = targetPosition;
    _numBlocks = numBlocks;
    _nextBlockIdx = 0;
    _blockMotionVector = (_targetPosition - _axesState.getUnitsFromOrigin()) / double(numBlocks);

#ifdef DEBUG_RAMPED_BLOCK
    LOG_I(MODULE_PREFIX, "moveTo curUnits %s curSteps %s newUnits %s numBlocks %d blockMotionVector %s)",
                _axesState.getUnitsFromOrigin().getDebugStr().c_str(),
                _axesState.getStepsFromOrigin().getDebugStr().c_str(),
                _targetPosition.getDebugStr().c_str(),
                _numBlocks, 
                _blockMotionVector.getDebugStr().c_str());
#endif

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// pumpBlockSplitter - should be called regularly 
// A single moveTo command can be split into blocks - this function checks if such
// splitting is in progress and adds the split-up motion blocks accordingly
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::pumpBlockSplitter(MotionPipelineIF& motionPipeline)
{
    // Check if we can add anything to the pipeline
    while (motionPipeline.canAccept())
    {
        // Check if any blocks remain to be expanded out
        if (_numBlocks <= 0)
            return;

        // Add to pipeline any blocks that are waiting to be expanded out
        AxesValues<AxisPosDataType> nextBlockDest = _axesState.getUnitsFromOrigin() + _blockMotionVector;

        // Bump position
        _nextBlockIdx++;

        // Check if done, use end point coords if so
        if (_nextBlockIdx >= _numBlocks)
        {
            _numBlocks = 0;
            nextBlockDest = _targetPosition;
        }

        // Prepare add to planner
        _blockMotionArgs.setAxesPositions(nextBlockDest);
        _blockMotionArgs.setMoreMovesComing(_numBlocks != 0);

#ifdef DEBUG_BLOCK_SPLITTER
        LOG_I(MODULE_PREFIX, "pumpBlockSplitter last %s + delta %s => dest %s (%s) nextBlockIdx %d, numBlocks %d", 
                    _axesState.getUnitsFromOrigin().getDebugStr().c_str(),
                    _blockMotionVector.getDebugStr().c_str(),
                    nextBlockDest.getDebugStr().c_str(),
                    _blockMotionArgs.getTargetPos().getDebugStr().c_str(), 
                    _nextBlockIdx,
                    _numBlocks);
#endif

        // Add to planner
        addToPlanner(_blockMotionArgs, motionPipeline);

        // Enable motors
        _motorEnabler.enableMotors(true, false);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add a movement to the pipeline using the planner which computes suitable motion
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionBlockManager::addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline)
{
    // Get kinematics
    if (!_pRaftKinematics)
    {
        LOG_W(MODULE_PREFIX, "addToPlanner no geometry set");
        return false;
    }

    // Convert the move to actuator coordinates
    AxesValues<AxisStepsDataType> actuatorCoords;
    _pRaftKinematics->ptToActuator(args.getTargetPos(), 
            actuatorCoords, 
            _axesState, 
            _axesParams);

    // Plan the move
    bool moveOk = _motionPlanner.moveToRamped(args, actuatorCoords, 
                        _axesState, _axesParams, motionPipeline);
#ifdef DEBUG_COORD_UPDATES
    LOG_I(MODULE_PREFIX, "addToPlanner moveOk %d pt %s actuator %s", 
            moveOk,
            args.getTargetPos().getDebugStr().c_str(),
            actuatorCoords.toJSON().c_str());
#endif

    // Correct overflows if necessary
    if (moveOk)
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
            _axesState.getUnitsFromOrigin().getDebugStr().c_str());
#endif
    }
    else
    {
        LOG_W(MODULE_PREFIX, "addToPlanner moveToRamped failed");
    }
    return moveOk;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Convert actuator coords to real-world
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::coordsActuatorToRealWorld(const AxesValues<AxisStepsDataType> &targetActuator, 
            AxesValues<AxisPosDataType> &outPt) const
{
    // Get geometry
    if (!_pRaftKinematics)
    {
        LOG_W(MODULE_PREFIX, "coordsActuatorToRealWorld no geometry set");
        return;
    }

    _pRaftKinematics->actuatorToPt(targetActuator, outPt, _axesState, _axesParams);
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
