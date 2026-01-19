/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlockManager
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionBlockManager.h"
#include "RaftKinematicsSystem.h"
#include "KinematicsSingleArmSCARA.h"

#if USE_SINGLE_SPLIT_BLOCK
#include "MotionPipeline.h"  // For getLastAddedBlock() cast
#endif

// #define DEBUG_RAMPED_BLOCK
// #define DEBUG_COORD_UPDATES
// #define DEBUG_BLOCK_SPLITTER
// #define DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
// #define DEBUG_MOTION_BLOCK_MANAGER_TIMINGS_DETAILED

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
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t startTimeUs = micros();
#endif

    _blockMotionArgs = args;
    _numBlocks = numBlocks;
    _nextBlockIdx = 0;
    _finalTargetPos = args.getAxesPosConst();
    
    // Enable actuator space interpolation for split blocks (avoids repeated IK)
    _useActuatorInterpolation = false;
    if (_numBlocks > 1 && _pRaftKinematics)
    {
        // Calculate start actuator coordinates (from current position)
        bool startValid = _pRaftKinematics->ptToActuator(_axesState.getUnitsFromOrigin(), 
                _startActuatorCoords, _axesState, _axesParams, false);
        
        // Calculate end actuator coordinates (final target)
        bool endValid = _pRaftKinematics->ptToActuator(_finalTargetPos, 
                _endActuatorCoords, _axesState, _axesParams, true);
        
        _useActuatorInterpolation = (startValid && endValid);
        
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
        if (_useActuatorInterpolation)
        {
            LOG_I(MODULE_PREFIX, "Using actuator interpolation for %d blocks", _numBlocks);
        }
#endif
    }
    _blockMotionVector = (_finalTargetPos - _axesState.getUnitsFromOrigin()) / double(numBlocks);

    // Geometric bounds checking optimization for kinematics with alternate IK solutions
    // Only validate endpoint if it's near workspace boundaries, avoiding expensive full-path validation
    if (numBlocks > 1 && _pRaftKinematics && _pRaftKinematics->supportsAlternateSolutions())
    {
        // Get the block distance parameter from axes params (typically 10mm)
        double blockDistMM = _axesParams.getMaxBlockDistMM();
        
        // Calculate distances from origin for start and end points
        AxisPosDataType startX = _axesState.getUnitsFromOrigin(0);
        AxisPosDataType startY = _axesState.getUnitsFromOrigin(1);
        double startDist = sqrt(startX*startX + startY*startY);
        
        AxisPosDataType endX = _finalTargetPos.getVal(0);
        AxisPosDataType endY = _finalTargetPos.getVal(1);
        double endDist = sqrt(endX*endX + endY*endY);
        
        // Get the max radius from kinematics (SCARA workspace outer limit)
        // Safe cast: supportsAlternateSolutions() is only true for SCARA
        double maxRadiusMM = static_cast<const KinematicsSingleArmSCARA*>(_pRaftKinematics)->getMaxRadiusMM();
        
        // Check if the move is entirely within safe workspace (more than blockDistMM from boundaries)
        bool needsValidation = (startDist > maxRadiusMM - blockDistMM) || 
                               (endDist > maxRadiusMM - blockDistMM) ||
                               (startDist < blockDistMM) || 
                               (endDist < blockDistMM);
        
        if (needsValidation)
        {
            // Near boundaries - validate only the endpoint to choose IK solution
            _pRaftKinematics->setPreferAlternateSolution(false);
            AxesValues<AxisStepsDataType> unusedSteps;
            bool primaryEndpointValid = _pRaftKinematics->ptToActuator(_finalTargetPos, unusedSteps, _axesState, _axesParams, true);
            
            if (!primaryEndpointValid)
            {
                // Try alternate solution
                LOG_I(MODULE_PREFIX, "Primary IK solution invalid at endpoint, testing alternate...");
                _pRaftKinematics->setPreferAlternateSolution(true);
                bool alternateEndpointValid = _pRaftKinematics->ptToActuator(_finalTargetPos, unusedSteps, _axesState, _axesParams, true);
                
                if (alternateEndpointValid)
                {
                    LOG_I(MODULE_PREFIX, "Alternate IK solution valid");
                }
                else
                {
                    LOG_W(MODULE_PREFIX, "Both IK solutions invalid at endpoint, disabling block splitting");
                    _numBlocks = 1;
                    _pRaftKinematics->setPreferAlternateSolution(false);
                }
            }
        }
        else
        {
            // Move is entirely within safe workspace - no validation needed
            _pRaftKinematics->setPreferAlternateSolution(false);
        }
    }
    
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "addRampedBlock TIMING: total=%lluus numBlocks=%d", totalTimeUs, numBlocks);
#endif

#ifdef DEBUG_RAMPED_BLOCK
    LOG_I(MODULE_PREFIX, "addRampedBlock curUnits %s curSteps %s targetPosUnits %s numBlocks %d blockMotionVector %s)",
                _axesState.getUnitsFromOrigin().getDebugJSON("unFrOr").c_str(),
                _axesState.getStepsFromOrigin().getDebugJSON("stFrOr").c_str(),
                _finalTargetPos.getDebugJSON("targ").c_str(),
                _numBlocks, 
                _blockMotionVector.getDebugJSON("vec").c_str());
#endif

    // Phase 2+: Route to new single split-block path when feature enabled
    // This allows parallel development and testing of new architecture
#if USE_SINGLE_SPLIT_BLOCK
    // Note: Feature currently disabled for testing - will be enabled in Phase 5
    // When enabled, uses single MotionBlock with split metadata instead of multiple blocks
#endif

    return true;
}

#if USE_SINGLE_SPLIT_BLOCK
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add ramped block as single split-block (Phase 2+)
/// Creates one MotionBlock with split metadata instead of multiple separate blocks
/// Reduces memory allocations, pipeline operations, and planner overhead
/// @param args Motion arguments for the entire segment
/// @param numBlocks Number of sub-blocks for geometric waypoints  
/// @param motionPipeline Motion pipeline to add the block to
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftRetCode MotionBlockManager::addRampedBlockSingle(const MotionArgs& args, uint32_t numBlocks,
                                                     MotionPipelineIF& motionPipeline, String* respMsg)
{
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t startTimeUs = micros();
#endif

    // Get kinematics
    if (!_pRaftKinematics)
    {
        if (respMsg)
            *respMsg = "No kinematics geometry configured";
        LOG_W(MODULE_PREFIX, "addRampedBlockSingle no geometry set");
        return RAFT_INVALID_OBJECT;
    }

    // Calculate start actuator coordinates (IK #1)
    AxesValues<AxisStepsDataType> startActuatorCoords;
    bool startValid = _pRaftKinematics->ptToActuator(_axesState.getUnitsFromOrigin(), 
            startActuatorCoords, _axesState, _axesParams, false);
    
    if (!startValid)
    {
        if (respMsg)
            *respMsg = "START_OUT_OF_BOUNDS";
        LOG_W(MODULE_PREFIX, "addRampedBlockSingle start position out of bounds");
        return RAFT_INVALID_DATA;
    }

    // Calculate end actuator coordinates (IK #2)
    AxesValues<AxisStepsDataType> endActuatorCoords;
    bool endValid = _pRaftKinematics->ptToActuator(args.getAxesPosConst(), 
            endActuatorCoords, _axesState, _axesParams, true);
    
    if (!endValid)
    {
        if (respMsg)
            *respMsg = "END_OUT_OF_BOUNDS";
        LOG_W(MODULE_PREFIX, "addRampedBlockSingle end position out of bounds");
        return RAFT_INVALID_DATA;
    }

    // Create single MotionArgs for the whole segment
    // Important: This represents the entire motion, not individual waypoints
    MotionArgs singleBlockArgs = args;
    singleBlockArgs.setMoreMovesComing(false); // This is the final block for this motion
    
    // Enable motors once for the entire sequence
    _motorEnabler.enableMotors(true, false);

#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t plannerStartUs = micros();
#endif

    // Call planner ONCE to create the single block
    // The block will have one acceleration profile for the entire segment
    RaftRetCode rc = _motionPlanner.moveToRamped(
        singleBlockArgs, endActuatorCoords, _axesState, _axesParams, 
        motionPipeline, respMsg, false); // Don't defer recalculation
        
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t plannerTimeUs = micros() - plannerStartUs;
#endif

    if (rc == RAFT_OK)
    if (rc == RAFT_OK)
    {
        // Configure the block with split metadata
        // This tells the block it represents multiple geometric waypoints
        // Each waypoint is an interpolation point in actuator space
        
        // Cast to concrete MotionPipeline type to access getLastAddedBlock()
        // This is safe because the interface is always implemented by MotionPipeline
        MotionPipeline* pPipeline = static_cast<MotionPipeline*>(&motionPipeline);
        MotionBlock* block = pPipeline->getLastAddedBlock();
        
        if (block)
        {
            block->configureSplitBlock(numBlocks, startActuatorCoords, endActuatorCoords);
            
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
            LOG_I(MODULE_PREFIX, "addRampedBlockSingle: Configured split-block with %d waypoints", numBlocks);
#endif
        }
        else
        {
            LOG_W(MODULE_PREFIX, "addRampedBlockSingle: Failed to get last added block for split configuration");
            rc = RAFT_INVALID_OBJECT;
        }
    }

#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "addRampedBlockSingle TIMING: total=%lluus planner=%lluus numWaypoints=%d",
          totalTimeUs, plannerTimeUs, numBlocks);
#endif

    return rc;
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Pump the block splitter - should be called regularly 
/// @param motionPipeline Motion pipeline to add the block to
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
/// @note This is used to manage splitting of a single moveTo command into multiple blocks
RaftRetCode MotionBlockManager::pumpBlockSplitter(MotionPipelineIF& motionPipeline, String* respMsg)
{
#if USE_SINGLE_SPLIT_BLOCK
    // Phase 5: Use single split-block path when feature enabled
    // This creates one block with split metadata instead of multiple separate blocks
    if (_numBlocks > 0)
    {
        // Create single block representing all waypoints
        RaftRetCode rc = addRampedBlockSingle(_blockMotionArgs, _numBlocks, motionPipeline, respMsg);
        
        // Clear state regardless of result
        _numBlocks = 0;
        _nextBlockIdx = 0;
        
        return rc;
    }
    return RAFT_OK;
#else
    // Original multi-block path (Phase 1-4 compatibility)
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t startTimeUs = micros();
    uint32_t blocksProcessed = 0;
    uint64_t totalPlannerTimeUs = 0;
#endif

    // Check if we can add anything to the pipeline
    while (motionPipeline.canAccept())
    {
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
        uint64_t loopStartUs = micros();
#endif
        // Check if any blocks remain to be expanded out
        if (_numBlocks <= 0)
        {
            // Reset solution preference when move sequence completes
            if (_pRaftKinematics && _pRaftKinematics->supportsAlternateSolutions())
            {
                _pRaftKinematics->setPreferAlternateSolution(false);
            }
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
            if (blocksProcessed > 0)
            {
                uint64_t totalTimeUs = micros() - startTimeUs;
                LOG_I(MODULE_PREFIX, "pumpBlockSplitter TIMING: total=%lluus blocksProcessed=%d avgPlannerTime=%lluus",
                    totalTimeUs, blocksProcessed, blocksProcessed > 0 ? totalPlannerTimeUs / blocksProcessed : 0);
            }
#endif
            return RAFT_OK;
        }

        // Add to pipeline any blocks that are waiting to be expanded out
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
        uint64_t posCalcStartUs = micros();
#endif
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
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
        uint64_t posCalcTimeUs = micros() - posCalcStartUs;
#endif

#ifdef DEBUG_BLOCK_SPLITTER
        LOG_I(MODULE_PREFIX, "pumpBlockSplitter last %s + delta %s => dest %s (%s) nextBlockIdx %d, numBlocks %d", 
                    _axesState.getUnitsFromOrigin().getDebugJSON("unFrOr").c_str(),
                    _blockMotionVector.getDebugJSON("vec").c_str(),
                    nextBlockDest.getDebugJSON("dst").c_str(),
                    _blockMotionArgs.getAxesPos().getDebugJSON("cur").c_str(), 
                    _nextBlockIdx,
                    _numBlocks);
#endif

        // Add to planner (always defer recalculation - we do it in batch after loop)
        bool isLastBlock = (_numBlocks == 0);
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
        uint64_t plannerStartUs = micros();
#endif
        RaftRetCode rc = addToPlanner(_blockMotionArgs, motionPipeline, respMsg, true);
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
        uint64_t plannerTimeUs = micros() - plannerStartUs;
        totalPlannerTimeUs += plannerTimeUs;
        // Disabled per-block logging to reduce overhead (~23ms for 27 blocks)
        // uint64_t loopTimeUs = micros() - loopStartUs;
        // if (blocksProcessed < 3 || blocksProcessed == 26) {
        //     LOG_I(MODULE_PREFIX, "pumpBlock[%d]: loop=%lluus posCalc=%lluus planner=%lluus", 
        //           blocksProcessed, loopTimeUs, posCalcTimeUs, plannerTimeUs);
        // }
#endif

        if (rc != RAFT_OK)
        {
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
            LOG_I(MODULE_PREFIX, "pumpBlockSplitter TIMING: blocksProcessed=%d avgPlannerTime=%lluus FAILED",
                  blocksProcessed, blocksProcessed > 0 ? totalPlannerTimeUs / blocksProcessed : 0);
#endif
            return rc;
        }

        // Bump processed count
        blocksProcessed++;

        // Enable motors (only once for first block)
        if (blocksProcessed == 1)
            _motorEnabler.enableMotors(true, false);

        // If this was the last block, recalculate pipeline once
        if (isLastBlock)
        {
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
            uint64_t recalcStartUs = micros();
#endif
            _motionPlanner.recalculatePipelinePublic(motionPipeline, _axesParams, blocksProcessed);
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
            uint64_t recalcTimeUs = micros() - recalcStartUs;
            LOG_I(MODULE_PREFIX, "pumpBlockSplitter batch recalculation: %lluus for %d blocks", recalcTimeUs, blocksProcessed);
#endif
        }
    }
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "pumpBlockSplitter TIMING: total=%lluus blocksProcessed=%d avgPlannerTime=%lluus pipelineFull=true",
          totalTimeUs, blocksProcessed, blocksProcessed > 0 ? totalPlannerTimeUs / blocksProcessed : 0);
#endif
    return RAFT_OK;
#endif  // USE_SINGLE_SPLIT_BLOCK
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add to planner
/// @param args MotionArgs define the parameters for motion
/// @param motionPipeline Motion pipeline to add the block to
/// @param respMsg Optional pointer to string for error message
/// @param deferRecalc If true, defer pipeline recalculation (for batch mode)
/// @return RaftRetCode
/// @note The planner is responsible for computing suitable motion
///       and args may be modified by this function
RaftRetCode MotionBlockManager::addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline, String* respMsg, bool deferRecalc)
{
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t addStartUs = micros();
#endif
    // Get kinematics
    if (!_pRaftKinematics)
    {
        if (respMsg)
            *respMsg = "No kinematics geometry configured";
        LOG_W(MODULE_PREFIX, "addToPlanner no geometry set");
        return RAFT_INVALID_OBJECT;
    }

#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t ikStartUs = micros();
#endif
    // Convert the move to actuator coordinates
    AxesValues<AxisStepsDataType> actuatorCoords;
    bool coordsValid = false;
    
    // Use interpolation for intermediate split blocks to avoid expensive IK
    if (_useActuatorInterpolation && _nextBlockIdx > 0 && _numBlocks > 0)
    {
        // Linearly interpolate actuator coordinates
        float t = (float)_nextBlockIdx / (float)(_nextBlockIdx + _numBlocks);
        for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        {
            actuatorCoords.setVal(axisIdx, 
                _startActuatorCoords.getVal(axisIdx) + 
                (int32_t)((float)(_endActuatorCoords.getVal(axisIdx) - _startActuatorCoords.getVal(axisIdx)) * t));
        }
        coordsValid = true;
    }
    else
    {
        // Full IK calculation for first/last blocks or when interpolation disabled
        coordsValid = _pRaftKinematics->ptToActuator(args.getAxesPosConst(), 
                actuatorCoords, 
                _axesState, 
                _axesParams,
                args.constrainToBounds());
    }
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t ikTimeUs = micros() - ikStartUs;
#endif
    
    // Check if coordinates are valid
    if (!coordsValid)
    {
        if (respMsg)
            *respMsg = "OUT_OF_BOUNDS";
        LOG_W(MODULE_PREFIX, "addToPlanner position out of bounds x=%.2f y=%.2f", 
                args.getAxesPosConst().getVal(0), args.getAxesPosConst().getVal(1));
        return RAFT_INVALID_DATA;
    }

#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t plannerCallStartUs = micros();
#endif
    // Plan the move
    RaftRetCode rc = _motionPlanner.moveToRamped(args, actuatorCoords, 
                        _axesState, _axesParams, motionPipeline, respMsg, deferRecalc);
#ifdef DEBUG_MOTION_BLOCK_MANAGER_TIMINGS
    uint64_t plannerCallTimeUs = micros() - plannerCallStartUs;
    // Disabled per-block logging to reduce overhead (~23ms for 27 blocks)
    // uint64_t totalAddTimeUs = micros() - addStartUs;
    // static int addLogCtr = 0;
    // if (addLogCtr < 3 || addLogCtr == 26) {
    //     LOG_I(MODULE_PREFIX, "addToPlanner[%d]: total=%lluus ik=%lluus planner=%lluus", 
    //           addLogCtr, totalAddTimeUs, ikTimeUs, plannerCallTimeUs);
    // }
    // addLogCtr++;
    // // Reset counter when sequence ends
    // if (_numBlocks == 0) addLogCtr = 0;
#endif
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

void MotionBlockManager::setCurPositionAsOrigin()
{
    _axesState.setOrigin();
}
