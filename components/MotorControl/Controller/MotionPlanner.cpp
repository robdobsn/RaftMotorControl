/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPlanner
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionPlanner.h"
#include "MotionArgs.h"

// #define DEBUG_REQUESTED_VELOCITY
// #define DEBUG_ANGLE_CALCULATIONS
// #define DEBUG_TEST_DUMP
// #define DEBUG_MOTIONPLANNER_INFO
// #define DEBUG_MOTIONPLANNER_DETAILED_INFO
// #define DEBUG_MOTIONPLANNER_BEFORE
// #define DEBUG_MOTIONPLANNER_AFTER

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
MotionPlanner::MotionPlanner(const AxesParams& axesParams) : 
        _axesParams(axesParams)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup
/// @param stepGenPeriodUs Step generation period in microseconds
void MotionPlanner::setup(uint32_t stepGenPeriodUs)
{
    _stepGenPeriodNs = stepGenPeriodUs * 1000;
    LOG_I(MODULE_PREFIX, "setup maxJunctionDeviationMM %0.2f stepGenPeriodNs %d", _axesParams.getMaxJunctionDeviationMM(), _stepGenPeriodNs);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Move in non-ramped fashion (used for homing, etc)
/// @param args MotionArgs define the parameters for motion
/// @param axesState Current state of the axes including position and origin status
/// @param axesParams Parameters for the axes
/// @param motionPipeline Motion pipeline to add the block to
/// @return AxesValues<AxisStepsDataType> containing the destination actuator coordinates
/// @note This function is used for non-ramped motion such as homing where the actuator moves at a constant speed
///       args may be modified by this function
AxesValues<AxisStepsDataType> MotionPlanner::moveToNonRamped(MotionArgs &args,
                    AxesState& axesState,
                    const AxesParams &axesParams, 
                    MotionPipelineIF& motionPipeline)
{
    // Create a block for this movement which will end up on the pipeline
    MotionBlock block;
    block._entrySpeedMMps = 0;
    block._exitSpeedMMps = 0;
    block.setTimerPeriodNs(_stepGenPeriodNs);

    // Find if there are any steps
    bool hasSteps = false;
    AxisStepRateDataType lowestMaxStepRatePerSecForAnyAxis = 1e8;
    AxesValues<AxisStepsDataType> stepsToTarget;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        // Check if any steps to perform
        AxisStepsDataType steps = 0;
        if (args.getAxesSpecified().getVal(axisIdx))
        {
            if (args.isRelative())
                steps = args.getAxesPosConst().getVal(axisIdx);
            else
                steps = args.getAxesPosConst().getVal(axisIdx) - axesState.getUnitsFromOrigin(axisIdx);
        }

        // Set steps to target
        if (steps != 0)
        {
            hasSteps = true;
            if (lowestMaxStepRatePerSecForAnyAxis > axesParams.getMaxStepRatePerSec(axisIdx))
                lowestMaxStepRatePerSecForAnyAxis = axesParams.getMaxStepRatePerSec(axisIdx);
        }
        // Value (and direction)
        stepsToTarget.setVal(axisIdx, steps);
    }

    block.setStepsToTarget(stepsToTarget);

    // Check there are some actual steps
    if (!hasSteps)
        return axesState.getStepsFromOrigin();

    // Set unit vector
    block._unitVecAxisWithMaxDist = 1.0;

    // set end-stop check requirements
    block.setEndStopsToCheck(args.getEndstopCheck());

    // Set numbered command index if present
    block.setMotionTrackingIndex(args.getMotionTrackingIndex());

    // Compute the requestedVelocity
    AxisSpeedDataType requestedVelocity = lowestMaxStepRatePerSecForAnyAxis;
    if (args.isTargetSpeedValid() && (requestedVelocity > args.getTargetSpeed()))
        requestedVelocity = args.getTargetSpeed();

    // Feedrate percent (scale calculated velocities by this amount)
    double feedrateAsRatioToMax = args.getFeedrate() / 100.0;
    if (args.isFeedrateUnitsPerMin())
    {
        feedrateAsRatioToMax = 1.0;
        if (axesParams.masterAxisMaxSpeed() != 0)
            feedrateAsRatioToMax = args.getFeedrate() / 60.0 / axesParams.masterAxisMaxSpeed();
    }
    requestedVelocity *= feedrateAsRatioToMax;
    block._requestedSpeed = requestedVelocity;

    // Prepare for stepping
    if (block.prepareForStepping(axesParams, true))
    {
        // No more changes
        block._canExecute = true;
    }

    // Add the block
    motionPipeline.add(block);
    _prevMotionBlockValid = true;

    // Return the change in actuator position
    AxesValues<AxisStepsDataType> newStepsFromOrigin = axesState.getStepsFromOrigin() + block.getStepsToTarget();

#ifdef DEBUG_MOTIONPLANNER_INFO
    LOG_I(MODULE_PREFIX, "^^^^^^^^^^^^^^^^^^^^^^^STEPWISE^^^^^^^^^^^^^^^^^^^^^^^^");
    motionPipeline.debugShowBlocks(axesParams);
#endif

    return newStepsFromOrigin;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add a ramped (variable acceleration) motion block
/// @param args MotionArgs define the parameters for motion
/// @param destActuatorCoords Destination actuator coordinates
/// @param axesState Current state of the axes including position and origin status
/// @param axesParams Parameters for the axes
/// @param motionPipeline Motion pipeline to add the block to
/// @return true if a block was added
bool MotionPlanner::moveToRamped(const MotionArgs& args,
            const AxesValues<AxisStepsDataType>& destActuatorCoords,
            AxesState& axesState,
            const AxesParams& axesParams, 
            MotionPipelineIF& motionPipeline)
{
    // Find first primary axis
    int firstPrimaryAxis = -1;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
        if (axesParams.isPrimaryAxis(axisIdx))
            firstPrimaryAxis = axisIdx;
    if (firstPrimaryAxis == -1)
        firstPrimaryAxis = 0;

    // Find axis deltas and sum of squares of motion on primary axes
    float deltas[AXIS_VALUES_MAX_AXES];
    bool isAMove = false;
    bool isAPrimaryMove = false;
    int axisWithMaxMoveDist = 0;
    float squareSum = 0;
    AxesValues<AxisPosDataType> targetAxesPos;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        // Calculate target position
        targetAxesPos.setVal(axisIdx, args.getAxesPosConst().getVal(axisIdx));

        // Calculate deltas
        deltas[axisIdx] = targetAxesPos.getVal(axisIdx) - axesState.getUnitsFromOrigin(axisIdx);
        if (deltas[axisIdx] != 0)
        {
            isAMove = true;
            if (axesParams.isPrimaryAxis(axisIdx))
            {
                squareSum += pow(deltas[axisIdx], 2);
                isAPrimaryMove = true;
            }
        }

        // Check max distance axis
        if (fabs(deltas[axisIdx]) > fabs(deltas[axisWithMaxMoveDist]))
            axisWithMaxMoveDist = axisIdx;
    }

    // Distance being moved
    float moveDist = sqrt(squareSum);

    // Ignore if there is no real movement
    if (!isAMove || moveDist < MotionBlock::MINIMUM_MOVE_DIST_MM)
        return false;

    // Create a block for this movement which will end up on the pipeline
    MotionBlock block;

    // Set timing
    block.setTimerPeriodNs(_stepGenPeriodNs);
    
    // Set flag to indicate if more moves coming
    block._blockIsFollowed = args.getMoreMovesComing();

    // set end-stop check requirements
    block.setEndStopsToCheck(args.getEndstopCheck());

    // Set motion tracking index if present
    block.setMotionTrackingIndex(args.getMotionTrackingIndex());

    // Compute the requestedVelocity from the first primary axis
    AxisSpeedDataType requestedVelocity = axesParams.getMaxSpeedUps(firstPrimaryAxis);
    if (args.isTargetSpeedValid() && (requestedVelocity > args.getTargetSpeed()))
        requestedVelocity = args.getTargetSpeed();

    // Feedrate percent (scale calculated velocities by this amount)
    double feedrateAsRatioToMax = args.getFeedrate() / 100.0;
    if (args.isFeedrateUnitsPerMin())
    {
        feedrateAsRatioToMax = 1.0;
        if (axesParams.masterAxisMaxSpeed() != 0)
            feedrateAsRatioToMax = args.getFeedrate() / 60.0 / axesParams.masterAxisMaxSpeed();
    }
    requestedVelocity *= feedrateAsRatioToMax;

#ifdef DEBUG_REQUESTED_VELOCITY
    LOG_I(MODULE_PREFIX, "maxSpeed %0.2f targetSpeed %0.2f feedrate %0.2f requestedVelocity %0.2f", axesParams.getMaxSpeedUps(firstPrimaryAxis), 
            args.getTargetSpeed(), args.getFeedrate(), requestedVelocity);
#endif

    // Find the unit vectors for the primary axes and check the feedrate
    AxesValues<AxisUnitVectorDataType> unitVectors;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        if (axesParams.isPrimaryAxis(axisIdx))
        {
            // Unit vector calculation
            unitVectors.setVal(axisIdx, deltas[axisIdx] / moveDist);
        }
    }

    // Store values in the block
    block._requestedSpeed = requestedVelocity;
    block._moveDistPrimaryAxesMM = moveDist;

    // Find if there are any steps
    bool hasSteps = false;
    AxesValues<AxisStepsDataType> stepsToPerform;
    for (int axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        // Check if any steps to perform
        float stepsFloat = destActuatorCoords.getVal(axisIdx) - axesState.getStepsFromOrigin(axisIdx);
        int32_t steps = int32_t(ceilf(stepsFloat));
        if (steps != 0)
            hasSteps = true;
        // Value (and direction)
        stepsToPerform.setVal(axisIdx, steps);
    }
    block.setStepsToTarget(stepsToPerform);

#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
    LOG_I(MODULE_PREFIX, "F %0.2f D %0.2f uX %0.2f uY %0.2f, uZ %0.2f maxStAx %d maxDAx %d %s", 
            requestedVelocity,
            moveDist, 
            unitVectors.getVal(0), unitVectors.getVal(1), unitVectors.getVal(2), 
            block._axisIdxWithMaxSteps, axisWithMaxMoveDist,
            hasSteps ? "has steps" : "NO STEPS");
#endif

    // Check there are some actual steps
    if (!hasSteps)
        return false;

    // Set the dist moved on the axis with max steps
    block._unitVecAxisWithMaxDist = unitVectors.getVal(axisWithMaxMoveDist);

    // If there is a prior block then compute the maximum speed at exit of the second block to keep
    // the max junction deviation (mm) within bounds - there are more comments in the Smoothieware (and GRBL) code
    float maxJunctionDeviationMM = axesParams.getMaxJunctionDeviationMM();
    float vmaxJunctionMMps = _minimumPlannerSpeedMMps;

    // Invalidate the data stored for the prev element if the pipeline becomes empty
    if (!motionPipeline.canGet())
        _prevMotionBlockValid = false;

    // Calculate the maximum speed for the junction between two blocks
    if (isAPrimaryMove && _prevMotionBlockValid)
    {
        float prevParamSpeed = isAPrimaryMove ? _prevMotionBlock._maxParamSpeedMMps : 0;
        if (maxJunctionDeviationMM > 0.0f && prevParamSpeed > 0.0f)
        {
            // Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
            // NOTE: Max junction speed is computed without sin() or acos() by trig half angle identity.
            float cosTheta = - unitVectors.vectorMultSum(_prevMotionBlock._unitVectors);

#ifdef DEBUG_ANGLE_CALCULATIONS
            LOG_I(MODULE_PREFIX, " moveToRamped prevMotion %s newMotion %s cosTheta %0.2f",
                        _prevMotionBlock._unitVectors.toJSON().c_str(), 
                        unitVectors.toJSON().c_str(),
                        cosTheta);
#endif

            // Skip and use default max junction speed for 0 degree acute junction
            if (cosTheta < 0.95F)
            {
                vmaxJunctionMMps = fmin(prevParamSpeed, block._requestedSpeed);
                // Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
                if (cosTheta > -0.95F)
                {
                    // Compute maximum junction speed based on maximum acceleration and junction deviation
                    // Trig half angle identity, always positive
                    float sinThetaD2 = sqrt(0.5F * (1.0F - cosTheta));
                    vmaxJunctionMMps = fmin(vmaxJunctionMMps,
                                            sqrt(axesParams.masterAxisMaxAccel() * maxJunctionDeviationMM * sinThetaD2 /
                                                (1.0F - sinThetaD2)));
                }

#ifdef DEBUG_ANGLE_CALCULATIONS
                LOG_I(MODULE_PREFIX, "moveToRamped cosTheta %0.2f vmaxJnMMps %0.2f maxJnDevMM %0.2f",
                        cosTheta, vmaxJunctionMMps, maxJunctionDeviationMM);
#endif

            }
        }
    }
    block._maxEntrySpeedMMps = vmaxJunctionMMps;

#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
    LOG_I(MODULE_PREFIX, "PrevMoveInQueue %d, maxJunctionDeviationMM %0.2f, blockMaxEntrySpeedMMps %0.2f",
                motionPipeline.canGet(), maxJunctionDeviationMM, block._maxEntrySpeedMMps);
#endif

    // Add the element to the pipeline and remember previous element
    motionPipeline.add(block);
    MotionBlockSequentialData prevBlockInfo;
    prevBlockInfo._maxParamSpeedMMps = block._requestedSpeed;
    prevBlockInfo._unitVectors = unitVectors;
    _prevMotionBlock = prevBlockInfo;
    _prevMotionBlockValid = true;

    // Recalculate the whole queue
    recalculatePipeline(motionPipeline, axesParams);

    // Update current position
    axesState.setPosition(targetAxesPos, block.getStepsToTarget(), true);

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Recalculate the pipeline
/// @param motionPipeline Pipeline to recalculate
/// @param axesParams Parameters for the axes
/// @note This function is called after a block has been added to the pipeline
///       and recalculates the pipeline to ensure that the blocks are correctly
///       set up for the ramp generator to execute.
///       The last block in the pipe (most recently added) will have zero exit speed
///       For each block, walking backwards in the queue :
///         (a) We know the desired exit speed so calculate the entry speed using v^2 = u^2 + 2*a*s
///         (b) Set the exit speed for the previous block from this entry speed
///       Then walk forward in the queue starting with the first block that can be changed:
///         (1) Set the entry speed from the previous block (or to 0 if none)
///         (2) Calculate the max possible exit speed for the block using the same formula as above
///         (3) Set the entry speed for the next block using this exit speed
///       Finally prepare the block for stepper motor actuation
void MotionPlanner::recalculatePipeline(MotionPipelineIF& motionPipeline, const AxesParams &axesParams)
{
#ifdef DEBUG_MOTIONPLANNER_BEFORE
    LOG_I(MODULE_PREFIX, "^^^^^^^^^^^^^^^^^^^^^^^BEFORE RECALC^^^^^^^^^^^^^^^^^^^^^^^^");
    motionPipeline.debugShowBlocks(axesParams);
#endif

    // Iterate the block queue in backwards time order stopping at the first block that has its recalculateFlag false
    int reverseBlockIdx = 0;
    int earliestBlockToReprocess = -1;
    float previousBlockExitSpeed = 0;
    // For the very last block currently in the pipeline the exit speed must be 0 - hence the following line
    // sets the entry speed for the following block initially to 0 as this will be the speed given to exit of
    // that last block in the pipe
    float followingBlockEntrySpeed = 0;
    MotionBlock *pBlock = NULL;
    MotionBlock *pFollowingBlock = NULL;
    while (true)
    {
        // Get the block at current index
        pBlock = motionPipeline.peekNthFromPut(reverseBlockIdx);
        if (pBlock == NULL)
        {
#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
            LOG_I(MODULE_PREFIX, "+++++ No block at reverse idx %d, quitting", reverseBlockIdx);
#endif
            break;
        }
        // Stop if this block is already executing
        if (pBlock->_isExecuting)
        {
            // Get the exit speed from this executing block to use as the entry speed when going forwards
            previousBlockExitSpeed = pBlock->_exitSpeedMMps;
#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
            LOG_I(MODULE_PREFIX, "+++++ Block already executing reverse idx %d", reverseBlockIdx);
#endif
            break;
        }

        // If entry speed is already at the maximum entry speed then we can stop here as no further changes are
        // going to be made by going back further
        if ((pBlock->_entrySpeedMMps == pBlock->_maxEntrySpeedMMps) && (reverseBlockIdx > 1))
        {
#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
            LOG_I(MODULE_PREFIX, "+++++ Optimizing block %d, prevSpeed %f", reverseBlockIdx, pBlock->_exitSpeedMMps);
#endif
            //Get the exit speed from this block to use as the entry speed when going forwards
            previousBlockExitSpeed = pBlock->_exitSpeedMMps;
            break;
        }

        // If there was a following block (remember we're working backwards) then now set the entry speed
        if (pFollowingBlock)
        {
            // Assume for now that that whole block will be deceleration and calculate the max speed we can enter to be able to slow
            // to the exit speed required
            float maxAchievableSpeed = MotionBlock::maxAchievableSpeed(axesParams.masterAxisMaxAccel(),
                                                                    pFollowingBlock->_exitSpeedMMps, pFollowingBlock->_moveDistPrimaryAxesMM);
            pFollowingBlock->_entrySpeedMMps = fmin(maxAchievableSpeed, pFollowingBlock->_maxEntrySpeedMMps);

            // Remember entry speed (to use as exit speed in the next loop)
            followingBlockEntrySpeed = pFollowingBlock->_entrySpeedMMps;
#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
            LOG_I(MODULE_PREFIX, "+++++ Following block (reverse idx %d) entry %0.2f maxAchievable %0.2f maxEntry %0.2f", 
                            reverseBlockIdx, followingBlockEntrySpeed, maxAchievableSpeed, pFollowingBlock->_maxEntrySpeedMMps);
#endif
        }

        // Remember the following block for the next pass
        pFollowingBlock = pBlock;

        // Set the block's exit speed to the entry speed of the block after this one
        pBlock->_exitSpeedMMps = followingBlockEntrySpeed;

        // Remember this as the earliest block to reprocess when going forwards
        earliestBlockToReprocess = reverseBlockIdx;

        // Next
        reverseBlockIdx++;

#ifdef DEBUG_MOTIONPLANNER_DETAILED_INFO
        LOG_I(MODULE_PREFIX, "+++++ Backward pass (reverse idx %d) entry %0.2f(max %0.2f) exit %0.2f", 
                        reverseBlockIdx, pBlock->_entrySpeedMMps, pBlock->_maxEntrySpeedMMps, pBlock->_exitSpeedMMps);
#endif
    }

    // Now iterate in forward time order
    for (reverseBlockIdx = earliestBlockToReprocess; reverseBlockIdx >= 0; reverseBlockIdx--)
    {
        // Get the block to calculate for
        pBlock = motionPipeline.peekNthFromPut(reverseBlockIdx);
        if (!pBlock)
            break;

        // Set the entry speed to the previous block exit speed
        // if (pBlock->_entrySpeedMMps > previousBlockExitSpeed)
        pBlock->_entrySpeedMMps = previousBlockExitSpeed;

        // Calculate maximum speed possible for the block - based on acceleration at the best rate
        AxisSpeedDataType maxExitSpeed = pBlock->maxAchievableSpeed(axesParams.masterAxisMaxAccel(),
                                                        pBlock->_entrySpeedMMps, pBlock->_moveDistPrimaryAxesMM);
        pBlock->_exitSpeedMMps = fmin(maxExitSpeed, pBlock->_exitSpeedMMps);

        // Remember for next block
        previousBlockExitSpeed = pBlock->_exitSpeedMMps;
    }

    // Recalculate acceleration and deceleration curves
    for (reverseBlockIdx = earliestBlockToReprocess; reverseBlockIdx >= 0; reverseBlockIdx--)
    {
        // Get the block to calculate for
        pBlock = motionPipeline.peekNthFromPut(reverseBlockIdx);
        if (!pBlock)
            break;

        // Prepare this block for stepping
        if (pBlock->prepareForStepping(axesParams, false))
        {
            // Check if the block is part of a split block and has at least one more block following it
            // in which case wait until at least two blocks are in the pipeline before locking down the
            // first so that acceleration can be allowed to happen more smoothly
            if ((!pBlock->_blockIsFollowed) || (motionPipeline.count() > 1))
            {
                // No more changes
                pBlock->_canExecute = true;
            }
        }
    }

#ifdef DEBUG_MOTIONPLANNER_AFTER
    LOG_I(MODULE_PREFIX, ".................AFTER RECALC.......................");
    motionPipeline.debugShowBlocks(axesParams);
#elif defined(DEBUG_MOTIONPLANNER_INFO)
    motionPipeline.debugShowTopBlock(axesParams);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Debug show pipeline
/// @param motionPipeline Motion pipeline to show
/// @param minQLen Minimum queue length to show
void MotionPlanner::debugShowPipeline(MotionPipelineIF& motionPipeline, unsigned int minQLen)
{
    if (minQLen != -1 && motionPipeline.count() != minQLen)
        return;
    int curIdx = 0;
    while (MotionBlock *pCurBlock = motionPipeline.peekNthFromGet(curIdx))
    {
        LOG_I(MODULE_PREFIX, "#%d En %0.2f Ex %0.2f (maxEntry %0.2f, requestedVel %0.2f) mm/s", curIdx,
                    pCurBlock->_entrySpeedMMps, pCurBlock->_exitSpeedMMps,
                    pCurBlock->_maxEntrySpeedMMps, pCurBlock->_requestedSpeed);
        // Next
        curIdx++;
    }
}
