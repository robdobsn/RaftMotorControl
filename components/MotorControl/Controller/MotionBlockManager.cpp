/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlockManager
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionBlockManager.h"
#include "Geometries/AxisGeomBase.h"
#include "Geometries/AxisGeomXYZ.h"

#define DEBUG_RAMPED_BLOCK
#define DEBUG_COORD_UPDATES
#define DEBUG_BLOCK_SPLITTER

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Static
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const char* MODULE_PREFIX = "MotionBlockManager";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionBlockManager::MotionBlockManager(MotionPipeline& motionPipeline, 
                MotorEnabler& motorEnabler, 
                AxesParams& axesParams)
                :   _motionPipeline(motionPipeline), 
                    _motorEnabler(motorEnabler), 
                    _axesParams(axesParams)
{
    clear();
}

MotionBlockManager::~MotionBlockManager()
{
    // Remove geometry
    if (_pAxisGeometry)
        delete _pAxisGeometry;
}

void MotionBlockManager::clear()
{
    _numBlocks = 0;
    _nextBlockIdx = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::setup(const String& geometry, bool allowAllOutOfBounds, 
            double junctionDeviation, bool homingNeededBeforeAnyMove,
            uint32_t stepGenPeriodUs)
{
    // Allow out-of-bounds motion and homing needs
    _allowAllOutOfBounds = allowAllOutOfBounds;
    _homingNeededBeforeAnyMove = homingNeededBeforeAnyMove;

    // Motion Pipeline and Planner
    _motionPlanner.setup(junctionDeviation, stepGenPeriodUs);

    // Configure geometry
    if (_pAxisGeometry)
    {
        delete _pAxisGeometry;
        _pAxisGeometry = NULL;
    }
    if (geometry.equalsIgnoreCase("XYZ"))
        _pAxisGeometry = new AxisGeomXYZ();    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motion helper for linear motion
// Linear motion is used for homing, etc
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionBlockManager::addLinearBlock(const MotionArgs& args)
{
    AxesParamVals<AxisStepsDataType> stepsFromHome = _motionPlanner.moveToLinear(args, 
                    _lastCommandedAxesPositions.stepsFromHome, 
                    _axesParams, 
                    _motionPipeline);

    // Since this was a linear move units from home is now invalid
    _lastCommandedAxesPositions.setUnitsFromHomeValidity(false);
    _lastCommandedAxesPositions.stepsFromHome = stepsFromHome;

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add block to be split
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionBlockManager::addRampedBlock(const MotionArgs& args, 
                const AxesPosValues& targetPosition, 
                uint32_t numBlocks)
{
    _blockMotionArgs = args;
    _targetPosition = targetPosition;
    _numBlocks = numBlocks;
    _nextBlockIdx = 0;
    _blockDeltaDistance = (_targetPosition - _lastCommandedAxesPositions.unitsFromHome) / double(numBlocks);

#ifdef DEBUG_RAMPED_BLOCK
    LOG_I(MODULE_PREFIX, "moveTo cur %s curSteps %s new %s numBlocks %d blockDeltaDist %s)",
                _lastCommandedAxesPositions.unitsFromHome.getDebugStr().c_str(),
                _lastCommandedAxesPositions.stepsFromHome.getDebugStr().c_str(),
                _targetPosition.getDebugStr().c_str(),
                _numBlocks, 
                _blockDeltaDistance.getDebugStr().c_str());
#endif

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// pumpBlockSplitter - should be called regularly 
// A single moveTo command can be split into blocks - this function checks if such
// splitting is in progress and adds the split-up motion blocks accordingly
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::pumpBlockSplitter()
{
    // Check if we can add anything to the pipeline
    while (_motionPipeline.canAccept())
    {
        // Check if any blocks remain to be expanded out
        if (_numBlocks <= 0)
            return;

        // Add to pipeline any blocks that are waiting to be expanded out
        AxesPosValues nextBlockDest = _lastCommandedAxesPositions.unitsFromHome + _blockDeltaDistance;

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
                    _lastCommandedAxesPositions.unitsFromHome.getDebugStr().c_str(),
                    _blockDeltaDistance.getDebugStr().c_str(),
                    nextBlockDest.getDebugStr().c_str(),
                    _blockMotionArgs.getAxesPositions().getDebugStr().c_str(), 
                    _nextBlockIdx,
                    _numBlocks);
#endif

        // Add to planner
        addToPlanner(_blockMotionArgs);

        // Enable motors
        _motorEnabler.enableMotors(true, false);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add a movement to the pipeline using the planner which computes suitable motion
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionBlockManager::addToPlanner(const MotionArgs &args)
{
    // Check we are not stopping
    // TODO 2021
    // if (_stopRequested)
    //     return false;
            
    // Convert the move to actuator coordinates
    AxesParamVals<AxisStepsDataType> actuatorCoords;
    if (!_pAxisGeometry)
    {
        LOG_W(MODULE_PREFIX, "addToPlanner no geometry set");
        return false;
    }

    _pAxisGeometry->ptToActuator(args.getAxesPositions(), 
            actuatorCoords, 
            _curPosition, 
            _axesParams,
            args.getAllowOutOfBounds() || _allowAllOutOfBounds);

    // Plan the move
    bool moveOk = _motionPlanner.moveToRamped(args, actuatorCoords, 
                        _lastCommandedAxesPositions, _axesParams, _motionPipeline);
#ifdef DEBUG_COORD_UPDATES
    LOG_I(MODULE_PREFIX, "addToPlanner moveOk %d pt %s actuator %s Allow OOB Global %d Point %d", 
            moveOk,
            args.getAxesPositions().getDebugStr().c_str(),
            actuatorCoords.toJSON().c_str(),
            args.getAllowOutOfBounds(), 
            _allowAllOutOfBounds);
#endif

    // Correct overflows if necessary
    if (moveOk)
    {
        // TODO check that this is already done in moveToRamped - it updates _lastCommandedAxisPos
        // // Update axisMotion
        // _lastCommandedAxisPos.unitsFromHome = args.getPointMM();

        // Correct overflows
        // TODO re-implement
        // if (_correctStepOverflowFn)
        // {
        //     _correctStepOverflowFn(_lastCommandedAxesPositions, _axesParams);
        // }            
#ifdef DEBUG_COORD_UPDATES
        LOG_I(MODULE_PREFIX, "addToPlanner updatedAxisPos %s",
            _lastCommandedAxesPositions.unitsFromHome.getDebugStr().c_str());
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

void MotionBlockManager::coordsActuatorToRealWorld(const AxesParamVals<AxisStepsDataType> &targetActuator, 
            AxesPosValues &outPt) const
{
    if (!_pAxisGeometry)
    {
        LOG_W(MODULE_PREFIX, "coordsActuatorToRealWorld no geometry set");
        return;
    }

    _pAxisGeometry->actuatorToPt(targetActuator, outPt, _curPosition, _axesParams);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pre-process coordinates (used for coordinate systems like Theta-Rho which are position dependent)
// This doesn't convert coords - just checks for things like wrap around in circular coordinate systems
// Note that values are modified in-place
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::preProcessCoords(AxesPosValues& axisPositions, const AxesParams& axesParams) const
{
    if (_pAxisGeometry)
        _pAxisGeometry->preProcessCoords(axisPositions, axesParams);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set current position as home
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionBlockManager::setCurPositionAsHome(uint32_t axisIdx)
{
    if (axisIdx >= AXIS_VALUES_MAX_AXES)
        return;
    _lastCommandedAxesPositions.unitsFromHome.setVal(axisIdx, _axesParams.getHomeOffsetVal(axisIdx));
    _lastCommandedAxesPositions.setUnitsFromHomeValidity(true);
    _lastCommandedAxesPositions.stepsFromHome.setVal(axisIdx, _axesParams.gethomeOffSteps(axisIdx));
    // _trinamicsController.setTotalStepPosition(i, _axesParams.gethomeOffSteps(i));
#ifdef DEBUG_COORD_UPDATES
        LOG_I(MODULE_PREFIX, "setCurPosAsHome axisIdx %d curMM %0.2f steps %d", axisIdx,
                    _lastCommandedAxesPositions.unitsFromHome.getVal(axisIdx),
                    _lastCommandedAxesPositions.stepsFromHome.getVal(axisIdx));
#endif
}
