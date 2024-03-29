/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionBlockManager
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesParams.h"
#include "AxesPosition.h"
#include "MotionArgs.h"
#include "MotorEnabler.h"
#include "MotionPlanner.h"
#include "GeometryManager.h"

class AxisGeomBase;
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
    void setup(const String& geometry, bool allowAllOutOfBounds, double junctionDeviation, 
                bool homingNeededBeforeAnyMove, uint32_t stepGenPeriodUs);

    // pumpBlockSplitter - should be called regularly 
    // A single moveTo command can be split into blocks - this function checks if such
    // splitting is in progress and adds the split-up motion blocks accordingly
    void pumpBlockSplitter(MotionPipelineIF& motionPipeline);

    // Check is busy
    bool isBusy() const
    {
        return _numBlocks != 0;
    }

    // Add linear motion block
    bool addLinearBlock(const MotionArgs& args, MotionPipelineIF& motionPipeline);

    // Add rampled block (which may be split up)
    bool addRampedBlock(const MotionArgs& args, 
                const AxesPosValues& targetPosition, 
                uint32_t numBlocks);

    // Get last commanded position in axes units
    AxesPosValues getLastPos() const
    {
        return _lastCommandedAxesPositions.unitsFromHome;
    }

    // Check last commanded position is valid
    bool lastPosValid() const
    {
        return _lastCommandedAxesPositions.unitsFromHomeValid();
    }

    // Convert actuator coords to real-world coords
    void coordsActuatorToRealWorld(const AxesParamVals<AxisStepsDataType> &targetActuator, 
                AxesPosValues &outPt) const;

    // Convert coordinates (used for coordinate systems like Theta-Rho which are position dependent)
    // This doesn't convert coords - just checks for things like wrap around in circular coordinate systems
    // Note that values are modified in-place
    void preProcessCoords(AxesPosValues& axisPositions, const AxesParams& axesParams) const;

    // Set current position as home
    void setCurPositionAsHome(uint32_t axisIdx);

    // Homing needed before any move
    bool isHomingNeededBeforeMove() const
    {
        return _homingNeededBeforeAnyMove;
    }
    
private:
    // Args for motion
    MotionArgs _blockMotionArgs;

    // Current position
    AxesPosition _curPosition;

    // Target position
    AxesPosValues _targetPosition;

    // Block delta distance
    AxesPosValues _blockDeltaDistance;

    // Num blocks to split over
    uint32_t _numBlocks = 0;

    // Next block to return
    uint32_t _nextBlockIdx = 0;

    // Planner used to plan the pipeline of motion
    MotionPlanner _motionPlanner;

    // Motor enabler
    MotorEnabler& _motorEnabler;

    // Geometry manager
    GeometryManager _geometryManager;

    // Axes parameters
    AxesParams& _axesParams;

    // Last commanded axes positions
    AxesPosition _lastCommandedAxesPositions;

    // Allow all out of bounds movement
    bool _allowAllOutOfBounds = false;

    // Homing is needed before any movement
    bool _homingNeededBeforeAnyMove = false;

    // Helpers
    bool addToPlanner(const MotionArgs &args, MotionPipelineIF& motionPipeline);
};
