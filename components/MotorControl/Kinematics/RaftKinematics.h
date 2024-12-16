/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftKinematics
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesValues.h"
#include "AxesState.h"
#include "AxesParams.h"
#include "MotionArgs.h"

/// @brief Kinematics
/// This class is responsible for converting between cartesian coordinates and actuator values
/// It is used by the motion planner to convert between the two coordinate systems
/// The class is abstract and must be implemented by a derived class
class RaftKinematics
{
public:
    virtual ~RaftKinematics()
    {
    }

    /// @brief Convert a point in cartesian to actuator steps
    /// @param targetPt Target point cartesian from origin
    /// @param outActuator Output actuator in absolute steps from origin
    /// @param curAxesState Current position (in both units and steps from origin)
    /// @param axesParams Axes parameters
    /// @param constrainToBounds Constrain out of bounds (if not constrained then return false if the point is out of bounds)
    /// @param minimizeMotion Minimize motion
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                            AxesValues<AxisStepsDataType>& outActuator,
                            const AxesState& curAxesState,
                            const AxesParams& axesParams,
                            bool constrainToBounds,
                            bool minimizeMotion) const = 0;

    /// @brief Convert actuator steps to a point in cartesian
    /// @param targetActuator Target actuator steps
    /// @param outPt Output point in cartesian
    /// @param curAxesState Current axes state (axes position and origin status)
    /// @param axesParams Axes parameters
    /// @return true if successful
    virtual bool actuatorToPt(const AxesValues<AxisStepsDataType> &targetActuator, 
                AxesValues<AxisPosDataType> &outPt, 
                const AxesState &curAxesState, 
                const AxesParams &axesParams) const = 0;

    // Correct step overflow (necessary in continuous rotation bots)
    virtual void correctStepOverflow(AxesState &curAxesState, 
                const AxesParams &axesParams) const
    {
    }

    /// @brief Pre-process coordinates
    /// @param args Motion arguments (may be modified) including target position
    /// @param axesParams Axes parameters
    /// @return Distance to move in MM
    /// @note This is used to manage unspecified axes and for coordinate systems like Theta-Rho 
    ///       which are curret-position dependent
    virtual AxisDistDataType preProcessCoords(MotionArgs& args,
                const AxesState& axesState,
                const AxesParams& axesParams) const
    {
        // Check for any axes not specified and/or require relative motion
        double movementDistSumSq = 0;
        AxesValues<AxisPosDataType> axisPositions;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
        {
            // Get target axis position
            AxisPosDataType targetAxisPos = 0;
            if (!args.getAxesSpecified().getVal(i))
                targetAxisPos = axesState.getUnitsFromOrigin(i);
            else if (args.isRelative())
                targetAxisPos = axesState.getUnitsFromOrigin(i) + args.getAxesPosConst().getVal(i);
            else
                targetAxisPos = args.getAxesPosConst().getVal(i);

            // Set the target axis position
            axisPositions.setVal(i, targetAxisPos);

            // Handle distance calculation
            if (axesParams.isPrimaryAxis(i))
                movementDistSumSq += pow(targetAxisPos - axesState.getUnitsFromOrigin(i), 2);
        }

        // Set back into args
        args.setAxesPositions(axisPositions);

        // Return block distance (for splitting up into blocks if required)
        return sqrt(movementDistSumSq);
    }
};
