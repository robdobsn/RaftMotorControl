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
    /// @param args Motion arguments (includes out of bounds action)
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                              AxesValues<AxisStepsDataType>& outActuator,
                              const AxesState& curAxesState,
                              const AxesParams& axesParams,
                              OutOfBoundsAction outOfBoundsAction) const = 0;

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

    /// @brief Check if this kinematics supports alternate IK solutions
    /// @return true if alternate solutions are available (e.g., SCARA elbow-up/down)
    /// @note Default implementation returns false (most kinematics have single solution)
    virtual bool supportsAlternateSolutions() const
    {
        return false;
    }

    /// @brief Set preference for alternate IK solution
    /// @param prefer True to prefer alternate solution
    /// @note Only has effect if supportsAlternateSolutions() returns true
    /// @note Default implementation does nothing
    virtual void setPreferAlternateSolution(bool prefer) const
    {
        // Default: no-op for kinematics without alternate solutions
        (void)prefer;
    }

    /// @brief Get current alternate solution preference
    /// @return True if preferring alternate solution
    /// @note Default implementation returns false
    virtual bool getPreferAlternateSolution() const
    {
        return false;
    }

    /// @brief Get Cartesian workspace bounds for proportionate coordinate conversion
    /// @param axisIdx Axis index (0=X, 1=Y, etc.)
    /// @param minVal Output minimum value for this axis
    /// @param maxVal Output maximum value for this axis
    /// @param axesParams Axes parameters
    /// @return true if bounds are available for this axis
    /// @note Default implementation uses axesParams minUnits/maxUnits
    /// @note Kinematics with non-Cartesian joints (like SCARA) should override this
    ///       to provide the Cartesian workspace bounds instead of joint bounds
    virtual bool getCartesianWorkspaceBounds(uint32_t axisIdx, 
                                             AxisPosDataType& minVal, 
                                             AxisPosDataType& maxVal,
                                             const AxesParams& axesParams) const
    {
        // Default: use axis params (works for Cartesian kinematics like XYZ)
        minVal = axesParams.getMinUnits(axisIdx);
        maxVal = axesParams.getMaxUnits(axisIdx);
        // Return true only if bounds are actually set (non-zero range)
        return (maxVal - minVal) > 0.001;
    }

    /// @brief Validate that all intermediate points in a linear path are reachable
    /// @param startPt Start point in Cartesian coordinates
    /// @param endPt End point in Cartesian coordinates
    /// @param numSegments Number of segments to test along the path
    /// @param curAxesState Current axes state
    /// @param axesParams Axes parameters
    /// @return true if all intermediate points are reachable
    /// @note Default implementation returns true (assumes all points reachable)
    /// @note Kinematics with constraints (like SCARA) should override this
    virtual bool validateLinearPath(const AxesValues<AxisPosDataType>& startPt,
                                   const AxesValues<AxisPosDataType>& endPt,
                                   uint32_t numSegments,
                                   const AxesState& curAxesState,
                                   const AxesParams& axesParams) const
    {
        // Default: assume all paths are valid (e.g., for Cartesian kinematics)
        (void)startPt;
        (void)endPt;
        (void)numSegments;
        (void)curAxesState;
        (void)axesParams;
        return true;
    }
};
