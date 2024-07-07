/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftKinematics
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesValues.h"

class AxesState;
class AxesParams;

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
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                              AxesValues<AxisStepsDataType>& outActuator,
                              const AxesState& curAxesState,
                              const AxesParams& axesParams) const = 0;

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

    // Convert coordinates (used for coordinate systems like Theta-Rho which are position dependent)
    // This doesn't convert coords - just checks for things like wrap around in circular coordinate systems
    // Note that values are modified in-place
    virtual void preProcessCoords(AxesValues<AxisPosAndValidDataType>& axisPositions, const AxesParams& axesParams) const
    {
    }
};
