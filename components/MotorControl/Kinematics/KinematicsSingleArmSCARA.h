/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// KinematicsSingleArmSCARA
//
// Rob Dobson 2016-2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftKinematics.h"
#include "AxesParams.h"
#include "AxisUtils.h"
#include "AxesState.h"

// Warn
#define WARN_KINEMATICS_SA_SCARA_POS_OUT_OF_BOUNDS

// Debug
// #define DEBUG_KINEMATICS_SA_SCARA
// #define DEBUG_KINEMATICS_SA_SCARA_SETUP
// #define DEBUG_KINEMATICS_SA_SCARA_RELATIVE_ANGLE

class KinematicsSingleArmSCARA : public RaftKinematics
{
public:

    /// @brief create method for factory
    /// @return new instance of this class
    static RaftKinematics *create(const RaftJsonIF& config)
    {
        return new KinematicsSingleArmSCARA(config);
    }

    /// @brief Constructor
    /// @param config Configuration
    KinematicsSingleArmSCARA(const RaftJsonIF& config)
    {
        _arm1LenMM = AxisPosDataType(config.getDouble("arm1LenMM", DEFAULT_ARM_LENGTH_MM));
        _arm2LenMM = AxisPosDataType(config.getDouble("arm2LenMM", DEFAULT_ARM_LENGTH_MM));
        _maxRadiusMM = AxisPosDataType(config.getDouble("maxRadiusMM", _arm1LenMM + _arm2LenMM));
        _originTheta2OffsetDegrees = AxisPosDataType(config.getDouble("originTheta2OffsetDegrees", 180));

        // Check validity
        if (_arm1LenMM < MIN_ARM_LENGTH_MM)
            _arm1LenMM = DEFAULT_ARM_LENGTH_MM;
        if (_arm2LenMM < MIN_ARM_LENGTH_MM)
            _arm2LenMM = DEFAULT_ARM_LENGTH_MM;
        if (_maxRadiusMM > _arm1LenMM + _arm2LenMM)
            _maxRadiusMM = _arm1LenMM + _arm2LenMM;

#ifdef DEBUG_KINEMATICS_SA_SCARA_SETUP
        LOG_I(MODULE_PREFIX, "arm1Len %.2fmm arm2Len %.2fmm maxRadius %.2fmm",
                _arm1LenMM, _arm2LenMM, _maxRadiusMM);
#endif
    }

    /// @brief Convert a point in cartesian to actuator steps
    /// @param targetPt Target point cartesian from origin
    /// @param outActuator Output actuator in absolute steps from origin
    /// @param curAxesState Current position (in both units and steps from origin)
    /// @param axesParams Axes parameters
    /// @param constrainToBounds Constrain out of bounds (if not constrained then return false if the point is out of bounds)
    /// @param minimizeMotion Minimize motion to the target point
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                            AxesValues<AxisStepsDataType>& outActuator,
                            const AxesState& curAxesState,
                            const AxesParams& axesParams,
                            bool constrainToBounds,
                            bool minimizeMotion) const override final
    {
        // Initialize solutions for polar coordinates
        AxesValues<AxisCalcDataType> targetSoln1, targetSoln2;

        // Convert Cartesian coordinates to polar coordinates
        bool posValid = cartesianToPolar(targetPt, targetSoln1, targetSoln2, axesParams);

        // Handle out-of-bounds
        if (!posValid)
        {
            if (!constrainToBounds)
            {
                return false;
            }
            // Constrain the point to bounds
            AxesValues<AxisPosDataType> constrainedPt = targetPt;
            axesParams.constrainPtToBounds(constrainedPt);

            // Recompute polar coordinates for the constrained point
            posValid = cartesianToPolar(constrainedPt, targetSoln1, targetSoln2, axesParams);

            // If the constrained point is still invalid, return false
            if (!posValid)
            {
                return false;
            }
        }

        // Choose the solution to minimize motion if requested
        AxesValues<AxisCalcDataType> chosenSolution, altSolution;
        if (minimizeMotion)
        {
            // Current angles
            AxesValues<AxisCalcDataType> curAngles;
            calculateAngles(curAxesState, curAngles, axesParams);
            LOG_I(MODULE_PREFIX, "ptToActuator curAngles %.2f° %.2f°", curAngles.getVal(0), curAngles.getVal(1));

            // // Check for a position close to the origin (both angles are 0)
            // if (fabs(targetPt.getVal(0)) < CLOSE_TO_ORIGIN_TOLERANCE_MM && fabs(targetPt.getVal(1)) < CLOSE_TO_ORIGIN_TOLERANCE_MM)
            // {
            //     // This solution involves only moving the elbow joint
            //     chosenSolution = { curAngles[0], -curAngles[0] };
            //     altSolution = { -curAngles[0], curAngles[0] };
            // }
            // else
            // {
                // Calculate relative angles for both solutions
                AxesValues<AxisCalcDataType> relAngleSoln1 = {
                    computeRelativeAngle(targetSoln1.getVal(0), curAngles[0]),
                    computeRelativeAngle(targetSoln1.getVal(1), curAngles[1])
                };

                AxesValues<AxisCalcDataType> relAngleSoln2 = {
                    computeRelativeAngle(targetSoln2.getVal(0), curAngles[0]),
                    computeRelativeAngle(targetSoln2.getVal(1), curAngles[1])
                };

                // Compute step distances for both solutions
                int32_t stepsSoln1Max = std::max(
                    abs(int32_t(relAngleSoln1.getVal(0) * axesParams.getStepsPerRot(0) / 360.0)),
                    abs(int32_t(relAngleSoln1.getVal(1) * axesParams.getStepsPerRot(1) / 360.0))
                );

                int32_t stepsSoln2Max = std::max(
                    abs(int32_t(relAngleSoln2.getVal(0) * axesParams.getStepsPerRot(0) / 360.0)),
                    abs(int32_t(relAngleSoln2.getVal(1) * axesParams.getStepsPerRot(1) / 360.0))
                );

                // Select the solution that minimizes the maximum step distance
                chosenSolution = (stepsSoln1Max <= stepsSoln2Max) ? relAngleSoln1 + curAngles : relAngleSoln2 + curAngles;
                altSolution = (stepsSoln1Max <= stepsSoln2Max) ? relAngleSoln2 + curAngles : relAngleSoln1 + curAngles;
            // }
        }
        else
        {
            // Default to using targetSoln1
            chosenSolution = targetSoln1;
            altSolution = targetSoln2;
        }

        // Convert the chosen solution to absolute actuator steps
        absAnglesToAbsSteps(chosenSolution, curAxesState, outActuator, axesParams);

        // Debug
    #ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "ptToActuator X %.2fmm (cur %.2fmm) Y %.2fmm (cur %.2fmm) moveDist %.2fmm best Θ1 %.2f° (steps %d) Θ2 %.2f° (steps %d) [alt Θs %.2f° %.2f°]",
                targetPt.getVal(0), curAxesState.getUnitsFromOrigin(0), 
                targetPt.getVal(1), curAxesState.getUnitsFromOrigin(1),
                sqrt(pow(targetPt.getVal(0) - curAxesState.getUnitsFromOrigin(0), 2) + pow(targetPt.getVal(1) - curAxesState.getUnitsFromOrigin(1), 2)),
                chosenSolution.getVal(0), outActuator.getVal(0),
                chosenSolution.getVal(1), outActuator.getVal(1),
                altSolution.getVal(0), altSolution.getVal(1));
    #endif
        return true;
    }

    /// @brief Convert actuator steps to a point in cartesian
    /// @param inActuator Actuator steps
    /// @param outPt Output point in cartesian
    /// @param curAxesState Current axes state (axes position and origin status)
    /// @param axesParams Axes parameters
    /// @return true if successful
    virtual bool actuatorToPt(const AxesValues<AxisStepsDataType>& inActuator,
                            AxesValues<AxisPosDataType>& outPt,
                            const AxesState& curAxesState,
                            const AxesParams& axesParams) const override final
    {
        // Convert actuator steps to angles in degrees
        AxisCalcDataType theta1Degrees = inActuator.getVal(0) * 360.0 / axesParams.getStepsPerRot(0);
        AxisCalcDataType theta2Degrees = inActuator.getVal(1) * 360.0 / axesParams.getStepsPerRot(1) + _originTheta2OffsetDegrees;

        // Convert degrees to radians
        AxisCalcDataType theta1Rad = AxisUtils::d2r(theta1Degrees);
        AxisCalcDataType theta2Rad = AxisUtils::d2r(theta2Degrees);

        // Calculate Cartesian coordinates
        AxisCalcDataType x = _arm1LenMM * cos(theta1Rad) + _arm2LenMM * cos(theta1Rad + theta2Rad);
        AxisCalcDataType y = _arm1LenMM * sin(theta1Rad) + _arm2LenMM * sin(theta1Rad + theta2Rad);

        // Set the output Cartesian point
        outPt = {static_cast<AxisPosDataType>(x), static_cast<AxisPosDataType>(y)};

    #ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "actuatorToPt steps [%d, %d] -> angles [%.2f°, %.2f°] -> cartesian [%.2fmm, %.2fmm]",
            inActuator.getVal(0), inActuator.getVal(1),
            theta1Degrees, theta2Degrees,
            x, y);
    #endif

        return true;
    }


    /// @brief Get arm lengths
    /// @param arm1LenMM Output arm 1 length in mm
    /// @param arm2LenMM Output arm 2 length in mm
    void getArmLengths(AxisPosDataType& arm1LenMM, AxisPosDataType& arm2LenMM) const
    {
        arm1LenMM = _arm1LenMM;
        arm2LenMM = _arm2LenMM;
    }

    /// @brief Get max radius
    /// @return Max radius in mm
    AxisPosDataType getMaxRadiusMM() const
    {
        return _maxRadiusMM;
    }

    /// @brief Get origin theta2 offset
    /// @return Origin theta2 offset in degrees
    AxisPosDataType getOriginTheta2OffsetDegrees() const
    {
        return _originTheta2OffsetDegrees;
    }

private:
    static constexpr const char *MODULE_PREFIX = "KinematicsSingleArmSCARA";

    /// @brief Convert from Cartesian to Polar coordinates
    /// @param targetPt Target point in Cartesian coordinates
    /// @param targetSoln1 Output solution 1 in Polar coordinates
    /// @param targetSoln2 Output solution 2 in Polar coordinates
    /// @param axesParams Axes parameters
    /// @return true if successful
    bool cartesianToPolar(const AxesValues<AxisPosDataType>& targetPt, 
            AxesValues<AxisCalcDataType>& targetSoln1, 
            AxesValues<AxisCalcDataType>& targetSoln2, 
            const AxesParams& axesParams) const
    {
        // Calculate distance from origin to pt (forms one side of triangle where arm segments form other sides)
        AxisCalcDataType thirdSideL3MM = sqrt(pow(targetPt.getVal(0), 2) + pow(targetPt.getVal(1), 2));

        // Check validity of position
        bool posValid = thirdSideL3MM <= (_arm1LenMM + _arm2LenMM) && 
                        (thirdSideL3MM >= fabs(_arm1LenMM - _arm2LenMM)) &&
                        thirdSideL3MM <= _maxRadiusMM;

        // Check valid
        if (posValid)
        {

            // Check for close to origin (third side is less than tolerance)
            if (thirdSideL3MM < CLOSE_TO_ORIGIN_TOLERANCE_MM)
            {
                targetSoln1 = { 0, 0 };
                targetSoln2 = { 0, 0 };
#ifdef DEBUG_KINEMATICS_SA_SCARA
                LOG_I(MODULE_PREFIX, "cartesianToPolar: Close to origin X%.2fmm Y%.2fmm theta1 %.2fd°/%.2f° theta2 %.2f°/%.2f° 3rdSide %.2fmm l1 %.2fmm l2 %.2fmm",
                        targetPt.getVal(0), targetPt.getVal(1),
                        targetSoln1.getVal(0), targetSoln2.getVal(0),
                        targetSoln1.getVal(1), targetSoln2.getVal(1),
                        thirdSideL3MM,
                        _arm1LenMM, _arm2LenMM);
#endif
            }
            else
            {

                // Calculate angle from x-axis to target point
                AxisCalcDataType targetAngleRads = atan2(targetPt.getVal(1), targetPt.getVal(0));

                // Calculate angle of triangle opposite L2
                AxisCalcDataType a2 = AxisUtils::cosineRule(thirdSideL3MM, _arm1LenMM, _arm2LenMM);

                // Calculate angle of triangle opposite side 3 (neither L1 nor L2)
                AxisCalcDataType a3 = AxisUtils::cosineRule(_arm1LenMM, _arm2LenMM, thirdSideL3MM);

                // Check for valid angles
                if (std::isnan(a2) || std::isnan(a3))
                {
                    posValid = false;
#ifdef DEBUG_KINEMATICS_SA_SCARA
                    LOG_W(MODULE_PREFIX, "cartesianToPolar: Unexpected NAN a2 %.2f a3 %.2f", a2, a3);
#endif
                }
                else
                {
                    // Calculate the alpha and beta angles in degrees
                    targetSoln1 = { AxisUtils::r2d(targetAngleRads - a2), AxisUtils::r2d(M_PI + targetAngleRads - a2 - a3 - AxisUtils::d2r(_originTheta2OffsetDegrees)) };
                    targetSoln2 = { AxisUtils::r2d(targetAngleRads + a2), AxisUtils::r2d(-M_PI + targetAngleRads + a2 + a3 - AxisUtils::d2r(_originTheta2OffsetDegrees)) };

#ifdef DEBUG_KINEMATICS_SA_SCARA
                    LOG_I(MODULE_PREFIX, "cartesianToPolar X%.2fmm Y%.2fmm theta1 %.2f°/%.2f° theta2 %.2f°/%.2f° targetAngle %.2f° 3rdSide %.2fmm a1 %.2fd a2 %.2f° a3 %.2f° l1 %.2fmm l2 %.2fmm",
                            targetPt.getVal(0), targetPt.getVal(1),
                            targetSoln1.getVal(0), targetSoln2.getVal(0),
                            targetSoln1.getVal(1), targetSoln2.getVal(1),
                            AxisUtils::r2d(targetAngleRads),
                            thirdSideL3MM,
                            AxisUtils::r2d(M_PI-a2-a3), AxisUtils::r2d(a2), AxisUtils::r2d(a3),
                            _arm1LenMM, _arm2LenMM);
#endif            
                }
            }

        }
        else
        {
#ifdef WARN_KINEMATICS_SA_SCARA_POS_OUT_OF_BOUNDS
            LOG_W(MODULE_PREFIX, "cartesianToPolar: Point out of reach: X %.2fmm Y %.2fmm L3 %.2fmm", 
                targetPt.getVal(0), targetPt.getVal(1), thirdSideL3MM);
#endif
        }

        return posValid;        
    }

    /// @brief Calculate the current axis angles
    /// @param curAxesState Current axes state (includes current position in steps from origin)
    /// @param anglesDegrees Output angles in degrees
    /// @param axesParams Axes parameters
    void calculateAngles(const AxesState& curAxesState,
                AxesValues<AxisCalcDataType>& anglesDegrees, 
                const AxesParams& axesParams) const
    {
        // All angles returned are in degrees anticlockwise from the x-axis
        AxisCalcDataType theta1Degrees = AxisUtils::wrapDegrees(curAxesState.getStepsFromOrigin(0) * 360.0 / axesParams.getStepsPerRot(0));
        AxisCalcDataType theta2Degrees = AxisUtils::wrapDegrees(curAxesState.getStepsFromOrigin(1) * 360.0 / axesParams.getStepsPerRot(1));
        anglesDegrees = { theta1Degrees, theta2Degrees };
#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "calculateAngles ax0Steps %d ax1Steps %d a %.2f° b %.2f°",
                curAxesState.getStepsFromOrigin(0), curAxesState.getStepsFromOrigin(1), anglesDegrees.getVal(0), anglesDegrees.getVal(1));
#endif        
    }

    /// @brief Calculate the relative angle (handling wrap-around at 0/360 degrees)
    /// @param targetRotation Target rotation
    /// @param curRotation Current rotation
    /// @return Relative angle
    double computeRelativeAngle(AxisCalcDataType targetRotation, AxisCalcDataType curRotation) const
    {
        // Calculate the difference angle
        double diffAngle = targetRotation - curRotation;

        // For angles between -180 and +180 just use the diffAngle
        double bestRotation = diffAngle;
        if (diffAngle <= -180)
            bestRotation = 360.0 + diffAngle;
        else if (diffAngle > 180)
            bestRotation = diffAngle - 360.0;
#ifdef DEBUG_KINEMATICS_SA_SCARA_RELATIVE_ANGLE
        LOG_I(MODULE_PREFIX, "computeRelativeAngle: target %.2f° cur %.2f° diff %.2f° best %.2f°",
                targetRotation, curRotation, diffAngle, bestRotation);
#endif
        return bestRotation;
    }

    /// @brief Convert absolute angles to absolute steps
    /// @param relativeAngles Absolute angles
    /// @param curAxesState Current axes state (position and origin status)
    /// @param outActuator Output actuator steps
    /// @param axesParams Axes parameters
    void absAnglesToAbsSteps(const AxesValues<AxisCalcDataType>& absAngles, 
            const AxesState& curAxesState, 
            AxesValues<AxisStepsDataType>& outActuator, 
            const AxesParams& axesParams) const
    {
        // Convert relative polar to steps
        int32_t stepsAbs0 = int32_t(roundf(absAngles.getVal(0) * axesParams.getStepsPerRot(0) / 360.0));
        int32_t stepsAbs1 = int32_t(roundf(absAngles.getVal(1) * axesParams.getStepsPerRot(1) / 360.0));

        // Add to existing
        outActuator.setVal(0, stepsAbs0);
        outActuator.setVal(1, stepsAbs1);
#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "absAnglesToAbsSteps [0] absAngle %.2f° absSteps %d [1] absAngle %.2f° absSteps %d",
                absAngles.getVal(0), outActuator.getVal(0), absAngles.getVal(1), outActuator.getVal(1));
#endif        
    }

    // Arm lengths in mm
    static const constexpr AxisPosDataType MIN_ARM_LENGTH_MM = 0.1;
    static const constexpr AxisPosDataType DEFAULT_ARM_LENGTH_MM = 100.0;
    AxisPosDataType _arm1LenMM = DEFAULT_ARM_LENGTH_MM;
    AxisPosDataType _arm2LenMM = DEFAULT_ARM_LENGTH_MM;

    // Max radius in mm
    AxisPosDataType _maxRadiusMM = DEFAULT_ARM_LENGTH_MM + DEFAULT_ARM_LENGTH_MM;

    // Origin theta2 offset in degrees (theta2 is the angle of the second arm anticlockwise from the x-axis)
    // 180 degrees is the default for a SCARA arm since this is the position where the end effector is in the centre
    // if theta1 is 0
    AxisPosDataType _originTheta2OffsetDegrees = 180;

    // Tolderance for check close to origin in mm
    static constexpr AxisPosDataType CLOSE_TO_ORIGIN_TOLERANCE_MM = 1;

};
