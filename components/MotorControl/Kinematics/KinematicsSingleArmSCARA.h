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
        _arm1LenMM = AxisPosDataType(config.getDouble("arm1LenMM", 100));
        _arm2LenMM = AxisPosDataType(config.getDouble("arm2LenMM", 100));
        _maxRadiusMM = AxisPosDataType(config.getDouble("maxRadiusMM", _arm1LenMM + _arm2LenMM));
        _originTheta2OffsetDegrees = AxisPosDataType(config.getDouble("originTheta2OffsetDegrees", 180));

#ifdef DEBUG_KINEMATICS_SA_SCARA_SETUP
        LOG_I(MODULE_PREFIX, "arm1LenMM %.2f arm2LenMM %.2f maxRadiusMM %.2f",
                _arm1LenMM, _arm2LenMM, _maxRadiusMM);
#endif
    }

    /// @brief Convert a point in cartesian to actuator steps
    /// @param targetPt Target point cartesian from origin
    /// @param outActuator Output actuator in absolute steps from origin
    /// @param curAxesState Current position (in both units and steps from origin)
    /// @param axesParams Axes parameters
    /// @param constrainToBounds Constrain out of bounds (if not constrained then return false if the point is out of bounds)
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                              AxesValues<AxisStepsDataType>& outActuator,
                              const AxesState& curAxesState,
                              const AxesParams& axesParams,
                              bool constrainToBounds) const override final
    {
        // Convert the current position to angles wrapped 0..360 degrees
        AxesValues<AxisCalcDataType> curAngles;
        calculateAngles(curAxesState, curAngles, axesParams);

        // Absolute and relative angle solutions
        AxesValues<AxisCalcDataType> absoluteAngleSolution;
        AxesValues<AxisCalcDataType> relativeAngleSolution;

        // Check for points close to the origin
        if (AxisUtils::isApprox(targetPt.getVal(0), 0, CLOSE_TO_ORIGIN_TOLERANCE_MM) && (AxisUtils::isApprox(targetPt.getVal(1), 0, CLOSE_TO_ORIGIN_TOLERANCE_MM)))
        {
            // Close to the origin is a special case where the arm is straight and the end effector are in the centre
            // Keep the current position for theta1, set theta2 to theta1+_originTheta2OffsetDegrees (i.e. doubled-back so end-effector is in centre)
            absoluteAngleSolution = { curAngles.getVal(0), curAngles.getVal(0) + _originTheta2OffsetDegrees };
            relativeAngleSolution = { 0, computeRelativeAngle(absoluteAngleSolution.getVal(1), curAngles.getVal(1)) };
#ifdef DEBUG_KINEMATICS_SA_SCARA
    		LOG_I(MODULE_PREFIX, "ptToActuator x %.2f y %.2f close to origin best angles theta1 %.2f (diff %.2f) theta2 %.2f (diff %.2f)", 
                            targetPt.getVal(0), targetPt.getVal(1), 
                            absoluteAngleSolution.getVal(0), relativeAngleSolution.getVal(0),
                            absoluteAngleSolution.getVal(1), relativeAngleSolution.getVal(1));
#endif
        }
        else
        {
            // Convert the target cartesian coords to polar wrapped to 0..360 degrees
            AxesValues<AxisCalcDataType> soln1, soln2;
            bool isValid = cartesianToPolar(targetPt, soln1, soln2, axesParams);
            if (!isValid)
            {
#ifdef WARN_KINEMATICS_SA_SCARA_POS_OUT_OF_BOUNDS
                LOG_W(MODULE_PREFIX, "ptToActuator out of bounds x %.2f y %.2f", targetPt.getVal(0), targetPt.getVal(1));
#endif
                return false;
            }

            // Choose the solution whose theta1 is closest to the current theta1
            double diff1 = fabs(computeRelativeAngle(soln1.getVal(0), curAngles.getVal(0)));
            double diff2 = fabs(computeRelativeAngle(soln2.getVal(0), curAngles.getVal(0)));
            if (diff1 < diff2) {
                relativeAngleSolution = {
                    computeRelativeAngle(soln1.getVal(0), curAngles.getVal(0)),
                    computeRelativeAngle(soln1.getVal(1), curAngles.getVal(1))
                };
            } else {
                relativeAngleSolution = {
                    computeRelativeAngle(soln2.getVal(0), curAngles.getVal(0)),
                    computeRelativeAngle(soln2.getVal(1), curAngles.getVal(1))
                };
            }

#ifdef DEBUG_KINEMATICS_SA_SCARA
            LOG_I(MODULE_PREFIX, "ptToActuator: target X %.2f Y %.2f, soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f, chosen relAngle0 %.2f relAngle1 %.2f, absSteps0 %d absSteps1 %d, curAngles theta1 %.2f theta2 %.2f", 
                targetPt.getVal(0), targetPt.getVal(1),
                soln1.getVal(0), soln1.getVal(1),
                soln2.getVal(0), soln2.getVal(1),
                relativeAngleSolution.getVal(0), relativeAngleSolution.getVal(1),
                outActuator.getVal(0), outActuator.getVal(1),
                curAngles.getVal(0), curAngles.getVal(1));
#endif

        }

        // Apply this to calculate required steps (relative to the current position)
        relativeAnglesToAbsoluteSteps(relativeAngleSolution, curAxesState, outActuator, axesParams);

        // Debug
#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "ptToActuator X %.2f (%.2f) Y %.2f (%.2f) dist %.2f abs steps %d %d minRot1 %.2f minRot2 %.2f", 
                targetPt.getVal(0), targetPt.getVal(1),
                curAxesState.getUnitsFromOrigin(0), curAxesState.getUnitsFromOrigin(1),
                sqrt(pow(targetPt.getVal(0) - curAxesState.getUnitsFromOrigin(0), 2) + pow(targetPt.getVal(1) - curAxesState.getUnitsFromOrigin(1), 2)),
                outActuator.getVal(0), outActuator.getVal(1),
                relativeAngleSolution.getVal(0), relativeAngleSolution.getVal(1));
#endif
#ifdef DEBUG_KINEMATICS_SA_SCARA
    LOG_I(MODULE_PREFIX, "ptToActuator: curAngles theta1 %.2f theta2 %.2f, chosen relAngle0 %.2f relAngle1 %.2f", curAngles.getVal(0), curAngles.getVal(1), relativeAngleSolution.getVal(0), relativeAngleSolution.getVal(1));
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
        // Convert to angles
        AxesValues<AxisCalcDataType> curAngles;
        calculateAngles(curAxesState, curAngles, axesParams);

        // Calculate the x and y coordinates
        outPt = { (AxisPosDataType) (_arm1LenMM * cos(AxisUtils::d2r(curAngles.getVal(0))) + _arm2LenMM * cos(AxisUtils::d2r(curAngles.getVal(1)))),
                    (AxisPosDataType) (_arm1LenMM * sin(AxisUtils::d2r(curAngles.getVal(0))) + _arm2LenMM * sin(AxisUtils::d2r(curAngles.getVal(1))) )};

#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "actuatorToPt steps %d, %d x %.2f y %.2f theta1 %.2f theta2 %.2f",
                inActuator.getVal(0), inActuator.getVal(1), outPt.getVal(0), outPt.getVal(1), curAngles.getVal(0), curAngles.getVal(1));
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

        // Calculate angle from x-axis to target point
        AxisCalcDataType targetAngleRads = atan2(targetPt.getVal(1), targetPt.getVal(0));

        // Calculate angle of triangle opposite L2
        AxisCalcDataType a2 = AxisUtils::cosineRule(thirdSideL3MM, _arm1LenMM, _arm2LenMM);

        // Calculate angle of triangle opposite side 3 (neither L1 nor L2)
        AxisCalcDataType a3 = AxisUtils::cosineRule(_arm1LenMM, _arm2LenMM, thirdSideL3MM);

        // Calculate the alpha and beta angles in degrees
        targetSoln1 = { AxisUtils::r2d(targetAngleRads + a2), AxisUtils::r2d(-M_PI + targetAngleRads + a2 + a3) };
        targetSoln2 = { AxisUtils::r2d(targetAngleRads - a2), AxisUtils::r2d(M_PI + targetAngleRads - a2 - a3) };

#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "cartesianToPolar INPUT X%.2f Y%.2f -> soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f", targetPt.getVal(0), targetPt.getVal(1), targetSoln1.getVal(0), targetSoln1.getVal(1), targetSoln2.getVal(0), targetSoln2.getVal(1));
#endif

        if (fabs(targetPt.getVal(0) - 100.0) < 1e-2 && fabs(targetPt.getVal(1)) < 1e-2) {
            LOG_I(MODULE_PREFIX, "TEST (100,0): soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f (expected: theta1=0, theta2=180)", targetSoln1.getVal(0), targetSoln1.getVal(1), targetSoln2.getVal(0), targetSoln2.getVal(1));
        }
        if (fabs(targetPt.getVal(0)) < 1e-2 && fabs(targetPt.getVal(1) - 100.0) < 1e-2) {
            LOG_I(MODULE_PREFIX, "TEST (0,100): soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f (expected: theta1=90, theta2=90)", targetSoln1.getVal(0), targetSoln1.getVal(1), targetSoln2.getVal(0), targetSoln2.getVal(1));
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
        AxisCalcDataType theta1Degrees = AxisUtils::wrapDegrees(curAxesState.getStepsFromOrigin(0) * 360 / axesParams.getStepsPerRot(0));
        AxisCalcDataType theta2Degrees = AxisUtils::wrapDegrees(curAxesState.getStepsFromOrigin(1) * 360 / axesParams.getStepsPerRot(1) + _originTheta2OffsetDegrees);
        anglesDegrees = { theta1Degrees, theta2Degrees };
#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "stepsToPolar ax0Steps %d ax1Steps %d a %.2fd b %.2fd",
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
            bestRotation = 360 + diffAngle;
        else if (diffAngle > 180)
            bestRotation = diffAngle - 360;
#ifdef DEBUG_KINEMATICS_SA_SCARA_RELATIVE_ANGLE
        LOG_I(MODULE_PREFIX, "computeRelativeAngle: target %.2f cur %.2f diff %.2f best %.2f",
                targetRotation, curRotation, diffAngle, bestRotation);
#endif
        return bestRotation;
    }

    /// @brief Convert relative angles to absolute steps
    /// @param relativeAngles Relative angles
    /// @param curAxesState Current axes state (position and origin status)
    /// @param outActuator Output actuator steps
    /// @param axesParams Axes parameters
    void relativeAnglesToAbsoluteSteps(const AxesValues<AxisCalcDataType>& relativeAngles, 
            const AxesState& curAxesState, 
            AxesValues<AxisStepsDataType>& outActuator, 
            const AxesParams& axesParams) const
    {
        // Convert relative polar to steps
        int32_t stepsRel0 = int32_t(roundf(relativeAngles.getVal(0) * axesParams.getStepsPerRot(0) / 360));
        int32_t stepsRel1 = int32_t(roundf(relativeAngles.getVal(1) * axesParams.getStepsPerRot(1) / 360));

        // Add to existing
        outActuator.setVal(0, curAxesState.getStepsFromOrigin(0) + stepsRel0);
        outActuator.setVal(1, curAxesState.getStepsFromOrigin(1) + stepsRel1);
#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "relativeAnglesToAbsoluteSteps [0] relAngle %.2f relSteps %d curSteps %d absSteps %d [1] relAngle %.2f relSteps %d curSteps %d absSteps %d",
                relativeAngles.getVal(0), stepsRel0, curAxesState.getStepsFromOrigin(0), outActuator.getVal(0),
                relativeAngles.getVal(1), stepsRel1, curAxesState.getStepsFromOrigin(1), outActuator.getVal(1));
#endif        
    }

    // Arm lengths in mm
    AxisPosDataType _arm1LenMM = 100;
    AxisPosDataType _arm2LenMM = 100;

    // Max radius in mm
    AxisPosDataType _maxRadiusMM = 200;

    // Origin theta2 offset in degrees (theta2 is the angle of the second arm anticlockwise from the x-axis)
    // 180 degrees is the default for a SCARA arm since this is the position where the end effector is in the centre
    // if theta1 is 0
    AxisPosDataType _originTheta2OffsetDegrees = 180;

    // Tolderance for check close to origin in mm
    static constexpr AxisPosDataType CLOSE_TO_ORIGIN_TOLERANCE_MM = 1;

};
