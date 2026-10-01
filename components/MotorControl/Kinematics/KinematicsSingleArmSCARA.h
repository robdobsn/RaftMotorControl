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
// #define DEBUG_KINEMATICS_IK_FLIP

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
    /// @param args Motion arguments (includes out of bounds action)
    /// @return false if out of bounds or invalid
    virtual bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                              AxesValues<AxisStepsDataType>& outActuator,
                              const AxesState& curAxesState,
                              const AxesParams& axesParams,
                              OutOfBoundsAction outOfBoundsAction) const override final
    {
        // Convert the current position to angles wrapped 0..360 degrees
        AxesValues<AxisCalcDataType> curAngles;
        calculateAngles(curAxesState, curAngles, axesParams);

#ifdef DEBUG_KINEMATICS_SA_SCARA
        // Debug show the requested poing and the current position (in units from origin and steps from origin)
        LOG_I(MODULE_PREFIX, "ptToActuator: target X %.2f Y %.2f, curPos X %.2f Y %.2f, curSteps X %d Y %d, curAngles theta1 %.2f theta2 %.2f",
                targetPt.getVal(0), targetPt.getVal(1),
                curAxesState.getUnitsFromOrigin(0), curAxesState.getUnitsFromOrigin(1),
                curAxesState.getStepsFromOrigin(0), curAxesState.getStepsFromOrigin(1),
                curAngles.getVal(0), curAngles.getVal(1));
#endif

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
    		LOG_I(MODULE_PREFIX, "ptToActuator CLOSE_TO_ORIGIN best angles theta1 %.2f (diff %.2f) theta2 %.2f (diff %.2f)", 
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
                LOG_W(MODULE_PREFIX, "ptToActuator OUT_OF_BOUNDS x %.2f y %.2f", targetPt.getVal(0), targetPt.getVal(1));
#endif
                return false;
            }

            // Choose the solution needing the least TOTAL joint movement.
            //
            // This previously compared theta1 ONLY, which caused violent elbow
            // slams. The two SCARA solutions are mirrored about the line to the
            // target, so their theta1 values are often nearly equidistant from
            // the current pose - and when they are, floating-point noise decides
            // the winner while the two theta2 values sit up to 180 deg apart.
            // theta1 stayed smooth precisely because it was the quantity being
            // minimised, so the fault presented as the elbow alone banging
            // between branches.
            //
            // Measured on 2026-09-30 with event-triggered capture in
            // MotionPlanner: fifteen consecutive blocks at ~4 deg of joint
            // motion for ~8 mm of tool motion, then a single block demanding
            // 109-184 deg for the same ~8 mm, repeatedly, mid-workspace
            // (r = 27-134 mm, nowhere near a singularity). Audible as rapid
            // clunking; on one occasion it cost the machine its homing.
            //
            // Summing both axes makes the comparison reflect what the arm
            // actually has to do. Each relative angle is computed once here and
            // reused, so this is no more expensive than the version it replaces.
            double rel1Axis0 = computeRelativeAngle(soln1.getVal(0), curAngles.getVal(0));
            double rel1Axis1 = computeRelativeAngle(soln1.getVal(1), curAngles.getVal(1));
            double rel2Axis0 = computeRelativeAngle(soln2.getVal(0), curAngles.getVal(0));
            double rel2Axis1 = computeRelativeAngle(soln2.getVal(1), curAngles.getVal(1));
            double cost1 = fabs(rel1Axis0) + fabs(rel1Axis1);
            double cost2 = fabs(rel2Axis0) + fabs(rel2Axis1);

            bool useSoln1 = (cost1 <= cost2);
            if (_preferAlternateSolution)
                useSoln1 = !useSoln1;  // Swap to alternate solution

#ifdef DEBUG_KINEMATICS_IK_FLIP
            // Log the DECISION, not its aftermath, whenever the chosen solution
            // demands a large joint move. Everything needed to apportion blame
            // is in scope here and nowhere else:
            //
            //   chosen cost >  rejected cost  -> the _preferAlternateSolution
            //       inversion overrode a good choice; blame the caller that
            //       set the flag
            //   chosen cost <= rejected cost  -> both solutions are far from
            //       curAngles, so the REFERENCE is wrong, not the choice
            //
            // Earlier attempts guessed between those two and were wrong twice.
            // Rate-limited because a sustained fault would otherwise flood the
            // console; the logger is non-blocking so excess output is dropped
            // rather than stalling motion.
            {
                double chosenCost = useSoln1 ? cost1 : cost2;
                double rejectCost = useSoln1 ? cost2 : cost1;
                static uint32_t s_ikFlipLastMs = 0;
                if ((chosenCost > 40.0) &&
                    ((uint32_t)(millis() - s_ikFlipLastMs) > 2000))
                {
                    s_ikFlipLastMs = millis();
                    LOG_W(MODULE_PREFIX,
                        "IKFLIP tgt=%.1f,%.1f cur=(%.1f,%.1f) s1=(%.1f,%.1f)c=%.1f "
                        "s2=(%.1f,%.1f)c=%.1f chose=%d prefAlt=%d chosenCost=%.1f rejCost=%.1f %s",
                        targetPt.getVal(0), targetPt.getVal(1),
                        curAngles.getVal(0), curAngles.getVal(1),
                        soln1.getVal(0), soln1.getVal(1), cost1,
                        soln2.getVal(0), soln2.getVal(1), cost2,
                        useSoln1 ? 1 : 2, _preferAlternateSolution ? 1 : 0,
                        chosenCost, rejectCost,
                        (chosenCost > rejectCost) ? "INVERTED" : "BOTH-FAR");
                }
            }
#endif

            if (useSoln1) {
                relativeAngleSolution = {
                    AxisCalcDataType(rel1Axis0), AxisCalcDataType(rel1Axis1)
                };
            } else {
                relativeAngleSolution = {
                    AxisCalcDataType(rel2Axis0), AxisCalcDataType(rel2Axis1)
                };
            }

#ifdef DEBUG_KINEMATICS_SA_SCARA
            LOG_I(MODULE_PREFIX, "ptToActuator ANGLES CHOSEN: (%.2f, %.2f) ... ALTERNATIVE (%.2f, %.2f) CURRENT (%.2f, %.2f)", 
                useSoln1 ? soln1.getVal(0) : soln2.getVal(0), useSoln1 ? soln1.getVal(1) : soln2.getVal(1),
                useSoln1 ? soln2.getVal(0) : soln1.getVal(0), useSoln1 ? soln2.getVal(1) : soln1.getVal(1),
                curAngles.getVal(0), curAngles.getVal(1));
#endif
        }

        // Calculate required steps (relative to the current position)
        relativeAnglesToAbsoluteSteps(relativeAngleSolution, curAxesState, outActuator, axesParams);

#ifdef DEBUG_KINEMATICS_SA_SCARA
            LOG_I(MODULE_PREFIX, "ptToActuator REL ANGLE: (%.2f, %.2f), ABS_STEPS (%d, %d)", 
                relativeAngleSolution.getVal(0), relativeAngleSolution.getVal(1),
                outActuator.getVal(0), outActuator.getVal(1));
#endif

//         // Debug
// #ifdef DEBUG_KINEMATICS_SA_SCARA
//         LOG_I(MODULE_PREFIX, "ptToActuator TARGET POS (%.2f, %.2f) TARGET FROM ORIGIN (%.2f, %.2f) DIST %.2f ABS STEPS (%d, %d) NIN ROTATION (%.2f, %.2f)", 
//                 targetPt.getVal(0), targetPt.getVal(1),
//                 curAxesState.getUnitsFromOrigin(0), curAxesState.getUnitsFromOrigin(1),
//                 sqrt(pow(targetPt.getVal(0) - curAxesState.getUnitsFromOrigin(0), 2) + pow(targetPt.getVal(1) - curAxesState.getUnitsFromOrigin(1), 2)),
//                 outActuator.getVal(0), outActuator.getVal(1),
//                 relativeAngleSolution.getVal(0), relativeAngleSolution.getVal(1));
// #endif
// #ifdef DEBUG_KINEMATICS_SA_SCARA
//     LOG_I(MODULE_PREFIX, "ptToActuator: curAngles theta1 %.2f theta2 %.2f, chosen relAngle0 %.2f relAngle1 %.2f", curAngles.getVal(0), curAngles.getVal(1), relativeAngleSolution.getVal(0), relativeAngleSolution.getVal(1));
// #endif
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
        // Convert steps to angles - NOTE: Must use inActuator parameter, not curAxesState!
        // This ensures we calculate position for the actual current step count from the motors
        AxesValues<AxisCalcDataType> curAngles;
        calculateAnglesFromSteps(inActuator, curAngles, axesParams);

        // Calculate the x and y coordinates using forward kinematics
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

    /// @brief Get Cartesian workspace bounds for proportionate coordinate conversion
    /// @param axisIdx Axis index (0=X, 1=Y)
    /// @param minVal Output minimum value for this axis
    /// @param maxVal Output maximum value for this axis
    /// @param axesParams Axes parameters (unused for SCARA - bounds come from arm geometry)
    /// @return true if bounds are available for this axis
    /// @note For SCARA, the Cartesian workspace is a circle centered at origin with radius maxRadiusMM
    ///       So both X and Y range from -maxRadiusMM to +maxRadiusMM
    virtual bool getCartesianWorkspaceBounds(uint32_t axisIdx, 
                                             AxisPosDataType& minVal, 
                                             AxisPosDataType& maxVal,
                                             const AxesParams& axesParams) const override
    {
        (void)axesParams; // Unused for SCARA - bounds come from arm geometry
        
        // For SCARA, X and Y both range from -maxRadiusMM to +maxRadiusMM
        if (axisIdx <= 1)
        {
            minVal = -_maxRadiusMM;
            maxVal = _maxRadiusMM;
            return true;
        }
        // Other axes not supported
        return false;
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

        // Check validity of position. Allow a small reach tolerance: the arm physically
        // reaches arm1+arm2, so rounding (or a THR rho==1.0 landing exactly on the boundary)
        // must not spuriously reject a reachable rim point. AxisUtils::cosineRule clamps the
        // law-of-cosines argument to [-1,1], so a marginally-over point resolves to the
        // fully-extended (rim) pose rather than failing.
        const AxisCalcDataType reachTolMM = 0.5;
        bool posValid = thirdSideL3MM <= (_arm1LenMM + _arm2LenMM + reachTolMM) &&
                        (thirdSideL3MM >= fabs(_arm1LenMM - _arm2LenMM)) &&
                        thirdSideL3MM <= _maxRadiusMM + reachTolMM;

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
        LOG_I(MODULE_PREFIX, "cartesianToPolar INPUT X%.2f Y%.2f -> soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f, targetAngleRads %.2f thirdSideL3MM %.2f posValid %d", 
                    targetPt.getVal(0), targetPt.getVal(1), targetSoln1.getVal(0), targetSoln1.getVal(1), targetSoln2.getVal(0), targetSoln2.getVal(1),
                    targetAngleRads, thirdSideL3MM, posValid);

        if (fabs(targetPt.getVal(0) - 100.0) < 1e-2 && fabs(targetPt.getVal(1)) < 1e-2) {
            LOG_I(MODULE_PREFIX, "TEST (100,0): soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f (expected: theta1=0, theta2=180)", 
                    targetSoln1.getVal(0), targetSoln1.getVal(1), targetSoln2.getVal(0), targetSoln2.getVal(1));
        }
        if (fabs(targetPt.getVal(0)) < 1e-2 && fabs(targetPt.getVal(1) - 100.0) < 1e-2) {
            LOG_I(MODULE_PREFIX, "TEST (0,100): soln1 theta1 %.2f theta2 %.2f, soln2 theta1 %.2f theta2 %.2f (expected: theta1=90, theta2=90)", 
                    targetSoln1.getVal(0), targetSoln1.getVal(1), targetSoln2.getVal(0), targetSoln2.getVal(1));
        }
#endif

        return posValid;        
    }

    /// @brief Calculate angles from step values directly
    /// @param stepValues Step values for each axis
    /// @param anglesDegrees Output angles in degrees
    /// @param axesParams Axes parameters
    void calculateAnglesFromSteps(const AxesValues<AxisStepsDataType>& stepValues,
                AxesValues<AxisCalcDataType>& anglesDegrees, 
                const AxesParams& axesParams) const
    {
        // All angles returned are in degrees anticlockwise from the x-axis
        //
        // NOTE: adding homeOffsetSteps here (to correct the end-stop-midpoint
        // vs geometric-zero difference) was tried and REVERTED 2026-09-17.
        //
        // It measured WELL on static poses - 0.84 mm residual over a 32-pose
        // grid against a 1.44 mm baseline, every pose settling - but in the
        // full regression the arm drove continuously and every L1 pose failed
        // with "settle timeout".
        //
        // CAUSE NOW KNOWN - see the floating-point note below and 2026-10-01.
        // The thrashing described here was the SAME fault: this function
        // returned reference angles truncated to whole degrees (integer
        // division), which made the IK branch-selection costs tie and the
        // elbow choice arbitrary. With that fixed, chatter over an identical
        // 55-minute sequence fell from 228 samples to 2, and the survivors are
        // at r=1.3-3.1mm where the two solutions genuinely converge.
        //
        // THEREFORE: re-testing homeOffsetSteps is now worthwhile. It was
        // abandoned because of thrashing that is no longer present. Re-enable
        // behind the instrumentation (DEBUG_KINEMATICS_IK_FLIP) and watch for
        // ties rather than relying on static pose accuracy, which passed while
        // motion was broken.
        //
        // The two explanations proposed at the time were both wrong:
        //   1. "the forward/inverse round trip is not self-consistent" - it is;
        //      calculateAngles() feeds ptToActuator, and the relative-angle
        //      arithmetic cancels the offset exactly.
        //   2. "the split path tracks units while the offset is in steps, so
        //      they diverge" - simulated on the host replicating this
        //      arithmetic (blockMotionVector, per-block ptToActuator, step
        //      accumulation, setPosition): final error 0.025 mm independent of
        //      block count, and 0.02-0.04 mm over five chained moves. No drift.
        //
        // Candidates listed at the time: the CLOSE_TO_ORIGIN branch, out-of-
        // bounds handling, and alternate-solution selection. The last of these
        // was closest - the elbow WAS flipping between solutions - but the
        // trigger was not the selection logic itself. Instrumentation showed
        // _preferAlternateSolution was 0 in all 16 captured events, so the
        // alternate-solution path never participated; the reference angles fed
        // into the comparison were simply too coarse to discriminate.
        //
        // Do NOT re-apply without first instrumenting ptToActuator to log
        // target/current/relative angles during a failing move. Static
        // positioning accuracy is NOT a sufficient acceptance test - it passed
        // while motion was broken.
        // STATUS 2026-09-18: the offset fixes large-radius accuracy (0.773 mm
        // over a 32-pose grid at r=60..150, 32/32 split moves settling, vs
        // 3.08 mm without) but the arm THRASHES at small radius: L1 sweeps
        // r=30 and every pose there failed with "settle timeout". Near the
        // folded pose the two IK solutions converge
        // (ptToActuator logged "CHOSEN (338.83, 83.10) ALTERNATIVE (83.10,
        // 338.83)"), so successive split blocks may be flipping between
        // elbow-up and elbow-down - SUSPECTED, not confirmed.
        //
        // CONFIRMED 2026-10-01: they were flipping. Note the logged pair is
        // (a,b) and (b,a) - that is correct for equal-length links, where
        // theta1 = phi +/- acos(r/2L) and theta2 mirrors it, NOT a sign of a
        // bad alternate solution. The flipping was caused by truncated
        // reference angles, since fixed.
        //
        // homeOffsetSteps: homing parks on the end-stop MIDPOINT, which is a
        // per-axis calibration distance from the arm's geometric zero. Applied
        // here so reported position stays 0 at home while the kinematics uses
        // true angles. REQUIRES MotionControlIF::syncUnitsFromSteps() after
        // homing - without it the tracked Cartesian position and the step
        // count describe different places and split moves slam on block one.
        // OFFSET DISABLED pending the telemetry fix (see below). Re-enable by
        // restoring the getHomeOffsetSteps() terms - but SandBot.cpp's
        // getPublishJSON must apply the same offset first, or the host sees a
        // position the arm will never report reaching.
        // homeOffsetSteps is NOT applied here. It means "distance from the
        // end-stop midpoint to the park position that puts the EE at bed
        // centre", and is applied by HomingSeekCenter when choosing where to
        // park. Applying it here as an angle correction as well would
        // double-count it.
        // FLOATING POINT, deliberately. Both operands were int32_t
        // (AxisStepsDataType), so `steps * 360 / stepsPerRot` was INTEGER
        // division with two distinct failure modes, diagnosed 2026-10-01:
        //
        // 1. TRUNCATION. The reference angle was rounded to whole degrees -
        //    every logged value ended .0 - against a true step resolution of
        //    0.009 deg. These angles pick the IK elbow branch by comparing
        //    which solution is nearer the current pose, and at 1 deg
        //    granularity that comparison TIES: captured with both candidate
        //    solutions costing exactly 111.0 deg. A tie makes the choice
        //    arbitrary, and choosing wrongly flips the elbow ~110-180 deg.
        //
        // 2. OVERFLOW. steps * 360 exceeds int32 above 5,965,232 steps =
        //    155 revolutions. A single bed_wipe winds the shoulder 63.4
        //    revolutions, so two wipes plus patterns reach that threshold -
        //    which is why the fault only ever appeared deep into long runs
        //    and never in short reproducers.
        //
        // Symptom: single motion blocks demanding 107-184 deg of joint travel
        // for 2-9 mm of tool travel, audible as violent clunking, and on one
        // occasion the machine lost homing.
        AxisCalcDataType theta1Degrees = AxisUtils::wrapDegrees(
                    double(stepValues.getVal(0)) * 360.0 / double(axesParams.getStepsPerRot(0)));
        AxisCalcDataType theta2Degrees = AxisUtils::wrapDegrees(
                    double(stepValues.getVal(1)) * 360.0 / double(axesParams.getStepsPerRot(1))
                    + _originTheta2OffsetDegrees);
        anglesDegrees = { theta1Degrees, theta2Degrees };
#ifdef DEBUG_KINEMATICS_SA_SCARA
        LOG_I(MODULE_PREFIX, "calculateAnglesFromSteps steps (%d, %d) angles (%.2f°, %.2f°)",
                stepValues.getVal(0), stepValues.getVal(1), anglesDegrees.getVal(0), anglesDegrees.getVal(1));
#endif        
    }

    /// @brief Calculate the current axis angles (wrapper for backwards compatibility)
    /// @param curAxesState Current axes state (includes current position in steps from origin)
    /// @param anglesDegrees Output angles in degrees
    /// @param axesParams Axes parameters
    void calculateAngles(const AxesState& curAxesState,
                AxesValues<AxisCalcDataType>& anglesDegrees, 
                const AxesParams& axesParams) const
    {
        AxesValues<AxisStepsDataType> stepValues;
        stepValues.setVal(0, curAxesState.getStepsFromOrigin(0));
        stepValues.setVal(1, curAxesState.getStepsFromOrigin(1));
        calculateAnglesFromSteps(stepValues, anglesDegrees, axesParams);
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
        LOG_I(MODULE_PREFIX, "relAnglesToAbsSteps relAngle (%.2f, %.2f) relSteps (%d, %d) curSteps (%d, %d) absSteps (%d, %d)",
                relativeAngles.getVal(0), relativeAngles.getVal(1), 
                stepsRel0, stepsRel1, 
                curAxesState.getStepsFromOrigin(0), curAxesState.getStepsFromOrigin(1), 
                outActuator.getVal(0), outActuator.getVal(1));
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

    // Solution preference for inverse kinematics
    // When true, prefers the alternate IK solution (used for path planning to avoid invalid intermediate points)
    mutable bool _preferAlternateSolution = false;

public:
    /// @brief Check if this kinematics supports alternate IK solutions
    /// @return true (SCARA has elbow-up/down solutions)
    virtual bool supportsAlternateSolutions() const override
    {
        return true;
    }

    /// @brief Set solution preference for inverse kinematics
    /// @param prefer True to prefer alternate solution (elbow-up vs elbow-down)
    virtual void setPreferAlternateSolution(bool prefer) const override
    {
        _preferAlternateSolution = prefer;
    }

    /// @brief Get current solution preference
    /// @return True if preferring alternate solution
    virtual bool getPreferAlternateSolution() const override
    {
        return _preferAlternateSolution;
    }

    /// @brief Validate that all intermediate points in a linear path are reachable
    /// @param startPt Start point in Cartesian coordinates
    /// @param endPt End point in Cartesian coordinates
    /// @param numSegments Number of segments to test along the path
    /// @param curAxesState Current axes state
    /// @param axesParams Axes parameters
    /// @return true if all intermediate points are reachable
    virtual bool validateLinearPath(const AxesValues<AxisPosDataType>& startPt,
                                   const AxesValues<AxisPosDataType>& endPt,
                                   uint32_t numSegments,
                                   const AxesState& curAxesState,
                                   const AxesParams& axesParams) const override
    {
        if (numSegments == 0)
            return true;

        // Calculate delta per segment
        AxesValues<AxisPosDataType> delta;
        for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
            delta.setVal(i, (endPt.getVal(i) - startPt.getVal(i)) / double(numSegments));

        // Test each intermediate point
        AxesState testState = curAxesState;
        for (uint32_t seg = 1; seg <= numSegments; seg++)
        {
            // Calculate test point
            AxesValues<AxisPosDataType> testPt;
            for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
                testPt.setVal(i, startPt.getVal(i) + delta.getVal(i) * seg);

            // Try inverse kinematics
            AxesValues<AxisStepsDataType> actuatorCoords;
            bool valid = ptToActuator(testPt, actuatorCoords, testState, axesParams, OutOfBoundsAction::ALLOW);
            
            if (!valid)
            {
                return false;
            }

            // Update test state for next iteration (to simulate sequential moves)
            testState.setPosition(testPt, actuatorCoords, false);
        }

        return true;
    }




};
