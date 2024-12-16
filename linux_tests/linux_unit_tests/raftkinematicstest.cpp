#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <vector>

#include "RaftCore.h"
#include "KinematicsSingleArmSCARA.h"

/// @brief Compute the forward kinematics of a 2R planar robot
/// @param theta1degs angle in degrees anticlockwise from x-axis to first link
/// @param theta2degs angle in degrees anticlockwise from x-axis to second link
/// @param l1 length of first link
/// @param l2 length of second link
/// @return x, y position of end effector
std::tuple<double, double> forwardKinematics(double theta1degs, double theta2degs, double l1, double l2, double theta2OffsetDegs) {
    double x = l1 * cos(AxisUtils::d2r(theta1degs)) + l2 * cos(AxisUtils::d2r(theta2degs + theta2OffsetDegs));
    double y = l1 * sin(AxisUtils::d2r(theta1degs)) + l2 * sin(AxisUtils::d2r(theta2degs + theta2OffsetDegs));
    printf("theta1 %.2f° cos(theta1) %.2f sin(theta1) %.2f theta2 %.2f° cos(theta2+180) %.2f sin(theta2+180) %.2f x1: %.2fmm x2: %.2fmm x1+x2 %.2fmm, y1: %.2fmm y2: %.2fmm y1+y2 %.2fmm\n",
                theta1degs, cos(AxisUtils::d2r(theta1degs)), sin(AxisUtils::d2r(theta1degs)),
                theta2degs, cos(AxisUtils::d2r(theta2degs + theta2OffsetDegs)), sin(AxisUtils::d2r(theta2degs + theta2OffsetDegs)),
                l1 * cos(AxisUtils::d2r(theta1degs)), l2 * cos(AxisUtils::d2r(theta2degs + theta2OffsetDegs)),
                x, 
                l1 * sin(AxisUtils::d2r(theta1degs)), l2 * sin(AxisUtils::d2r(theta2degs + theta2OffsetDegs)),
                y);
    return std::make_tuple(x, y);
}

int main()
{
    // Read JSON file testAxesDefinition.json into a string
    std::ifstream jsonAxesDefinition("testAxesDefinition.json");
    if (!jsonAxesDefinition.is_open())
    {
        std::cerr << "Failed to open JSON file" << std::endl;
        return 1;
    }
    std::string jsonStr((std::istreambuf_iterator<char>(jsonAxesDefinition)), std::istreambuf_iterator<char>());
    RaftJson config(jsonStr.c_str());

    // Create AxesParams object and setup axes
    AxesParams axesParams;
    axesParams.setupAxes(config);

    // Show params
    axesParams.debugLog();

    // Get motion section of config
    RaftJsonPrefixed motionConfig(config, "motion");

    // Kinematics object
    KinematicsSingleArmSCARA kinematics(motionConfig);
    AxisPosDataType l1, l2;
    kinematics.getArmLengths(l1, l2);
    AxisPosDataType maxRadiusMM = kinematics.getMaxRadiusMM();

    // Test points
    std::vector<std::vector<AxisPosDataType>> testPoints;
    testPoints.push_back({0, 0});
    testPoints.push_back({l1, l1});
    testPoints.push_back({l1, 0});
    testPoints.push_back({0, l1});
    testPoints.push_back({maxRadiusMM, 0});
    testPoints.push_back({0, 0});
    testPoints.push_back({maxRadiusMM/2, maxRadiusMM/2});
    testPoints.push_back({-maxRadiusMM/2, maxRadiusMM/2});
    testPoints.push_back({maxRadiusMM/2, -maxRadiusMM/2});
    testPoints.push_back({-maxRadiusMM/2, -maxRadiusMM/2});
    testPoints.push_back({0, maxRadiusMM});
    testPoints.push_back({0, -maxRadiusMM});
    testPoints.push_back({maxRadiusMM/2, -maxRadiusMM/2});
    testPoints.push_back({-maxRadiusMM/2, -maxRadiusMM/2});
    testPoints.push_back({0, maxRadiusMM});

    // Current position
    AxesState axesState;

    // Run the points through the forward and inverse kinematics and compute RMS error
    for (uint32_t i = 0; i < testPoints.size(); i++)
    {
        // RaftKinematics
        AxesValues<AxisPosDataType> inputValues = {testPoints[i][0], testPoints[i][1]};
        AxesValues<AxisStepsDataType> outputValues;
        bool rslt = kinematics.ptToActuator(inputValues, outputValues, axesState, axesParams, false, true);
        if (!rslt)
        {
            printf("Inverse kinematics failed for point %d at %.2f, %.2f\n", i, testPoints[i][0], testPoints[i][1]);
            continue;
        }
        // printf("Raft inverse kinematics ok %d at %.2f, %.2f actuator %d, %d\n", i, testPoints[i][0], testPoints[i][1], outputValues.getVal(0), outputValues.getVal(1));

        // Store the current position in both cartesian and actuator coordinates
        axesState.setPosition(inputValues, outputValues, false);

        // Convert steps to angles
        AxesValues<AxisCalcDataType> anglesDegrees;
        for (uint32_t j = 0; j < outputValues.numAxes(); j++)
        {
            anglesDegrees.setVal(j, outputValues.getVal(j) * 360.0 / axesParams.getStepsPerRot(j));
        }
        printf("Raft inverse kinematics ok %d at %.2f, %.2f actuator %d, %d angles %.2f°, %.2f°\n", i, testPoints[i][0], testPoints[i][1], outputValues.getVal(0), outputValues.getVal(1), anglesDegrees.getVal(0), anglesDegrees.getVal(1));

        // Compute forward kinematics to test
        std::tuple<double, double> fwdKinematics = forwardKinematics(anglesDegrees.getVal(0), anglesDegrees.getVal(1), l1, l2, kinematics.getOriginTheta2OffsetDegrees());

        // Calculate RMS error
        double forwardX = std::get<0>(fwdKinematics);
        double forwardY = std::get<1>(fwdKinematics);
        double rmsError = sqrt(pow(testPoints[i][0]-forwardX, 2) + pow(testPoints[i][1]-forwardY, 2));
        static double MAX_RMS_ERROR = 0.1;

        printf("Raft forwardKinematics %s steps1 %d steps2 %d -> x %.2f y %.2f errorX %.2f errorY %.2f RMSError %.2fmm\n",
                rmsError < MAX_RMS_ERROR ? "OK" : "!!! IMPRECISE RESULT !!!",
                outputValues.getVal(0), outputValues.getVal(1),
                forwardX, forwardY,
                testPoints[i][0]-forwardX, testPoints[i][1]-forwardY,
                rmsError);


        // printf("Raft inv-kinematics RMSError %.2fmm x %.2f (%.2f) y %.2f (%.2f) steps %d, %d\n", 
        //             sqrt(pow(testPoints[i][0]-std::get<0>(fwdKinematics), 2) + pow(testPoints[i][1]-std::get<1>(fwdKinematics), 2)),
        //             testPoints[i][0], std::get<0>(fwdKinematics),
        //             testPoints[i][1], std::get<1>(fwdKinematics),
        //             outputValues.getVal(0), outputValues.getVal(1));
    }
}
