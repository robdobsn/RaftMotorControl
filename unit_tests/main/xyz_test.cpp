/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Unit test of XYZ geometry
//
// Rob Dobson 2020
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <limits.h>
#include "unity.h"
#include "AxisGeomXYZ.h"
#include "AxesPosition.h"
#include "AxesParams.h"

static const char* MODULE_PREFIX = "XYZUnitTest";

// Test points and expected results
struct TestGeomPt
{
    TestGeomPt(AxesPosValues pt_, AxesParamVals<AxisStepsDataType> actuator_)
        : pt(pt_), actuator(actuator_) {}
    AxesPosValues pt;
    AxesParamVals<AxisStepsDataType> actuator;
    AxesPosition curPos;
};

void testPtToActuator(AxisGeomXYZ& axisGeom, AxesParams& axesParams, TestGeomPt* pTestPts, int numTestPts)
{
    // Iterate through test points
    for (int testPtIdx = 0; testPtIdx < numTestPts; testPtIdx++)
    {
        // Get test point
        TestGeomPt& testPt = pTestPts[testPtIdx];

        // Convert point to actuator
        AxesParamVals<AxisStepsDataType> actuator;
        bool success = axisGeom.ptToActuator(testPt.pt, actuator, testPt.curPos, axesParams, false);
        TEST_ASSERT_MESSAGE(success, "ptToActuator failed");

        // Check actuator values
        TEST_ASSERT_MESSAGE(fabs(actuator.getVal(0) - testPt.actuator.getVal(0)) < 0.0001, "X actuator incorrect");
        TEST_ASSERT_MESSAGE(fabs(actuator.getVal(1) - testPt.actuator.getVal(1)) < 0.0001, "Y actuator incorrect");
        TEST_ASSERT_MESSAGE(fabs(actuator.getVal(2) - testPt.actuator.getVal(2)) < 0.0001, "Z actuator incorrect");
    }
}

TEST_CASE("XYZ test 1", "[Geometry]")
{
    LOG_I(MODULE_PREFIX, "XYZ test 1");
    
    // Create XYZ geometry
    AxisGeomXYZ axisGeomXYZ;

    // Create axes params
    AxesParams axesParams;
    const char* paramsStr =
        R"("axes": [
            {
                {
                    "name": "x",
                    "params": {
                        "unitsPerRot": 360,
                        "stepsPerRot": 200,
                        "maxSpeed": 10,
                        "maxAcc": 10
                    }
                },
                {
                    "name": "y",
                    "params": {
                        "unitsPerRot": 360,
                        "stepsPerRot": 200,
                        "maxSpeed": 10,
                        "maxAcc": 10
                    }
                },
                {
                    "name": "z",
                    "params": {
                        "unitsPerRot": 360,
                        "stepsPerRot": 200,
                        "maxSpeed": 10,
                        "maxAcc": 10
                    }
                }
            }
        ])";
    RaftJson axesParamsJson(paramsStr);
    axesParams.setupAxes(axesParamsJson);

    // Test points
    TestGeomPt testPts[] = {
        {{0, 0, 0}, {0, 0, 0}},
        {{1, 0, 0}, {1, 0, 0}},
        {{0, 1, 0}, {0, 1, 0}},
        {{0, 0, 1}, {0, 0, 1}},
        {{1, 1, 1}, {1, 1, 1}},
        {{-1, 0, 0}, {-1, 0, 0}},
        {{0, -1, 0}, {0, -1, 0}},
        {{0, 0, -1}, {0, 0, -1}},
        {{-1, -1, -1}, {-1, -1, -1}},
        {{1, 1, 1}, {1, 1, 1}},
        {{1, 2, 3}, {1, 2, 3}},
        {{-1, -2, -3}, {-1, -2, -3}},
        {{1.1, 2.2, 3.3}, {1, 2, 3}},
        {{-1.1, -2.2, -3.3}, {-1, -2, -3}},
        {{1.5, 2.5, 3.5}, {2, 3, 4}},
        {{-1.5, -2.5, -3.5}, {-2, -3, -4}},
    };

    testPtToActuator(axisGeomXYZ, axesParams, testPts, sizeof(testPts) / sizeof(TestGeomPt));

}
