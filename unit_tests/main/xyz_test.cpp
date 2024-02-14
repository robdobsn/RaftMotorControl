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

static const char* MODULE_PREFIX = "XYZUnitTest";

// Test points and expected results
struct TestGeomPt
{
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
        TEST_ASSERT_MESSAGE(fabs(actuator._axis[0] - testPt.actuator._axis[0]) < 0.0001, "X actuator incorrect");
        TEST_ASSERT_MESSAGE(fabs(actuator._axis[1] - testPt.actuator._axis[1]) < 0.0001, "Y actuator incorrect");
        TEST_ASSERT_MESSAGE(fabs(actuator._axis[2] - testPt.actuator._axis[2]) < 0.0001, "Z actuator incorrect");
    }
}
{
    TEST_ASSERT_MESSAGE(numLocalVars == 0, "Incorrect number of local vars");
}

TEST_CASE("XYZ test 1", "[Geometry]")
{
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
    axesParams.setupAxes(paramsStr);

    // Test points
    TestGeomPt testPts[] = {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{1, 0, 0}, {1, 0, 0}, {0, 0, 0}},
        {{0, 1, 0}, {0, 1, 0}, {0, 0, 0}},
        {{0, 0, 1}, {0, 0, 1}, {0, 0, 0}},
        {{1, 1, 1}, {1, 1, 1}, {0, 0, 0}},
        {{-1, 0, 0}, {-1, 0, 0}, {0, 0, 0}},
        {{0, -1, 0}, {0, -1, 0}, {0, 0, 0}},
        {{0, 0, -1}, {0, 0, -1}, {0, 0, 0}},
        {{-1, -1, -1}, {-1, -1, -1}, {0, 0, 0}},
        {{1, 1, 1}, {1, 1, 1}, {0, 0, 0}},
        {{1, 2, 3}, {1, 2, 3}, {0, 0, 0}},
        {{-1, -2, -3}, {-1, -2, -3}, {0, 0, 0}},
        {{1.1, 2.2, 3.3}, {1, 2, 3}, {0, 0, 0}},
        {{-1.1, -2.2, -3.3}, {-1, -2, -3}, {0, 0, 0}},
        {{1.5, 2.5, 3.5}, {2, 3, 4}, {0, 0, 0}},
        {{-1.5, -2.5, -3.5}, {-2, -3, -4}, {0, 0, 0}},
    };

    testPtToActuator(axisGeomXYZ, axesParams, testPts, sizeof(testPts) / sizeof(TestGeomPt));

}
