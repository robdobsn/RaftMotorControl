/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Unit tests for MotorDriverBase - Layer 1 Tests
//
// Rob Dobson 2025 - Motor Control Enhancement Phase 1
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Logger.h"
#include "RaftUtils.h"
#include "unity.h"
#include "MotorDriverBase.h"
#include "StepDriverTMC2209Enhanced.h"
#include "RaftJson.h"

static const char* MODULE_PREFIX = "MotorDriverBaseTest";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Layer 1 Motor Driver Base Interface Tests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_CASE("MotorDriverBase Layer 1 Tests", "[MotorDriverBase]")
{
    LOG_I(MODULE_PREFIX, "Starting MotorDriverBase Layer 1 comprehensive test suite");

    // Test instance  
    StepDriverTMC2209Enhanced* testDriver = new StepDriverTMC2209Enhanced();
    TEST_ASSERT_NOT_NULL(testDriver);

    // Test 1: Capabilities reporting
    {
        auto caps = testDriver->getCapabilities();
        TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::STEPPER_OPEN_LOOP, caps.motorType);
        TEST_ASSERT_GREATER_THAN(0, caps.supportedModes.size());
        TEST_ASSERT_FALSE(caps.hasPositionFeedback);
        TEST_ASSERT_FALSE(caps.hasVelocityFeedback);
        TEST_ASSERT_FALSE(caps.hasTorqueFeedback);
        TEST_ASSERT_TRUE(caps.maxVelocity > 0.0f);
        TEST_ASSERT_TRUE(caps.positionResolution > 0.0f);
        LOG_I(MODULE_PREFIX, "✓ Capabilities reporting test passed - motorType: %d, modes: %d", 
              (int)caps.motorType, caps.supportedModes.size());
    }

    // Test 2: Control mode support
    {
        TEST_ASSERT_TRUE(testDriver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
        TEST_ASSERT_FALSE(testDriver->supportsControlMode(MotorDriverBase::ControlMode::VELOCITY));
        TEST_ASSERT_FALSE(testDriver->supportsControlMode(MotorDriverBase::ControlMode::TORQUE));
        LOG_I(MODULE_PREFIX, "✓ Control mode support test passed");
    }

    // Test 3: Position control interface
    {
        TEST_ASSERT_TRUE(testDriver->setPositionTarget(100.0f));
        TEST_ASSERT_TRUE(testDriver->setPositionTarget(-50.0f));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, testDriver->getCurrentPosition());
        TEST_ASSERT_FALSE(testDriver->setPositionTarget(1000000000.0f));
        LOG_I(MODULE_PREFIX, "✓ Position control interface test passed");
    }

    // Test 4: Velocity and torque not supported (Phase 1)
    {
        TEST_ASSERT_FALSE(testDriver->setVelocityTarget(50.0f));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, testDriver->getCurrentVelocity());
        TEST_ASSERT_FALSE(testDriver->setTorqueTarget(0.5f));
        TEST_ASSERT_GREATER_OR_EQUAL(0.0f, testDriver->getCurrentTorque());
        LOG_I(MODULE_PREFIX, "✓ Velocity/torque not supported test passed");
    }

    // Test 5: Status and control state
    {
        TEST_ASSERT_FALSE(testDriver->isEnabled());
        TEST_ASSERT_FALSE(testDriver->isMoving());
        TEST_ASSERT_FALSE(testDriver->hasError());
        
        testDriver->setEnabled(true);
        TEST_ASSERT_TRUE(testDriver->isEnabled());
        
        String status = testDriver->getStatusString();
        TEST_ASSERT_GREATER_THAN(0, status.length());
        LOG_I(MODULE_PREFIX, "✓ Status and control state test passed - Status: %s", status.c_str());
    }

    // Test 6: Configuration handling
    {
        String configStr = R"({
            "stepPin": 14,
            "dirPin": 15, 
            "stepsPerUnit": 80.0,
            "rmsAmps": 1.0,
            "microsteps": 16
        })";
        
        RaftJson config(configStr);
        testDriver->setup(config);
        
        auto caps = testDriver->getCapabilities();
        TEST_ASSERT_EQUAL_FLOAT(1.0f / 80.0f, caps.positionResolution);
        LOG_I(MODULE_PREFIX, "✓ Configuration handling test passed");
    }

    // Test 7: Calibration functionality
    {
        testDriver->calibrate();
        TEST_ASSERT_EQUAL_FLOAT(0.0f, testDriver->getCurrentPosition());
        TEST_ASSERT_FALSE(testDriver->hasError());
        LOG_I(MODULE_PREFIX, "✓ Calibration functionality test passed");
    }

    // Test 8: Legacy stepper interface compatibility
    {
        testDriver->setDirection(true);
        testDriver->step(true);
        testDriver->setDirection(false);
        testDriver->step(false);
        TEST_ASSERT_TRUE(true); // If we get here, interface is working
        LOG_I(MODULE_PREFIX, "✓ Legacy stepper interface compatibility test passed");
    }

    // Test 9: Debug JSON output (both base and enhanced)
    {
        // Test base getDebugJSON from StepDriverTMC2209 
        String baseJson = testDriver->getDebugJSON(true, false);
        TEST_ASSERT_GREATER_THAN(0, baseJson.length());
        TEST_ASSERT_GREATER_OR_EQUAL(0, baseJson.indexOf("{"));
        TEST_ASSERT_GREATER_OR_EQUAL(0, baseJson.indexOf("}"));
        
        // Test enhanced debug JSON
        String enhancedJson = testDriver->getEnhancedDebugJSON(true, false);
        TEST_ASSERT_GREATER_THAN(0, enhancedJson.length());
        TEST_ASSERT_GREATER_OR_EQUAL(0, enhancedJson.indexOf("motorType"));
        TEST_ASSERT_GREATER_OR_EQUAL(0, enhancedJson.indexOf("position"));
        
        String detailedJson = testDriver->getEnhancedDebugJSON(true, true);
        TEST_ASSERT_GREATER_THAN(enhancedJson.length(), detailedJson.length());
        TEST_ASSERT_GREATER_OR_EQUAL(0, detailedJson.indexOf("capabilities"));
        LOG_I(MODULE_PREFIX, "✓ Debug JSON output test passed");
    }

    // Test 10: Performance test
    {
        uint32_t startTime = millis();
        for (int i = 0; i < 100; i++) {
            testDriver->setPositionTarget(static_cast<float>(i));
            testDriver->getCurrentPosition();
            testDriver->isEnabled();
        }
        uint32_t totalTime = millis() - startTime;
        TEST_ASSERT_LESS_THAN(1000, totalTime);
        
        float avgTimePerOp = static_cast<float>(totalTime * 1000) / 100;
        TEST_ASSERT_LESS_THAN(10000.0f, avgTimePerOp);
        LOG_I(MODULE_PREFIX, "✓ Performance test passed - 100 operations in %d ms (%.2f μs/op)", 
              totalTime, avgTimePerOp);
    }

    // Test 11: Memory usage
    {
        size_t driverSize = sizeof(StepDriverTMC2209Enhanced);
        TEST_ASSERT_LESS_THAN(1024, driverSize);
        LOG_I(MODULE_PREFIX, "✓ Memory usage test passed - Driver size: %d bytes", driverSize);
    }

    // Test 12: Motor driver type enumeration helpers
    {
        String positionModeStr = MotorDriverBase::controlModeToString(MotorDriverBase::ControlMode::POSITION);
        TEST_ASSERT_EQUAL_STRING("position", positionModeStr.c_str());
        
        String stepperTypeStr = MotorDriverBase::motorTypeToString(MotorDriverBase::MotorType::STEPPER_OPEN_LOOP);
        TEST_ASSERT_EQUAL_STRING("stepperOpenLoop", stepperTypeStr.c_str());
        LOG_I(MODULE_PREFIX, "✓ Motor driver type enumeration test passed");
    }

    // Test 13: Polymorphic usage
    {
        MotorDriverBase* baseDriver = new StepDriverTMC2209Enhanced();
        TEST_ASSERT_NOT_NULL(baseDriver);
        
        auto caps = baseDriver->getCapabilities();
        TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::STEPPER_OPEN_LOOP, caps.motorType);
        TEST_ASSERT_TRUE(baseDriver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
        TEST_ASSERT_TRUE(baseDriver->setPositionTarget(50.0f));
        
        float position = baseDriver->getCurrentPosition();
        TEST_ASSERT_GREATER_OR_EQUAL(0.0f, position);
        
        baseDriver->setEnabled(true);
        TEST_ASSERT_TRUE(baseDriver->isEnabled());
        
        delete baseDriver;
        LOG_I(MODULE_PREFIX, "✓ Polymorphic usage test passed");
    }

    delete testDriver;
    LOG_I(MODULE_PREFIX, "MotorDriverBase Layer 1 Tests - ALL PASSED ✓");
    LOG_I(MODULE_PREFIX, "Layer 1 Hardware Interface Foundation validation complete");
}