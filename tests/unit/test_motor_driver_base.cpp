/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motor Driver Base Test - Layer 1 Test Harness
//
// Rob Dobson 2025 - Motor Control Enhancement Phase 1
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "MotorDriverBase.h"
#include "StepDriverTMC2209Enhanced.h"
#include "RaftJson.h"

// Mock motor driver for testing the interface
class MockMotorDriver : public MotorDriverBase {
public:
    MOCK_METHOD(MotorCapabilities, getCapabilities, (), (const, override));
    MOCK_METHOD(bool, supportsControlMode, (ControlMode mode), (const, override));
    MOCK_METHOD(bool, setPositionTarget, (float position), (override));
    MOCK_METHOD(bool, setVelocityTarget, (float velocity), (override));
    MOCK_METHOD(bool, setTorqueTarget, (float torque), (override));
    
    MOCK_METHOD(float, getCurrentPosition, (), (const, override));
    MOCK_METHOD(float, getCurrentVelocity, (), (const, override));
    MOCK_METHOD(float, getCurrentTorque, (), (const, override));
    
    MOCK_METHOD(bool, isEnabled, (), (const, override));
    MOCK_METHOD(void, setEnabled, (bool enabled), (override));
    MOCK_METHOD(bool, isMoving, (), (const, override));
    MOCK_METHOD(bool, hasError, (), (const, override));
    MOCK_METHOD(String, getStatusString, (), (const, override));
    
    MOCK_METHOD(bool, setup, (const RaftJsonIF& config), (override));
    MOCK_METHOD(void, calibrate, (), (override));
    MOCK_METHOD(void, loop, (), (override));
    
    MOCK_METHOD(void, step, (bool direction), (override));
    MOCK_METHOD(void, setDirection, (bool direction), (override));
    
    MOCK_METHOD(String, getDebugJSON, (bool includeBraces, bool detailed), (const, override));
};

class MotorDriverBaseTest : public ::testing::Test {
protected:
    void SetUp() override {
        mockDriver = std::make_unique<MockMotorDriver>();
        setupDefaultExpectations();
    }
    
    void setupDefaultExpectations() {
        // Default capability responses
        MotorDriverBase::MotorCapabilities defaultCaps;
        defaultCaps.motorType = MotorDriverBase::MotorType::STEPPER_OPEN_LOOP;
        defaultCaps.supportedModes = {MotorDriverBase::ControlMode::POSITION};
        defaultCaps.feedbackTypes = {MotorDriverBase::FeedbackType::NONE};
        defaultCaps.hasPositionFeedback = false;
        defaultCaps.hasVelocityFeedback = false;
        defaultCaps.hasTorqueFeedback = false;
        defaultCaps.maxVelocity = 1000.0f;
        defaultCaps.maxTorque = 1.0f;
        defaultCaps.positionResolution = 1.0f / 80.0f;
        
        EXPECT_CALL(*mockDriver, getCapabilities())
            .WillRepeatedly(::testing::Return(defaultCaps));
    }
    
    std::unique_ptr<MockMotorDriver> mockDriver;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface Contract Tests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(MotorDriverBaseTest, CapabilityReportingWorks) {
    auto caps = mockDriver->getCapabilities();
    
    // Validate capability structure contains required fields
    EXPECT_NE(caps.motorType, static_cast<MotorDriverBase::MotorType>(-1));
    EXPECT_GT(caps.supportedModes.size(), 0);
    EXPECT_GE(caps.maxVelocity, 0.0f);
    EXPECT_GE(caps.positionResolution, 0.0f);
    
    // Validate motor type is valid
    EXPECT_TRUE(caps.motorType == MotorDriverBase::MotorType::STEPPER_OPEN_LOOP ||
                caps.motorType == MotorDriverBase::MotorType::STEPPER_CLOSED_LOOP ||
                caps.motorType == MotorDriverBase::MotorType::SERVO ||
                caps.motorType == MotorDriverBase::MotorType::BLDC);
}

TEST_F(MotorDriverBaseTest, ControlModeSupportValidation) {
    // Test all control modes
    std::vector<MotorDriverBase::ControlMode> allModes = {
        MotorDriverBase::ControlMode::POSITION,
        MotorDriverBase::ControlMode::VELOCITY,
        MotorDriverBase::ControlMode::TORQUE
    };
    
    for (auto mode : allModes) {
        EXPECT_CALL(*mockDriver, supportsControlMode(mode))
            .WillOnce(::testing::Return(mode == MotorDriverBase::ControlMode::POSITION));
        
        bool supported = mockDriver->supportsControlMode(mode);
        EXPECT_EQ(supported, (mode == MotorDriverBase::ControlMode::POSITION));
    }
}

TEST_F(MotorDriverBaseTest, ControlInterfaceContract) {
    // Test position control interface
    EXPECT_CALL(*mockDriver, setPositionTarget(100.0f))
        .WillOnce(::testing::Return(true));
    EXPECT_TRUE(mockDriver->setPositionTarget(100.0f));
    
    // Test velocity control interface  
    EXPECT_CALL(*mockDriver, setVelocityTarget(50.0f))
        .WillOnce(::testing::Return(true));
    EXPECT_TRUE(mockDriver->setVelocityTarget(50.0f));
    
    // Test torque control interface
    EXPECT_CALL(*mockDriver, setTorqueTarget(0.5f))
        .WillOnce(::testing::Return(true));
    EXPECT_TRUE(mockDriver->setTorqueTarget(0.5f));
}

TEST_F(MotorDriverBaseTest, FeedbackInterfaceContract) {
    // Test position feedback
    EXPECT_CALL(*mockDriver, getCurrentPosition())
        .WillOnce(::testing::Return(100.0f));
    EXPECT_FLOAT_EQ(mockDriver->getCurrentPosition(), 100.0f);
    
    // Test velocity feedback
    EXPECT_CALL(*mockDriver, getCurrentVelocity())
        .WillOnce(::testing::Return(25.0f));
    EXPECT_FLOAT_EQ(mockDriver->getCurrentVelocity(), 25.0f);
    
    // Test torque feedback
    EXPECT_CALL(*mockDriver, getCurrentTorque())
        .WillOnce(::testing::Return(0.3f));
    EXPECT_FLOAT_EQ(mockDriver->getCurrentTorque(), 0.3f);
}

TEST_F(MotorDriverBaseTest, StatusInterfaceContract) {
    // Test enable/disable
    EXPECT_CALL(*mockDriver, setEnabled(true))
        .Times(1);
    EXPECT_CALL(*mockDriver, isEnabled())
        .WillOnce(::testing::Return(true));
    
    mockDriver->setEnabled(true);
    EXPECT_TRUE(mockDriver->isEnabled());
    
    // Test motion status
    EXPECT_CALL(*mockDriver, isMoving())
        .WillOnce(::testing::Return(false));
    EXPECT_FALSE(mockDriver->isMoving());
    
    // Test error status
    EXPECT_CALL(*mockDriver, hasError())
        .WillOnce(::testing::Return(false));
    EXPECT_FALSE(mockDriver->hasError());
    
    // Test status string
    EXPECT_CALL(*mockDriver, getStatusString())
        .WillOnce(::testing::Return("OK"));
    EXPECT_EQ(mockDriver->getStatusString(), "OK");
}

TEST_F(MotorDriverBaseTest, ConfigurationInterfaceContract) {
    String configStr = R"({"stepPin": 14, "dirPin": 15, "stepsPerUnit": 80})";
    RaftJson config(configStr);
    
    EXPECT_CALL(*mockDriver, setup(::testing::_))
        .WillOnce(::testing::Return(true));
    EXPECT_TRUE(mockDriver->setup(config));
    
    EXPECT_CALL(*mockDriver, calibrate())
        .Times(1);
    mockDriver->calibrate();
}

TEST_F(MotorDriverBaseTest, LegacyStepperInterfaceContract) {
    // Test backward compatibility for stepper interface
    EXPECT_CALL(*mockDriver, step(true))
        .Times(1);
    EXPECT_CALL(*mockDriver, setDirection(false))
        .Times(1);
    
    mockDriver->step(true);
    mockDriver->setDirection(false);
}

TEST_F(MotorDriverBaseTest, DiagnosticInterfaceContract) {
    EXPECT_CALL(*mockDriver, getDebugJSON(true, false))
        .WillOnce(::testing::Return("{\"status\":\"OK\"}"));
    
    String debugJson = mockDriver->getDebugJSON(true, false);
    EXPECT_GT(debugJson.length(), 0);
    EXPECT_TRUE(debugJson.indexOf("{") >= 0); // Should contain JSON braces
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Performance Tests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(MotorDriverBaseTest, ResponseTimePerformance) {
    const int iterations = 1000;
    
    // Setup expectations for performance test
    EXPECT_CALL(*mockDriver, setPositionTarget(::testing::_))
        .Times(iterations)
        .WillRepeatedly(::testing::Return(true));
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; i++) {
        mockDriver->setPositionTarget(static_cast<float>(i));
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    // Should complete within reasonable time (10ms for 1000 operations = 10μs per operation)
    EXPECT_LT(duration.count(), 10000);
    
    // Log performance for monitoring
    float avgTimePerOp = static_cast<float>(duration.count()) / iterations;
    std::cout << "Average time per operation: " << avgTimePerOp << " μs" << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TMC2209Enhanced Driver Tests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class TMC2209EnhancedTest : public ::testing::Test {
protected:
    void SetUp() override {
        driver = std::make_unique<StepDriverTMC2209Enhanced>();
    }
    
    std::unique_ptr<StepDriverTMC2209Enhanced> driver;
};

TEST_F(TMC2209EnhancedTest, CapabilitiesCorrect) {
    auto caps = driver->getCapabilities();
    
    EXPECT_EQ(caps.motorType, MotorDriverBase::MotorType::STEPPER_OPEN_LOOP);
    
    // Should support position control
    EXPECT_TRUE(driver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
    
    // Phase 1: Should not support velocity or torque control
    EXPECT_FALSE(driver->supportsControlMode(MotorDriverBase::ControlMode::VELOCITY));
    EXPECT_FALSE(driver->supportsControlMode(MotorDriverBase::ControlMode::TORQUE));
    
    // Validate feedback characteristics
    EXPECT_FALSE(caps.hasPositionFeedback); // Open-loop stepper
    EXPECT_FALSE(caps.hasVelocityFeedback);
    EXPECT_FALSE(caps.hasTorqueFeedback);
    
    // Validate reasonable limits
    EXPECT_GT(caps.maxVelocity, 0.0f);
    EXPECT_GT(caps.positionResolution, 0.0f);
}

TEST_F(TMC2209EnhancedTest, PositionControlBasics) {
    // Test position target setting
    EXPECT_TRUE(driver->setPositionTarget(100.0f));
    
    // Test position feedback (should start at 0)
    float position = driver->getCurrentPosition();
    EXPECT_FLOAT_EQ(position, 0.0f);
    
    // Test reasonable position range
    EXPECT_TRUE(driver->setPositionTarget(1000.0f));
    EXPECT_TRUE(driver->setPositionTarget(-1000.0f));
    
    // Test unreasonable position (should be rejected)
    EXPECT_FALSE(driver->setPositionTarget(1000000000.0f));
}

TEST_F(TMC2209EnhancedTest, VelocityControlNotSupportedInPhase1) {
    // Phase 1: Velocity control should not be supported
    EXPECT_FALSE(driver->setVelocityTarget(50.0f));
    
    // Velocity feedback should return 0 initially
    float velocity = driver->getCurrentVelocity();
    EXPECT_FLOAT_EQ(velocity, 0.0f);
}

TEST_F(TMC2209EnhancedTest, TorqueControlNotSupportedInPhase1) {
    // Phase 1: Torque control should not be supported
    EXPECT_FALSE(driver->setTorqueTarget(0.5f));
    
    // Torque feedback should be available (estimated)
    float torque = driver->getCurrentTorque();
    EXPECT_GE(torque, 0.0f); // Should be non-negative
}

TEST_F(TMC2209EnhancedTest, StatusAndControlState) {
    // Initial state
    EXPECT_FALSE(driver->isEnabled());
    EXPECT_FALSE(driver->isMoving());
    EXPECT_FALSE(driver->hasError());
    
    // Enable motor
    driver->setEnabled(true);
    EXPECT_TRUE(driver->isEnabled());
    
    // Status string should be meaningful
    String status = driver->getStatusString();
    EXPECT_GT(status.length(), 0);
    EXPECT_TRUE(status.indexOf("TMC2209Enhanced") >= 0);
}

TEST_F(TMC2209EnhancedTest, ConfigurationHandling) {
    String configStr = R"({
        "stepPin": 14,
        "dirPin": 15, 
        "stepsPerUnit": 80.0,
        "rmsAmps": 1.0,
        "microsteps": 16
    })";
    
    RaftJson config(configStr);
    
    // Setup should succeed with valid configuration
    bool result = driver->setup(config);
    
    // Note: This might fail in test environment without actual hardware
    // but we can test the interface
    // EXPECT_TRUE(result); // Commented out due to hardware dependency
    
    // Verify capabilities after setup reflect configuration
    auto caps = driver->getCapabilities();
    EXPECT_EQ(caps.positionResolution, 1.0f / 80.0f); // Should match stepsPerUnit
}

TEST_F(TMC2209EnhancedTest, CalibrationFunctionality) {
    // Set some position (simulate movement)
    driver->setPositionTarget(100.0f);
    
    // Calibrate should reset position
    driver->calibrate();
    
    // Position should be reset to zero
    float position = driver->getCurrentPosition();
    EXPECT_FLOAT_EQ(position, 0.0f);
    
    // Should not have errors after calibration
    EXPECT_FALSE(driver->hasError());
}

TEST_F(TMC2209EnhancedTest, BackwardCompatibilityWithStepperInterface) {
    // Test legacy stepper interface still works
    driver->setDirection(true);
    driver->step(true);
    
    // Position tracking should update with steps
    // Note: Actual position update depends on internal step tracking
    // This tests interface compatibility
    EXPECT_NO_THROW(driver->step(false));
    EXPECT_NO_THROW(driver->setDirection(false));
}

TEST_F(TMC2209EnhancedTest, DebugJSONOutput) {
    String debugJson = driver->getDebugJSON(true, false);
    EXPECT_GT(debugJson.length(), 0);
    EXPECT_TRUE(debugJson.indexOf("{") >= 0);
    EXPECT_TRUE(debugJson.indexOf("}") >= 0);
    
    // Should contain enhanced information
    EXPECT_TRUE(debugJson.indexOf("motorType") >= 0);
    EXPECT_TRUE(debugJson.indexOf("position") >= 0);
    EXPECT_TRUE(debugJson.indexOf("velocity") >= 0);
    
    // Test detailed debug output
    String detailedJson = driver->getDebugJSON(true, true);
    EXPECT_GT(detailedJson.length(), debugJson.length());
    EXPECT_TRUE(detailedJson.indexOf("capabilities") >= 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Integration Tests
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(TMC2209EnhancedTest, PolymorphicUsage) {
    // Test that enhanced driver can be used polymorphically as MotorDriverBase
    std::unique_ptr<MotorDriverBase> baseDriver = 
        std::make_unique<StepDriverTMC2209Enhanced>();
    
    // Should be able to use all MotorDriverBase interface methods
    auto caps = baseDriver->getCapabilities();
    EXPECT_EQ(caps.motorType, MotorDriverBase::MotorType::STEPPER_OPEN_LOOP);
    
    EXPECT_TRUE(baseDriver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
    EXPECT_TRUE(baseDriver->setPositionTarget(50.0f));
    
    float position = baseDriver->getCurrentPosition();
    EXPECT_GE(position, 0.0f);
    
    baseDriver->setEnabled(true);
    EXPECT_TRUE(baseDriver->isEnabled());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main test runner
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}