# Test Harness Specifications

## Overview

This document specifies comprehensive test harnesses for each layer of the RaftMotorControl enhancement implementation. Each test harness validates interfaces, functionality, and performance at its architectural level using the new abstraction hierarchy.

## Test Harness Architecture

```
System Tests (End-to-End Integration)
    ↑
Integration Tests (Cross-Layer Communication) 
    ↑
Component Tests (Individual Layer Validation)
    ↑
Unit Tests (Class-Level Testing)
    ↑
Mock Infrastructure (Test Doubles & Stubs)
```

---

## Layer 1: Step/Direction Hardware Interface Test Harnesses

### **1.1 StepDriverStepDirectionBase Interface Test**

**Purpose**: Validate the clean step/direction hardware abstraction

**Test File**: `unit_tests/main/StepDriverStepDirectionBase_test.cpp`

#### **Test Implementation Infrastructure**
```cpp
// Test implementation for validation
class TestStepDriver : public StepDriverStepDirectionBase {
private:
    bool _stepActive = false;
    bool _direction = false;
    uint32_t _microsteps = 16;
    float _maxCurrentAmps = 1.0f;
    
public:
    // Required implementations
    void setMicrosteps(uint32_t microsteps) override {
        _microsteps = microsteps;
    }

    void setDirection(bool dirn, bool forceSet = false) override {
        _direction = dirn;
    }

    void stepStart() override {
        _stepActive = true;
    }

    bool stepEnd() override {
        if (_stepActive) {
            _stepActive = false;
            return true;
        }
        return false;
    }

    String getDriverType() const override {
        return "TestStepDriver";
    }

    void setMaxMotorCurrentAmps(float maxMotorCurrentAmps) override {
        _maxCurrentAmps = maxMotorCurrentAmps;
    }

    String getDebugJSON(bool includeBraces, bool detailed) const override {
        return includeBraces ? "{\"type\":\"test\"}" : "\"type\":\"test\"";
    }

    String getStatusJSON(bool includeBraces, bool detailed) const override {
        return includeBraces ? "{\"status\":\"ok\"}" : "\"status\":\"ok\"";
    }

    bool isOperatingOk() const override {
        return true;
    }

    // Test accessors
    bool getDirection() const { return _direction; }
    bool isStepActive() const { return _stepActive; }
    uint32_t getMicrosteps() const { return _microsteps; }
    float getMaxCurrentAmps() const { return _maxCurrentAmps; }
};
```

#### **Test Cases**
```cpp
void test_step_direction_interface(void) {
    TEST_ASSERT_NOT_NULL(testDriver);
    
    // Test stepping sequence
    TEST_ASSERT_FALSE(testDriver->isStepActive());
    testDriver->stepStart();
    TEST_ASSERT_TRUE(testDriver->isStepActive());
    
    bool stepResult = testDriver->stepEnd();
    TEST_ASSERT_TRUE(stepResult);
    TEST_ASSERT_FALSE(testDriver->isStepActive());
}

void test_direction_control(void) {
    testDriver->setDirection(true);
    TEST_ASSERT_TRUE(testDriver->getDirection());
    
    testDriver->setDirection(false);
    TEST_ASSERT_FALSE(testDriver->getDirection());
}

void test_microstep_configuration(void) {
    testDriver->setMicrosteps(32);
    TEST_ASSERT_EQUAL(32, testDriver->getMicrosteps());
    
    testDriver->setMicrosteps(8);
    TEST_ASSERT_EQUAL(8, testDriver->getMicrosteps());
}

void test_current_setting(void) {
    testDriver->setMaxMotorCurrentAmps(1.5f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f, testDriver->getMaxCurrentAmps());
}

void test_driver_identification(void) {
    String driverType = testDriver->getDriverType();
    TEST_ASSERT_TRUE(driverType == "TestStepDriver");
}

void test_status_reporting(void) {
    String statusJson = testDriver->getStatusJSON(true, false);
    TEST_ASSERT_TRUE(statusJson.length() > 0);
    TEST_ASSERT_TRUE(statusJson.indexOf("status") >= 0);
    
    String debugJson = testDriver->getDebugJSON(true, true);
    TEST_ASSERT_TRUE(debugJson.length() > 0);
}

void test_operational_status(void) {
    TEST_ASSERT_TRUE(testDriver->isOperatingOk());
}
```

#### **Validation Criteria**
- [ ] Step/direction interface operates correctly
- [ ] Microstep settings applied properly
- [ ] Current control works as expected
- [ ] Driver identification and status reporting functional
- [ ] No high-level motor concepts leaked into interface

### **1.2 StepDriverTMC2209Enhanced Integration Test**

**Purpose**: Validate TMC2209 driver works with new interface

**Test File**: `unit_tests/main/StepDriverTMC2209Enhanced_test.cpp`

#### **Mock Hardware Infrastructure**
```cpp
class MockTMC2209Hardware {
private:
    bool _stepPinState = false;
    bool _directionPinState = false;
    uint32_t _microsteps = 16;
    float _currentAmps = 1.0f;
    
public:
    void setStepPin(bool state) { _stepPinState = state; }
    void setDirectionPin(bool state) { _directionPinState = state; }
    void setMicrosteps(uint32_t microsteps) { _microsteps = microsteps; }
    void setCurrent(float amps) { _currentAmps = amps; }
    
    bool getStepPin() const { return _stepPinState; }
    bool getDirectionPin() const { return _directionPinState; }
    uint32_t getMicrosteps() const { return _microsteps; }
    float getCurrent() const { return _currentAmps; }
};
```

#### **Test Cases**
```cpp
void test_tmc2209_setup_and_configuration(void) {
    String config = R"({
        "stepPin": 14,
        "dirPin": 15,
        "microsteps": 16,
        "rmsAmps": 1.2
    })";
    
    StepDriverParams params;
    params.fromJSON(config);
    
    bool result = tmc2209Driver->setup("TestMotor", params, false);
    TEST_ASSERT_TRUE(result);
    
    String driverType = tmc2209Driver->getDriverType();
    TEST_ASSERT_TRUE(driverType == "TMC2209Enhanced");
}

void test_tmc2209_step_generation(void) {
    // Configure driver
    setupDriver();
    
    // Test step sequence
    tmc2209Driver->setDirection(true);
    tmc2209Driver->stepStart();
    
    // Verify hardware pins
    TEST_ASSERT_TRUE(mockHardware->getDirectionPin());
    TEST_ASSERT_TRUE(mockHardware->getStepPin());
    
    bool stepCompleted = tmc2209Driver->stepEnd();
    TEST_ASSERT_TRUE(stepCompleted);
    TEST_ASSERT_FALSE(mockHardware->getStepPin());
}

void test_tmc2209_microstep_configuration(void) {
    setupDriver();
    
    tmc2209Driver->setMicrosteps(32);
    // Verify register configuration via mock
    TEST_ASSERT_EQUAL(32, mockHardware->getMicrosteps());
}

void test_tmc2209_current_control(void) {
    setupDriver();
    
    tmc2209Driver->setMaxMotorCurrentAmps(1.8f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.8f, mockHardware->getCurrent());
}

void test_tmc2209_error_conditions(void) {
    setupDriver();
    
    // Test with invalid pin configuration
    StepDriverParams invalidParams;
    invalidParams.stepPin = -1; // Invalid pin
    
    bool result = tmc2209Driver->setup("InvalidMotor", invalidParams, false);
    TEST_ASSERT_FALSE(result);
}

void test_tmc2209_status_reporting(void) {
    setupDriver();
    
    String status = tmc2209Driver->getStatusJSON(true, false);
    TEST_ASSERT_TRUE(status.length() > 0);
    
    bool isOk = tmc2209Driver->isOperatingOk();
    TEST_ASSERT_TRUE(isOk);
}
```

#### **Validation Criteria**
- [ ] TMC2209 driver integrates with new interface
- [ ] All step/direction operations work correctly
- [ ] Hardware pin control functions properly
- [ ] Current and microstep settings applied
- [ ] Error conditions handled appropriately

---

## Layer 2: Abstract Motor Interface Test Harnesses

### **2.1 MotorDriverBase Interface Test**

**Purpose**: Validate high-level motor driver abstraction

**Test File**: `unit_tests/main/MotorDriverBase_test.cpp`

#### **Mock Motor Driver Infrastructure**
```cpp
class MockMotorDriver : public MotorDriverBase {
private:
    MotorCapabilities _caps;
    bool _enabled = false;
    bool _hasError = false;
    float _positions[3] = {0.0f, 0.0f, 0.0f};
    float _velocities[3] = {0.0f, 0.0f, 0.0f};
    float _torques[3] = {0.0f, 0.0f, 0.0f};
    uint32_t _numAxes = 3;
    
public:
    MockMotorDriver() {
        _caps.motorType = MotorType::BASIC_STEPPER;
        _caps.supportsPositionMode = true;
        _caps.supportsVelocityMode = false;
        _caps.supportsTorqueMode = false;
        _caps.hasPositionFeedback = false;
        _caps.hasVelocityFeedback = false;
        _caps.hasTorqueFeedback = false;
        _caps.maxVelocity = 1000.0f;
        _caps.maxTorque = 0.0f;
        _caps.positionResolution = 0.0125f; // 80 steps per unit
        _caps.numAxes = _numAxes;
    }
    
    // Full MotorDriverBase implementation with configurable behavior
    MotorCapabilities getCapabilities() const override { return _caps; }
    
    bool supportsControlMode(ControlMode mode) const override {
        switch (mode) {
            case ControlMode::POSITION: return _caps.supportsPositionMode;
            case ControlMode::VELOCITY: return _caps.supportsVelocityMode;
            case ControlMode::TORQUE: return _caps.supportsTorqueMode;
        }
        return false;
    }
    
    bool setPositionTarget(uint32_t axisIdx, float position) override {
        if (axisIdx >= _numAxes) return false;
        if (!_caps.supportsPositionMode) return false;
        _positions[axisIdx] = position;
        return true;
    }
    
    // ... other interface implementations
    
    // Test configuration methods
    void setCapabilities(const MotorCapabilities& caps) { _caps = caps; }
    void setHasError(bool hasError) { _hasError = hasError; }
    void setNumAxes(uint32_t numAxes) { 
        _numAxes = numAxes; 
        _caps.numAxes = numAxes; 
    }
};
```

#### **Test Cases**
```cpp
void test_motor_capabilities_query(void) {
    auto caps = mockDriver->getCapabilities();
    
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, caps.motorType);
    TEST_ASSERT_TRUE(caps.supportsPositionMode);
    TEST_ASSERT_FALSE(caps.supportsVelocityMode);
    TEST_ASSERT_FALSE(caps.supportsTorqueMode);
    TEST_ASSERT_EQUAL(3, caps.numAxes);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, caps.maxVelocity);
}

void test_control_mode_support_validation(void) {
    TEST_ASSERT_TRUE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
    TEST_ASSERT_FALSE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::VELOCITY));
    TEST_ASSERT_FALSE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::TORQUE));
}

void test_multi_axis_position_control(void) {
    // Test multiple axes
    TEST_ASSERT_TRUE(mockDriver->setPositionTarget(0, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, mockDriver->getCurrentPosition(0));
    
    TEST_ASSERT_TRUE(mockDriver->setPositionTarget(1, -50.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.0f, mockDriver->getCurrentPosition(1));
    
    TEST_ASSERT_TRUE(mockDriver->setPositionTarget(2, 25.5f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 25.5f, mockDriver->getCurrentPosition(2));
}

void test_unsupported_control_modes(void) {
    // Should fail because mock driver doesn't support velocity/torque modes
    TEST_ASSERT_FALSE(mockDriver->setVelocityTarget(0, 25.0f));
    TEST_ASSERT_FALSE(mockDriver->setTorqueTarget(0, 0.5f));
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mockDriver->getCurrentVelocity(0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mockDriver->getCurrentTorque(0));
}

void test_axis_bounds_validation(void) {
    // Invalid axis should return false/0.0
    TEST_ASSERT_FALSE(mockDriver->setPositionTarget(5, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mockDriver->getCurrentPosition(5));
}

void test_enable_disable_functionality(void) {
    TEST_ASSERT_FALSE(mockDriver->isEnabled());
    
    mockDriver->setEnabled(true);
    TEST_ASSERT_TRUE(mockDriver->isEnabled());
    
    mockDriver->setEnabled(false);
    TEST_ASSERT_FALSE(mockDriver->isEnabled());
}

void test_error_state_management(void) {
    TEST_ASSERT_FALSE(mockDriver->hasError());
    
    mockDriver->setHasError(true);
    TEST_ASSERT_TRUE(mockDriver->hasError());
    
    String status = mockDriver->getStatusString();
    TEST_ASSERT_TRUE(status.length() > 0);
}

void test_different_motor_types(void) {
    // Test BLDC motor capabilities
    MotorDriverBase::MotorCapabilities bldcCaps;
    bldcCaps.motorType = MotorDriverBase::MotorType::BLDC;
    bldcCaps.supportsPositionMode = true;
    bldcCaps.supportsVelocityMode = true;
    bldcCaps.supportsTorqueMode = true;
    bldcCaps.hasVelocityFeedback = true;
    bldcCaps.hasTorqueFeedback = true;
    
    mockDriver->setCapabilities(bldcCaps);
    
    auto caps = mockDriver->getCapabilities();
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BLDC, caps.motorType);
    TEST_ASSERT_TRUE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::VELOCITY));
    TEST_ASSERT_TRUE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::TORQUE));
}
```

#### **Validation Criteria**
- [ ] Motor capabilities correctly reported for different types
- [ ] Multi-axis support works properly
- [ ] Control mode validation functions correctly
- [ ] Error conditions handled appropriately
- [ ] Interface extensible for different motor types

---

## Layer 3: Motor Driver Implementation Test Harnesses

### **3.1 MotorDriverBasicStepper Test**

**Purpose**: Validate basic stepper driver implementation with encapsulated RampGenerator

**Test File**: `unit_tests/main/MotorDriverBasicStepper_test.cpp`

#### **Test Configuration Infrastructure**
```cpp
class MotorDriverBasicStepperTest {
private:
    MotorDriverBasicStepper* driver = nullptr;
    String testConfig = R"({
        "axes": [
            {
                "name": "X",
                "params": {"maxSpeed": 1000, "stepsPerUnit": 80, "maxAcceleration": 500},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15, "microsteps": 16}
            },
            {
                "name": "Y",
                "params": {"maxSpeed": 800, "stepsPerUnit": 100, "maxAcceleration": 400},
                "driver": {"driver": "TMC2209", "stepPin": 16, "dirPin": 17, "microsteps": 32}
            }
        ],
        "ramp": {"timerIndex": 0, "queueLength": 100}
    })";
    
public:
    void setUp() {
        driver = new MotorDriverBasicStepper();
    }
    
    void tearDown() {
        if (driver) {
            delete driver;
            driver = nullptr;
        }
    }
    
    void setupDriver(uint32_t numAxes = 2) {
        RaftJson config(testConfig);
        driver->setup(config, numAxes);
    }
};
```

#### **Test Cases**
```cpp
void test_basic_stepper_capabilities_reporting(void) {
    auto caps = driver->getCapabilities();
    
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, caps.motorType);
    TEST_ASSERT_TRUE(caps.supportsPositionMode);
    TEST_ASSERT_FALSE(caps.supportsVelocityMode); // Limited for basic steppers
    TEST_ASSERT_FALSE(caps.supportsTorqueMode);   // Not supported
    TEST_ASSERT_FALSE(caps.hasPositionFeedback);  // Step counting only
    TEST_ASSERT_FALSE(caps.hasVelocityFeedback);
    TEST_ASSERT_FALSE(caps.hasTorqueFeedback);
}

void test_multi_axis_setup_and_configuration(void) {
    setupDriver(2);
    
    auto caps = driver->getCapabilities();
    TEST_ASSERT_EQUAL(2, caps.numAxes);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, caps.maxVelocity); // From axis 0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0125f, caps.positionResolution); // 80 steps/unit
}

void test_position_control_interface(void) {
    setupDriver(2);
    
    // Test position setting for multiple axes
    bool result1 = driver->setPositionTarget(0, 100.0f);
    TEST_ASSERT_TRUE(result1);
    
    bool result2 = driver->setPositionTarget(1, -50.5f);
    TEST_ASSERT_TRUE(result2);
    
    // Test invalid axis
    bool result3 = driver->setPositionTarget(5, 100.0f);
    TEST_ASSERT_FALSE(result3);
}

void test_position_feedback_via_step_counting(void) {
    setupDriver(1);
    
    // Set a position and verify tracking
    driver->setPositionTarget(0, 80.0f); // 80 * 80 = 6400 steps
    
    // Position feedback should be based on step counting
    float currentPos = driver->getCurrentPosition(0);
    // Exact value depends on RampGenerator integration
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 80.0f, currentPos);
}

void test_unsupported_control_modes(void) {
    setupDriver(1);
    
    // Velocity control not supported for basic steppers
    bool velResult = driver->setVelocityTarget(0, 50.0f);
    TEST_ASSERT_FALSE(velResult);
    
    // Torque control not supported
    bool torqueResult = driver->setTorqueTarget(0, 0.5f);
    TEST_ASSERT_FALSE(torqueResult);
    
    // Feedback should return zero for unsupported modes
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, driver->getCurrentVelocity(0));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, driver->getCurrentTorque(0));
}

void test_enable_disable_coordination(void) {
    setupDriver(1);
    
    TEST_ASSERT_FALSE(driver->isEnabled());
    
    driver->setEnabled(true);
    TEST_ASSERT_TRUE(driver->isEnabled());
    
    driver->setEnabled(false);
    TEST_ASSERT_FALSE(driver->isEnabled());
}

void test_motion_status_tracking(void) {
    setupDriver(1);
    
    // Initially not moving
    TEST_ASSERT_FALSE(driver->isMoving());
    TEST_ASSERT_FALSE(driver->isBusy());
    
    // After setting position target, may be busy
    driver->setPositionTarget(0, 100.0f);
    // Status depends on RampGenerator pipeline state
}

void test_error_state_aggregation(void) {
    setupDriver(1);
    
    // Should not have errors initially
    TEST_ASSERT_FALSE(driver->hasError());
    
    // Error state should aggregate from step drivers
    String status = driver->getStatusString();
    TEST_ASSERT_TRUE(status.length() > 0);
    TEST_ASSERT_TRUE(status.indexOf("BasicStepper") >= 0);
}

void test_step_driver_integration(void) {
    setupDriver(2);
    
    // Verify step drivers are created and configured
    // This tests the internal setupStepDrivers() method
    auto caps = driver->getCapabilities();
    TEST_ASSERT_EQUAL(2, caps.numAxes);
    
    // Test that loop() doesn't crash
    driver->loop();
}

void test_ramp_generator_encapsulation(void) {
    setupDriver(1);
    
    // Test that RampGenerator functionality is working
    driver->setPositionTarget(0, 100.0f);
    
    // Should not be moving initially, but may become busy
    // This tests that the internal RampGenerator is properly integrated
    driver->loop(); // Process any pending motions
}

void test_axes_parameter_integration(void) {
    setupDriver(2);
    
    // Test that axis parameters are correctly loaded
    auto caps = driver->getCapabilities();
    
    // Max velocity should come from axis 0 parameters (1000.0)
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, caps.maxVelocity);
    
    // Position resolution should be 1/80 = 0.0125
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0125f, caps.positionResolution);
}
```

#### **Validation Criteria**
- [ ] Basic stepper capabilities correctly reported
- [ ] Multi-axis setup and configuration works
- [ ] Position control interface functional
- [ ] RampGenerator properly encapsulated
- [ ] Step drivers correctly integrated
- [ ] Error states aggregated from lower layers

### **3.2 BasicStepperRampGenerator Test**

**Purpose**: Validate renamed RampGenerator functionality

**Test File**: `unit_tests/main/BasicStepperRampGenerator_test.cpp`

#### **Test Cases**
```cpp
void test_ramp_generator_rename_functionality(void) {
    // Verify that renamed RampGenerator works identically
    BasicStepperRampGenerator rampGen;
    
    // Setup with mock step drivers
    std::vector<StepDriverStepDirectionBase*> mockDrivers;
    std::vector<EndStops*> mockEndStops;
    
    String config = R"({"timerIndex": 0, "queueLength": 50})";
    RaftJson rampConfig(config);
    
    rampGen.setup(rampConfig, mockDrivers, mockEndStops);
    
    // Test basic functionality
    rampGen.start();
    TEST_ASSERT_FALSE(rampGen.getPeriodUs() == 0); // Should have valid period
    
    rampGen.stop();
    rampGen.pause(true);
    rampGen.pause(false);
    
    // Should not crash
    rampGen.loop();
}

void test_step_driver_interface_compatibility(void) {
    // Test that renamed interface works with StepDriverStepDirectionBase
    BasicStepperRampGenerator rampGen;
    
    // Create test step driver
    TestStepDriver* testDriver = new TestStepDriver();
    std::vector<StepDriverStepDirectionBase*> drivers = { testDriver };
    std::vector<EndStops*> endStops;
    
    String config = R"({"timerIndex": 0})";
    RaftJson rampConfig(config);
    
    rampGen.setup(rampConfig, drivers, endStops);
    
    // Verify no crashes with new interface
    rampGen.loop();
    
    delete testDriver;
}
```

---

## Layer 4: Factory & Integration Test Harnesses

### **4.1 MotorDriverFactory Test**

**Purpose**: Validate factory pattern and motor type creation

**Test File**: `unit_tests/main/MotorDriverFactory_test.cpp`

#### **Test Cases**
```cpp
void test_factory_basic_stepper_creation(void) {
    String config = R"({
        "motorType": "BasicStepper",
        "axes": [
            {
                "name": "X",
                "params": {"stepsPerUnit": 80, "maxSpeed": 1000},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15}
            }
        ],
        "ramp": {"timerIndex": 0}
    })";
    
    RaftJson jsonConfig(config);
    auto driver = MotorDriverFactory::createDriver("BasicStepper", jsonConfig, 1);
    
    TEST_ASSERT_NOT_NULL(driver.get());
    
    auto caps = driver->getCapabilities();
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, caps.motorType);
    TEST_ASSERT_EQUAL(1, caps.numAxes);
}

void test_factory_creation_by_enum(void) {
    String config = R"({
        "axes": [{"name": "X", "driver": {"driver": "TMC2209"}}],
        "ramp": {}
    })";
    
    RaftJson jsonConfig(config);
    auto driver = MotorDriverFactory::createDriver(
        MotorDriverBase::MotorType::BASIC_STEPPER, 
        jsonConfig, 
        1
    );
    
    TEST_ASSERT_NOT_NULL(driver.get());
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, 
                      driver->getCapabilities().motorType);
}

void test_motor_type_auto_detection(void) {
    // Test explicit motor type specification
    String explicitConfig = R"({"motorType": "BasicStepper"})";
    RaftJson explicitJson(explicitConfig);
    auto type1 = MotorDriverFactory::detectMotorType(explicitJson);
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, type1);
    
    // Test detection from driver configuration
    String autoConfig = R"({
        "axes": [{"driver": {"driver": "TMC2209"}}]
    })";
    RaftJson autoJson(autoConfig);
    auto type2 = MotorDriverFactory::detectMotorType(autoJson);
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, type2);
    
    // Test servo stepper detection (future)
    String servoConfig = R"({
        "axes": [{"driver": {"driver": "TMC2209", "encoder": true}}]
    })";
    RaftJson servoJson(servoConfig);
    auto type3 = MotorDriverFactory::detectMotorType(servoJson);
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::SERVO_STEPPER, type3);
}

void test_supported_motor_types_registry(void) {
    auto supportedTypes = MotorDriverFactory::getSupportedMotorTypes();
    TEST_ASSERT_TRUE(supportedTypes.size() > 0);
    
    bool foundBasicStepper = false;
    for (const auto& type : supportedTypes) {
        if (type == "BasicStepper") {
            foundBasicStepper = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(foundBasicStepper);
}

void test_motor_type_name_conversion(void) {
    String name = MotorDriverFactory::getMotorTypeName(
        MotorDriverBase::MotorType::BASIC_STEPPER
    );
    TEST_ASSERT_TRUE(name == "BasicStepper");
    
    String unknownName = MotorDriverFactory::getMotorTypeName(
        static_cast<MotorDriverBase::MotorType>(999)
    );
    TEST_ASSERT_TRUE(unknownName == "Unknown");
}

void test_invalid_configuration_handling(void) {
    String invalidConfig = R"({"axes": []})"; // No axes
    RaftJson jsonConfig(invalidConfig);
    
    auto driver = MotorDriverFactory::createDriver("BasicStepper", jsonConfig, 0);
    // Should handle gracefully - may return null or configured driver
}

void test_fallback_motor_type_selection(void) {
    String config = R"({"axes": [{"driver": {"driver": "TMC2209"}}]})";
    RaftJson jsonConfig(config);
    
    // Test with non-existent motor type - should fall back to BasicStepper
    auto driver = MotorDriverFactory::createDriver("NonExistentType", jsonConfig, 1);
    
    TEST_ASSERT_NOT_NULL(driver.get());
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, 
                      driver->getCapabilities().motorType);
}
```

#### **Validation Criteria**
- [ ] Factory creates correct motor driver types
- [ ] Auto-detection logic works properly
- [ ] Configuration parsing handles all cases
- [ ] Error handling for invalid configurations
- [ ] Registry system extensible for future types

---

## Layer 5: Control Coordination Test Harnesses

### **5.1 MotionController Integration Test**

**Purpose**: Validate MotionController integration with new architecture

**Test File**: `unit_tests/main/MotionControllerIntegration_test.cpp`

#### **Test Cases**
```cpp
void test_motion_controller_setup_with_factory(void) {
    String config = R"({
        "motorType": "BasicStepper",
        "axes": [
            {
                "name": "X",
                "params": {"maxSpeed": 1000, "stepsPerUnit": 80},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15}
            }
        ],
        "ramp": {"timerIndex": 0}
    })";
    
    RaftJson jsonConfig(config);
    motionController->setup(jsonConfig);
    
    // Verify successful setup
    TEST_ASSERT_FALSE(motionController->isBusy());
    TEST_ASSERT_FALSE(motionController->isPaused());
}

void test_position_control_routing(void) {
    setupController();
    
    MotionArgs args;
    args.setControlMode(MotorDriverBase::ControlMode::POSITION);
    args.getAxesPos().setVal(0, 100.0f);
    args.getAxesSpecified().setVal(0, true);
    
    bool result = motionController->moveTo(args);
    TEST_ASSERT_TRUE(result);
}

void test_backward_compatibility_motion_args(void) {
    setupController();
    
    MotionArgs args;
    // Don't explicitly set control mode - should default to position
    args.getAxesPos().setVal(0, 50.0f);
    args.getAxesSpecified().setVal(0, true);
    
    bool result = motionController->moveTo(args);
    TEST_ASSERT_TRUE(result);
    
    // Should default to position mode
    TEST_ASSERT_EQUAL(MotorDriverBase::ControlMode::POSITION, args.getControlMode());
}

void test_special_command_handling(void) {
    setupController();
    
    // Test stop command
    MotionArgs stopArgs;
    stopArgs.setStopMotion(true);
    bool result = motionController->moveTo(stopArgs);
    TEST_ASSERT_TRUE(result);
    
    // Test clear queue command
    MotionArgs clearArgs;
    clearArgs.setClearQueue(true);
    result = motionController->moveTo(clearArgs);
    TEST_ASSERT_TRUE(result);
    
    // Test disable motors command
    MotionArgs disableArgs;
    disableArgs.setEnableMotors(false);
    result = motionController->moveTo(disableArgs);
    TEST_ASSERT_TRUE(result);
}

void test_unsupported_control_mode_rejection(void) {
    setupController();
    
    // Test velocity mode (not yet implemented)
    MotionArgs velArgs;
    velArgs.setControlMode(MotorDriverBase::ControlMode::VELOCITY);
    velArgs.setVelocityTarget(0, 50.0f);
    
    bool result = motionController->moveTo(velArgs);
    TEST_ASSERT_FALSE(result); // Should fail - not yet implemented
    
    // Test torque mode (not yet implemented)
    MotionArgs torqueArgs;
    torqueArgs.setControlMode(MotorDriverBase::ControlMode::TORQUE);
    torqueArgs.setTorqueTarget(0, 0.5f);
    
    result = motionController->moveTo(torqueArgs);
    TEST_ASSERT_FALSE(result); // Should fail - not yet implemented
}

void test_multi_axis_coordination(void) {
    String multiAxisConfig = R"({
        "axes": [
            {"name": "X", "params": {"maxSpeed": 1000}, "driver": {"driver": "TMC2209"}},
            {"name": "Y", "params": {"maxSpeed": 800}, "driver": {"driver": "TMC2209"}},
            {"name": "Z", "params": {"maxSpeed": 600}, "driver": {"driver": "TMC2209"}}
        ]
    })";
    
    RaftJson config(multiAxisConfig);
    motionController->setup(config);
    
    MotionArgs args;
    args.getAxesPos().setVal(0, 100.0f);
    args.getAxesPos().setVal(1, 50.0f);
    args.getAxesPos().setVal(2, -25.0f);
    args.getAxesSpecified().setVal(0, true);
    args.getAxesSpecified().setVal(1, true);
    args.getAxesSpecified().setVal(2, true);
    
    bool result = motionController->moveTo(args);
    TEST_ASSERT_TRUE(result);
}

void test_motion_controller_loop_integration(void) {
    setupController();
    
    // Send motion command
    MotionArgs args;
    args.getAxesPos().setVal(0, 100.0f);
    args.getAxesSpecified().setVal(0, true);
    motionController->moveTo(args);
    
    // Loop should not crash and should process motion
    for (int i = 0; i < 10; i++) {
        motionController->loop();
    }
}
```

#### **Validation Criteria**
- [ ] MotionController integrates with motor driver factory
- [ ] Position control routing works correctly
- [ ] Backward compatibility maintained
- [ ] Special commands handled properly
- [ ] Multi-axis coordination functional

---

## Layer 6: Application Interface Test Harnesses

### **6.1 MotorControl JSON Interface Test**

**Purpose**: Validate application-level JSON command interface

**Test File**: `unit_tests/main/MotorControlApplication_test.cpp`

#### **Test Cases**
```cpp
void test_existing_json_commands_unchanged(void) {
    setupMotorControl();
    
    // Test existing position command format
    String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":100}]})";
    RaftRetCode result = motorControl->sendCmdJSON(cmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
    
    // Test relative motion
    String relCmd = R"({"cmd":"motion","relative":true,"pos":[{"a":0,"p":10}]})";
    result = motorControl->sendCmdJSON(relCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
}

void test_enhanced_motion_args_parsing(void) {
    setupMotorControl();
    
    // Test explicit position mode
    String cmd = R"({
        "cmd": "motion",
        "mode": "position",
        "pos": [{"a": 0, "p": 50}]
    })";
    
    RaftRetCode result = motorControl->sendCmdJSON(cmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
}

void test_non_motion_commands_preserved(void) {
    setupMotorControl();
    
    // Test motor current command
    String currentCmd = R"({"cmd":"maxCurrent","axisIdx":0,"maxCurrentA":1.5})";
    RaftRetCode result1 = motorControl->sendCmdJSON(currentCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result1);
    
    // Test motor timeout command
    String timeoutCmd = R"({"cmd":"offAfter","offAfterS":15.0})";
    RaftRetCode result2 = motorControl->sendCmdJSON(timeoutCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result2);
    
    // Test pause command
    String pauseCmd = R"({"cmd":"pause"})";
    RaftRetCode result3 = motorControl->sendCmdJSON(pauseCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result3);
}

void test_status_reporting_compatibility(void) {
    setupMotorControl();
    
    // Test existing status JSON
    String statusJson = motorControl->getDataJSON(DEVICE_JSON_LEVEL_MED);
    TEST_ASSERT_TRUE(statusJson.length() > 0);
    
    // Test named value queries
    bool isFresh;
    double busyValue = motorControl->getNamedValue("b", isFresh);
    TEST_ASSERT_TRUE(isFresh);
    TEST_ASSERT_TRUE(busyValue == 0.0 || busyValue == 1.0); // Boolean-like
}

void test_new_control_mode_parsing_but_rejection(void) {
    setupMotorControl();
    
    // Phase 1: New velocity format should parse but not execute
    String velCmd = R"({
        "cmd": "motion",
        "mode": "velocity", 
        "velocityTargets": [{"a": 0, "v": 50.0}],
        "duration": 5000
    })";
    
    RaftRetCode result = motorControl->sendCmdJSON(velCmd.c_str());
    TEST_ASSERT_NOT_EQUAL(RAFT_OK, result); // Should reject - not implemented yet
}

void test_configuration_format_compatibility(void) {
    // Test that existing configuration format works
    String config = R"({
        "name": "MotorControl",
        "type": "MotorControl",
        "axes": [
            {
                "name": "X",
                "params": {"maxSpeed": 1000, "stepsPerUnit": 80},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15}
            }
        ]
    })";
    
    MotorControl testControl("TestControl", config.c_str());
    testControl.setup();
    
    // Should not crash and should be operational
    TEST_ASSERT_FALSE(testControl.isBusy());
}
```

#### **Validation Criteria**
- [ ] All existing JSON commands work unchanged
- [ ] Enhanced MotionArgs parsing transparent to users
- [ ] Non-motion commands preserved
- [ ] Status reporting unchanged
- [ ] Configuration backward compatible

---

## System Integration Test Harnesses

### **7.1 Full System Integration Test**

**Purpose**: Validate complete end-to-end system functionality

**Test File**: `unit_tests/main/FullSystemIntegration_test.cpp`

#### **Performance Benchmarking Infrastructure**
```cpp
class SystemPerformanceBenchmark {
private:
    uint32_t _startTime;
    uint32_t _commandCount;
    
public:
    void startBenchmark() {
        _startTime = millis();
        _commandCount = 0;
    }
    
    void recordCommand() {
        _commandCount++;
    }
    
    void endBenchmark(const char* testName) {
        uint32_t duration = millis() - _startTime;
        float commandsPerSecond = (_commandCount * 1000.0f) / duration;
        
        LOG_I("Benchmark", "%s: %d commands in %dms (%.2f cmd/s)", 
              testName, _commandCount, duration, commandsPerSecond);
    }
};
```

#### **Test Cases**
```cpp
void test_complete_system_operation(void) {
    setupCompleteSystem();
    
    // Test comprehensive motion sequence
    String commands[] = {
        R"({"cmd":"motion","pos":[{"a":0,"p":100}]})",
        R"({"cmd":"motion","pos":[{"a":0,"p":50},{"a":1,"p":25}]})",
        R"({"cmd":"motion","relative":true,"pos":[{"a":0,"p":10}]})",
        R"({"cmd":"maxCurrent","axisIdx":0,"maxCurrentA":1.2})",
        R"({"cmd":"pause"})",
        R"({"cmd":"resume"})"
    };
    
    for (const String& cmd : commands) {
        RaftRetCode result = system->sendCmdJSON(cmd.c_str());
        TEST_ASSERT_EQUAL(RAFT_OK, result);
    }
}

void test_architecture_layer_integration(void) {
    setupCompleteSystem();
    
    // Verify all layers are working together
    
    // 1. Factory created correct driver
    // 2. Driver integrated with MotionController
    // 3. JSON commands route correctly
    // 4. Status reporting works
    
    String statusJson = system->getDataJSON(DEVICE_JSON_LEVEL_FULL);
    TEST_ASSERT_TRUE(statusJson.length() > 0);
}

void test_performance_benchmarks(void) {
    setupCompleteSystem();
    SystemPerformanceBenchmark benchmark;
    
    benchmark.startBenchmark();
    
    // Send rapid sequence of commands
    for (int i = 0; i < 100; i++) {
        String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":)" + String(i) + R"(}]})";
        RaftRetCode result = system->sendCmdJSON(cmd.c_str());
        TEST_ASSERT_EQUAL(RAFT_OK, result);
        benchmark.recordCommand();
    }
    
    benchmark.endBenchmark("Command Processing Performance");
    
    // Should handle at least 100 commands per second
    // Actual verification would need timing measurements
}

void test_memory_usage_stability(void) {
    setupCompleteSystem();
    
    // Monitor memory usage over extended operation
    uint32_t initialMemory = ESP.getFreeHeap();
    
    // Send many commands and verify no memory leaks
    for (int i = 0; i < 1000; i++) {
        String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":1}]})";
        system->sendCmdJSON(cmd.c_str());
        
        // Process some loops
        for (int j = 0; j < 10; j++) {
            system->loop();
        }
    }
    
    uint32_t finalMemory = ESP.getFreeHeap();
    uint32_t memoryDelta = initialMemory - finalMemory;
    
    // Should not leak significant memory (< 1KB tolerance)
    TEST_ASSERT_TRUE(memoryDelta < 1024);
}

void test_error_recovery_and_robustness(void) {
    setupCompleteSystem();
    
    // Test various error conditions
    String errorCommands[] = {
        R"({"cmd":"motion","pos":[{"a":10,"p":100}]})",  // Invalid axis
        R"({"cmd":"motion","pos":[{invalid}]})",         // Invalid JSON
        R"({"cmd":"unknownCommand"})",                   // Unknown command
        R"({"invalid":"json})"                           // Malformed JSON
    };
    
    for (const String& cmd : errorCommands) {
        RaftRetCode result = system->sendCmdJSON(cmd.c_str());
        // Should handle gracefully, not crash
    }
    
    // System should still be responsive after errors
    String validCmd = R"({"cmd":"motion","pos":[{"a":0,"p":10}]})";
    RaftRetCode result = system->sendCmdJSON(validCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
}

void test_multi_axis_coordination(void) {
    setupMultiAxisSystem(3); // 3-axis system
    
    // Test coordinated multi-axis motion
    String coordCmd = R"({
        "cmd": "motion",
        "pos": [
            {"a": 0, "p": 100},
            {"a": 1, "p": 50},
            {"a": 2, "p": -25}
        ]
    })";
    
    RaftRetCode result = system->sendCmdJSON(coordCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
    
    // Verify all axes are coordinated
    // (Implementation specific verification)
}

void test_real_time_responsiveness(void) {
    setupCompleteSystem();
    
    // Test that system responds within timing constraints
    uint32_t maxResponseTime = 0;
    
    for (int i = 0; i < 50; i++) {
        uint32_t startTime = micros();
        
        String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":)" + String(i) + R"(}]})";
        system->sendCmdJSON(cmd.c_str());
        
        uint32_t responseTime = micros() - startTime;
        if (responseTime > maxResponseTime) {
            maxResponseTime = responseTime;
        }
    }
    
    // Should respond within 1ms for most commands
    TEST_ASSERT_TRUE(maxResponseTime < 1000); // 1ms in microseconds
}
```

#### **Validation Criteria**
- [ ] Complete end-to-end functionality operational
- [ ] All architecture layers properly integrated
- [ ] Performance meets or exceeds requirements
- [ ] Memory usage stable over extended operation
- [ ] Error handling robust and recovery complete
- [ ] Real-time responsiveness maintained

---

## Test Execution Strategy

### **Test Execution Order**
1. **Unit Tests**: Individual class validation
2. **Component Tests**: Layer-specific functionality
3. **Integration Tests**: Cross-layer communication
4. **System Tests**: End-to-end functionality

### **Continuous Validation**
- Run unit tests on every code change
- Run component tests on layer completion
- Run integration tests on phase completion
- Run system tests on milestone completion

### **Test Coverage Requirements**
- Unit Tests: >90% code coverage
- Component Tests: >85% functionality coverage
- Integration Tests: >95% interface coverage
- System Tests: 100% user scenario coverage

### **Performance Testing Requirements**
- Command processing: <1ms response time
- Memory usage: <1KB growth over 1000 operations
- System throughput: >100 commands/second
- Real-time constraints: Maintained under load

This comprehensive test harness specification ensures robust validation of the new architecture while maintaining complete backward compatibility and preparing for future enhancements.