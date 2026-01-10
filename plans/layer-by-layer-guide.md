# Layer-by-Layer Implementation Guide

## Overview

This guide provides detailed, step-by-step instructions for implementing the RaftMotorControl enhancements using a bottom-up approach with test harnesses at each architectural layer.

## Architecture Layers (Bottom to Top)

```
Layer 6: Application Interface (MotorControl class)
         ↑
Layer 5: Control Coordination (MotionController)  
         ↑
Layer 4: Factory & Integration (MotorDriverFactory, Integration)
         ↑
Layer 3: Motor Driver Implementation (MotorDriverBasicStepper)
         ↑  
Layer 2: Abstract Motor Interface (MotorDriverBase)
         ↑
Layer 1: Step/Direction Hardware Interface (StepDriverStepDirectionBase)
```

---

## Layer 1: Step/Direction Hardware Interface Foundation

### **Objective**: Create clean step/direction hardware abstraction

### **1.1 Step Driver Step Direction Base**

**File**: `components/MotorControl/Steppers/StepDriverStepDirectionBase.h` (rename from StepDriverBase.h)

#### **Step 1.1.1: Rename and Refactor StepDriverBase**

```cpp
#pragma once

#include <stdint.h>
#include "RaftUtils.h"
#include "StepDriverParams.h"

class RaftBus;

class StepDriverStepDirectionBase
{
public:
    StepDriverStepDirectionBase();
    virtual ~StepDriverStepDirectionBase();

    // Setup
    virtual bool setup(const String& stepperName, const StepDriverParams& stepperParams, bool usingISR);

    // Called after bus has been connected
    virtual void setupSerialBus(RaftBus* pBus, bool useBusForDirectionReversal);

    // Loop - called frequently
    virtual void loop();

    // Microsteps
    virtual void setMicrosteps(uint32_t microsteps) = 0;

    // Direction
    virtual void setDirection(bool dirn, bool forceSet = false) = 0;

    // Start and end a single step
    virtual void stepStart() = 0;
    virtual bool stepEnd() = 0;

    // Hardware specific
    virtual uint32_t getSerialAddress() const
    {
        return _serialBusAddress;
    }

    virtual String getDriverType() const = 0;

    virtual void setMaxMotorCurrentAmps(float maxMotorCurrentAmps) = 0;

    virtual String getDebugJSON(bool includeBraces, bool detailed) const = 0;

    virtual String getStatusJSON(bool includeBraces, bool detailed) const = 0;

    virtual bool isOperatingOk() const = 0;

protected:
    // All existing StepDriverBase protected/private members remain unchanged
    // ... (existing implementation details)
    
private:
    // Existing private members
    String _name;
    bool _hwIsSetup = false;
    uint8_t _serialBusAddress = 0;
    RaftBus* _pSerialBus = nullptr;
    
    // Debug
    static constexpr const char* MODULE_PREFIX = "StepDriverStepDirectionBase";    
};
```

#### **Step 1.1.2: Update StepDriverTMC2209Enhanced**

**File**: `components/MotorControl/Steppers/StepDriverTMC2209Enhanced.h`

```cpp
// Update inheritance and remove high-level methods
class StepDriverTMC2209Enhanced : public StepDriverStepDirectionBase {
public:
    // Constructor
    StepDriverTMC2209Enhanced();

    // StepDriverStepDirectionBase implementation
    virtual bool setup(const String& stepperName, const StepDriverParams& stepperParams, bool usingISR) override final;
    virtual void loop() override final;
    virtual void setMicrosteps(uint32_t microsteps) override final;
    virtual void setDirection(bool dirn, bool forceSet = false) override final;
    virtual void stepStart() override final;
    virtual bool stepEnd() override final;

    virtual String getDriverType() const override final
    {
        return "TMC2209Enhanced";
    }

    virtual String getDebugJSON(bool includeBraces, bool detailed) const override final;
    virtual String getStatusJSON(bool includeBraces, bool detailed) const override final;
    virtual void setMaxMotorCurrentAmps(float maxMotorCurrentAmps) override final;
    virtual bool isOperatingOk() const override final;

    // Remove high-level interface methods:
    // - Remove setPositionTarget, setVelocityTarget, setTorqueTarget
    // - Remove getCurrentPosition, getCurrentVelocity, getCurrentTorque
    // - Remove hasError (conflicted with lower-level error handling)

private:
    // Keep all existing private implementation unchanged
    // This is just an interface change, not implementation change
};
```

#### **Step 1.1.3: Create Test Harness for Step Direction Base**

**File**: `unit_tests/main/StepDriverStepDirectionBase_test.cpp`

```cpp
#include "unity.h"
#include "StepDriverStepDirectionBase.h"
#include "StepDriverParams.h"

// Test implementation for validation
class TestStepDriver : public StepDriverStepDirectionBase {
private:
    bool _stepActive = false;
    bool _direction = false;
    uint32_t _microsteps = 16;
    
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
        return "TestDriver";
    }

    void setMaxMotorCurrentAmps(float maxMotorCurrentAmps) override {
        // Test implementation
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
};

static TestStepDriver* testDriver = nullptr;

void setUp(void) {
    testDriver = new TestStepDriver();
}

void tearDown(void) {
    if (testDriver) {
        delete testDriver;
        testDriver = nullptr;
    }
}

void test_step_direction_interface(void) {
    TEST_ASSERT_NOT_NULL(testDriver);
    
    // Test stepping
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

void test_microstep_setting(void) {
    testDriver->setMicrosteps(32);
    TEST_ASSERT_EQUAL(32, testDriver->getMicrosteps());
    
    testDriver->setMicrosteps(8);
    TEST_ASSERT_EQUAL(8, testDriver->getMicrosteps());
}

void test_driver_type_reporting(void) {
    String driverType = testDriver->getDriverType();
    TEST_ASSERT_TRUE(driverType == "TestDriver");
}

void test_status_json_generation(void) {
    String statusJson = testDriver->getStatusJSON(true, false);
    TEST_ASSERT_TRUE(statusJson.length() > 0);
    TEST_ASSERT_TRUE(statusJson.indexOf("status") >= 0);
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_step_direction_interface);
    RUN_TEST(test_direction_control);
    RUN_TEST(test_microstep_setting);
    RUN_TEST(test_driver_type_reporting);
    RUN_TEST(test_status_json_generation);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

### **1.2 Validation Criteria**
- [ ] StepDriverStepDirectionBase interface is clean and focused on step/direction control
- [ ] StepDriverTMC2209Enhanced compiles and works with new interface
- [ ] All existing step/direction functionality preserved
- [ ] Test harness validates core step/direction operations
- [ ] No high-level motor control concepts in this layer

---

## Layer 2: Abstract Motor Interface

### **Objective**: Create high-level motor driver abstraction

### **2.1 Motor Driver Base Interface**

**File**: `components/MotorControl/MotorDriver/MotorDriverBase.h` (new folder/file)

#### **Step 2.1.1: Create Abstract Motor Driver Interface**

```cpp
#pragma once

#include <stdint.h>
#include "RaftJsonIF.h"

class MotorDriverBase {
public:
    enum class MotorType { 
        BASIC_STEPPER,          // Step/direction steppers (TMC2209, etc.)
        SERVO_STEPPER,          // Steppers with encoder feedback
        SERVO,                  // RC/Industrial servos
        BLDC                    // Brushless DC motors
    };
    
    enum class ControlMode { 
        POSITION,               // Position control
        VELOCITY,               // Velocity control  
        TORQUE                  // Torque/current control
    };
    
    struct MotorCapabilities {
        MotorType motorType;
        bool supportsPositionMode;
        bool supportsVelocityMode;
        bool supportsTorqueMode;
        bool hasPositionFeedback;
        bool hasVelocityFeedback;  
        bool hasTorqueFeedback;
        float maxVelocity;          // units/sec
        float maxTorque;            // Nm or equivalent
        float positionResolution;   // units per step/count
        uint32_t numAxes;          // Number of axes this driver handles
    };
    
    // Virtual destructor
    virtual ~MotorDriverBase() = default;
    
    // Capability queries
    virtual MotorCapabilities getCapabilities() const = 0;
    virtual bool supportsControlMode(ControlMode mode) const = 0;
    
    // High-level control interface (multi-axis)
    virtual bool setPositionTarget(uint32_t axisIdx, float position) = 0;
    virtual bool setVelocityTarget(uint32_t axisIdx, float velocity) = 0;
    virtual bool setTorqueTarget(uint32_t axisIdx, float torque) = 0;
    
    // Feedback interface  
    virtual float getCurrentPosition(uint32_t axisIdx) const = 0;
    virtual float getCurrentVelocity(uint32_t axisIdx) const = 0;
    virtual float getCurrentTorque(uint32_t axisIdx) const = 0;
    
    // Status and control
    virtual bool isEnabled() const = 0;
    virtual void setEnabled(bool enabled) = 0;
    virtual bool isMoving() const = 0;
    virtual bool hasError() const = 0;
    virtual String getStatusString() const = 0;
    
    // Multi-axis coordination and lifecycle
    virtual void setup(const RaftJsonIF& config, uint32_t numAxes) = 0;
    virtual void loop() = 0;
    virtual bool isBusy() const = 0;
    virtual void stop() = 0;
};
```

#### **Step 2.1.2: Create Test Harness for Motor Driver Base**

**File**: `unit_tests/main/MotorDriverBase_test.cpp`

```cpp
#include "unity.h"
#include "MotorDriverBase.h"

// Mock implementation for testing
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
    
    // MotorDriverBase implementation
    MotorCapabilities getCapabilities() const override {
        return _caps;
    }
    
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
    
    bool setVelocityTarget(uint32_t axisIdx, float velocity) override {
        if (axisIdx >= _numAxes) return false;
        if (!_caps.supportsVelocityMode) return false;
        _velocities[axisIdx] = velocity;
        return true;
    }
    
    bool setTorqueTarget(uint32_t axisIdx, float torque) override {
        if (axisIdx >= _numAxes) return false;
        if (!_caps.supportsTorqueMode) return false;
        _torques[axisIdx] = torque;
        return true;
    }
    
    float getCurrentPosition(uint32_t axisIdx) const override {
        if (axisIdx >= _numAxes) return 0.0f;
        return _positions[axisIdx];
    }
    
    float getCurrentVelocity(uint32_t axisIdx) const override {
        if (axisIdx >= _numAxes) return 0.0f;
        return _velocities[axisIdx];
    }
    
    float getCurrentTorque(uint32_t axisIdx) const override {
        if (axisIdx >= _numAxes) return 0.0f;
        return _torques[axisIdx];
    }
    
    bool isEnabled() const override { return _enabled; }
    void setEnabled(bool enabled) override { _enabled = enabled; }
    bool isMoving() const override { return false; }
    bool hasError() const override { return _hasError; }
    String getStatusString() const override { return "MockDriver"; }
    
    void setup(const RaftJsonIF& config, uint32_t numAxes) override {
        _numAxes = numAxes;
        _caps.numAxes = numAxes;
    }
    
    void loop() override { /* Mock implementation */ }
    bool isBusy() const override { return false; }
    void stop() override { /* Mock implementation */ }
    
    // Test helpers
    void setCapabilities(const MotorCapabilities& caps) { _caps = caps; }
    void setHasError(bool hasError) { _hasError = hasError; }
};

static MockMotorDriver* mockDriver = nullptr;

void setUp(void) {
    mockDriver = new MockMotorDriver();
}

void tearDown(void) {
    if (mockDriver) {
        delete mockDriver;
        mockDriver = nullptr;
    }
}

void test_motor_capabilities_query(void) {
    auto caps = mockDriver->getCapabilities();
    
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, caps.motorType);
    TEST_ASSERT_TRUE(caps.supportsPositionMode);
    TEST_ASSERT_FALSE(caps.supportsVelocityMode);
    TEST_ASSERT_FALSE(caps.supportsTorqueMode);
    TEST_ASSERT_EQUAL(3, caps.numAxes);
}

void test_control_mode_support(void) {
    TEST_ASSERT_TRUE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
    TEST_ASSERT_FALSE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::VELOCITY));
    TEST_ASSERT_FALSE(mockDriver->supportsControlMode(MotorDriverBase::ControlMode::TORQUE));
}

void test_position_control(void) {
    TEST_ASSERT_TRUE(mockDriver->setPositionTarget(0, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, mockDriver->getCurrentPosition(0));
    
    TEST_ASSERT_TRUE(mockDriver->setPositionTarget(1, -50.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.0f, mockDriver->getCurrentPosition(1));
}

void test_velocity_control_not_supported(void) {
    // Should fail because mock driver doesn't support velocity mode
    TEST_ASSERT_FALSE(mockDriver->setVelocityTarget(0, 25.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mockDriver->getCurrentVelocity(0));
}

void test_axis_bounds_checking(void) {
    // Invalid axis should return false/0.0
    TEST_ASSERT_FALSE(mockDriver->setPositionTarget(5, 100.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mockDriver->getCurrentPosition(5));
}

void test_enable_disable_control(void) {
    TEST_ASSERT_FALSE(mockDriver->isEnabled());
    
    mockDriver->setEnabled(true);
    TEST_ASSERT_TRUE(mockDriver->isEnabled());
    
    mockDriver->setEnabled(false);
    TEST_ASSERT_FALSE(mockDriver->isEnabled());
}

void test_error_handling(void) {
    TEST_ASSERT_FALSE(mockDriver->hasError());
    
    mockDriver->setHasError(true);
    TEST_ASSERT_TRUE(mockDriver->hasError());
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_motor_capabilities_query);
    RUN_TEST(test_control_mode_support);
    RUN_TEST(test_position_control);
    RUN_TEST(test_velocity_control_not_supported);
    RUN_TEST(test_axis_bounds_checking);
    RUN_TEST(test_enable_disable_control);
    RUN_TEST(test_error_handling);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

### **2.2 Validation Criteria**
- [ ] MotorDriverBase interface defines clean high-level abstraction
- [ ] Capability system works correctly
- [ ] Multi-axis support properly abstracted
- [ ] Interface is motor-type agnostic
- [ ] Mock implementation demonstrates interface usage

---

## Layer 3: Motor Driver Implementation

### **Objective**: Implement MotorDriverBasicStepper that encapsulates RampGenerator

### **3.1 Basic Stepper Motor Driver**

**File**: `components/MotorControl/MotorDriver/MotorDriverBasicStepper.h` (new file)

#### **Step 3.1.1: Create Basic Stepper Driver Implementation**

```cpp
#pragma once

#include "MotorDriverBase.h"
#include "BasicStepperRampGenerator.h"
#include "StepDriverStepDirectionBase.h"
#include "AxesParams.h"
#include <vector>

class MotorDriverBasicStepper : public MotorDriverBase {
private:
    BasicStepperRampGenerator _rampGenerator;
    std::vector<StepDriverStepDirectionBase*> _stepDrivers;
    AxesParams _axesParams;
    uint32_t _numAxes = 0;
    bool _isSetup = false;
    bool _enabled = false;
    
public:
    MotorDriverBasicStepper();
    virtual ~MotorDriverBasicStepper();
    
    // MotorDriverBase implementation
    MotorCapabilities getCapabilities() const override;
    bool supportsControlMode(ControlMode mode) const override;
    
    // Control interface
    bool setPositionTarget(uint32_t axisIdx, float position) override;
    bool setVelocityTarget(uint32_t axisIdx, float velocity) override;
    bool setTorqueTarget(uint32_t axisIdx, float torque) override;
    
    // Feedback interface
    float getCurrentPosition(uint32_t axisIdx) const override;
    float getCurrentVelocity(uint32_t axisIdx) const override;
    float getCurrentTorque(uint32_t axisIdx) const override;
    
    // Status and control
    bool isEnabled() const override { return _enabled; }
    void setEnabled(bool enabled) override;
    bool isMoving() const override;
    bool hasError() const override;
    String getStatusString() const override;
    
    // Multi-axis coordination
    void setup(const RaftJsonIF& config, uint32_t numAxes) override;
    void loop() override;
    bool isBusy() const override;
    void stop() override;
    
private:
    void setupStepDrivers(const RaftJsonIF& config);
    bool validateAxisIndex(uint32_t axisIdx) const;
    void convertPositionToSteps(uint32_t axisIdx, float position, int32_t& steps) const;
    void convertStepsToPosition(uint32_t axisIdx, int32_t steps, float& position) const;
};
```

#### **Step 3.1.2: Create BasicStepperRampGenerator (rename RampGenerator)**

**File**: `components/MotorControl/RampGenerator/BasicStepperRampGenerator.h` (rename from RampGenerator.h)

```cpp
// Rename existing RampGenerator to BasicStepperRampGenerator
// Keep ALL existing functionality - this is just a rename to clarify scope
#pragma once

#include "Logger.h"
#include "MotionBlock.h"
#include "RampGenStats.h"
#include "RampGenTimer.h"
#include "MotionPipeline.h"

class RampGenTimer;
class StepDriverStepDirectionBase; // Updated from StepDriverBase
class EndStops;

class BasicStepperRampGenerator
{
public:
    // Constructor / destructor
    BasicStepperRampGenerator();
    virtual ~BasicStepperRampGenerator();

    // Setup ramp generator
    void setup(const RaftJsonIF& config, 
            const std::vector<StepDriverStepDirectionBase*>& stepperDrivers, // Updated type
            const std::vector<EndStops*>& axisEndStops);

    // All existing methods remain unchanged - just interface type updates
    void loop();
    void start();
    void stop();
    void pause(bool pauseIt);
    
    // ... all other existing methods unchanged
    
private:
    // All existing private implementation unchanged
    std::vector<StepDriverStepDirectionBase*> _stepperDrivers; // Updated type
    
    // ... rest of implementation identical to current RampGenerator
};
```

#### **Step 3.1.3: Implement MotorDriverBasicStepper**

**File**: `components/MotorControl/MotorDriver/MotorDriverBasicStepper.cpp`

```cpp
#include "MotorDriverBasicStepper.h"
#include "StepDriverTMC2209Enhanced.h"
#include "MotionArgs.h"

MotorDriverBasicStepper::MotorDriverBasicStepper() {
    // Constructor
}

MotorDriverBasicStepper::~MotorDriverBasicStepper() {
    // Clean up step drivers
    for (auto* driver : _stepDrivers) {
        if (driver) {
            delete driver;
        }
    }
    _stepDrivers.clear();
}

MotorDriverBase::MotorCapabilities MotorDriverBasicStepper::getCapabilities() const {
    MotorCapabilities caps;
    caps.motorType = MotorType::BASIC_STEPPER;
    caps.supportsPositionMode = true;
    caps.supportsVelocityMode = false; // Limited velocity support
    caps.supportsTorqueMode = false;   // No torque control
    caps.hasPositionFeedback = false;  // Step counting only
    caps.hasVelocityFeedback = false;
    caps.hasTorqueFeedback = false;
    caps.numAxes = _numAxes;
    
    if (_numAxes > 0 && _isSetup) {
        caps.maxVelocity = _axesParams.getMaxSpeed(0); // Get from axis params
        caps.positionResolution = 1.0f / _axesParams.getStepsPerUnit(0);
    } else {
        caps.maxVelocity = 1000.0f; // Default
        caps.positionResolution = 0.0125f; // Default 80 steps/unit
    }
    
    return caps;
}

bool MotorDriverBasicStepper::supportsControlMode(ControlMode mode) const {
    return mode == ControlMode::POSITION;
}

bool MotorDriverBasicStepper::setPositionTarget(uint32_t axisIdx, float position) {
    if (!validateAxisIndex(axisIdx) || !_isSetup) {
        return false;
    }
    
    // Create MotionArgs for position command
    MotionArgs args;
    args.setControlMode(MotorDriverBase::ControlMode::POSITION);
    args.getAxesPos().setVal(axisIdx, position);
    args.getAxesSpecified().setVal(axisIdx, true);
    
    // Use existing motion planning through RampGenerator
    // This delegates to the existing motion pipeline
    return true; // Implementation would integrate with MotionBlockManager
}

bool MotorDriverBasicStepper::setVelocityTarget(uint32_t axisIdx, float velocity) {
    // Basic steppers have limited velocity control
    // For now, not supported - would require real-time step rate control
    return false;
}

bool MotorDriverBasicStepper::setTorqueTarget(uint32_t axisIdx, float torque) {
    // Basic steppers don't support torque control
    return false;
}

float MotorDriverBasicStepper::getCurrentPosition(uint32_t axisIdx) const {
    if (!validateAxisIndex(axisIdx) || !_isSetup) {
        return 0.0f;
    }
    
    // Get step position from RampGenerator and convert to units
    AxesValues<AxisStepsDataType> stepPos;
    _rampGenerator.getTotalStepPosition(stepPos);
    
    float position;
    convertStepsToPosition(axisIdx, stepPos.getVal(axisIdx), position);
    return position;
}

float MotorDriverBasicStepper::getCurrentVelocity(uint32_t axisIdx) const {
    // Basic steppers don't have velocity feedback
    // Could estimate from step rate, but not implemented for Phase 1
    return 0.0f;
}

float MotorDriverBasicStepper::getCurrentTorque(uint32_t axisIdx) const {
    // Basic steppers don't have torque feedback
    return 0.0f;
}

void MotorDriverBasicStepper::setEnabled(bool enabled) {
    _enabled = enabled;
    // Delegate to step drivers
    for (auto* driver : _stepDrivers) {
        if (driver) {
            // StepDriverStepDirectionBase doesn't have setEnabled, 
            // but can implement through setup or other means
        }
    }
}

bool MotorDriverBasicStepper::isMoving() const {
    return _rampGenerator.getMotionPipelineConst().count() > 0;
}

bool MotorDriverBasicStepper::hasError() const {
    if (!_isSetup) return true;
    
    // Check step drivers for errors
    for (auto* driver : _stepDrivers) {
        if (driver && !driver->isOperatingOk()) {
            return true;
        }
    }
    return false;
}

String MotorDriverBasicStepper::getStatusString() const {
    return "BasicStepper(" + String(_numAxes) + " axes)";
}

void MotorDriverBasicStepper::setup(const RaftJsonIF& config, uint32_t numAxes) {
    _numAxes = numAxes;
    
    // Setup axes parameters
    _axesParams.setupAxes(config);
    
    // Setup step drivers
    setupStepDrivers(config);
    
    // Setup ramp generator
    RaftJsonPrefixed rampConfig(config, "ramp");
    _rampGenerator.setup(rampConfig, _stepDrivers, {}); // Empty endstops for now
    _rampGenerator.start();
    
    _isSetup = true;
}

void MotorDriverBasicStepper::loop() {
    if (!_isSetup) return;
    
    // Loop step drivers
    for (auto* driver : _stepDrivers) {
        if (driver) {
            driver->loop();
        }
    }
    
    // Loop ramp generator
    _rampGenerator.loop();
}

bool MotorDriverBasicStepper::isBusy() const {
    return isMoving();
}

void MotorDriverBasicStepper::stop() {
    _rampGenerator.stop();
}

void MotorDriverBasicStepper::setupStepDrivers(const RaftJsonIF& config) {
    _stepDrivers.resize(_numAxes);
    
    // Extract hardware related to axes
    std::vector<String> axesVec;
    if (config.getArrayElems("axes", axesVec)) {
        uint32_t axisIdx = 0;
        for (RaftJson axisConfig : axesVec) {
            if (axisIdx >= _numAxes) break;
            
            // Create step driver based on configuration
            // For Phase 1, hardcode TMC2209Enhanced
            String driverType = axisConfig.getString("driver.driver", "TMC2209");
            if (driverType.equalsIgnoreCase("TMC2209")) {
                auto* stepDriver = new StepDriverTMC2209Enhanced();
                
                // Setup step driver parameters
                StepDriverParams stepperParams(axisConfig.getObject("driver"));
                String axisName = axisConfig.getString("name", "Axis" + String(axisIdx));
                
                stepDriver->setup(axisName, stepperParams, _rampGenerator.isUsingTimerISR());
                _stepDrivers[axisIdx] = stepDriver;
            }
            
            axisIdx++;
        }
    }
}

bool MotorDriverBasicStepper::validateAxisIndex(uint32_t axisIdx) const {
    return axisIdx < _numAxes;
}

void MotorDriverBasicStepper::convertPositionToSteps(uint32_t axisIdx, float position, int32_t& steps) const {
    if (validateAxisIndex(axisIdx)) {
        float stepsPerUnit = _axesParams.getStepsPerUnit(axisIdx);
        steps = static_cast<int32_t>(position * stepsPerUnit);
    } else {
        steps = 0;
    }
}

void MotorDriverBasicStepper::convertStepsToPosition(uint32_t axisIdx, int32_t steps, float& position) const {
    if (validateAxisIndex(axisIdx)) {
        float stepsPerUnit = _axesParams.getStepsPerUnit(axisIdx);
        position = static_cast<float>(steps) / stepsPerUnit;
    } else {
        position = 0.0f;
    }
}
```

#### **Step 3.1.4: Create Test Harness for Basic Stepper Driver**

**File**: `unit_tests/main/MotorDriverBasicStepper_test.cpp`

```cpp
#include "unity.h"
#include "MotorDriverBasicStepper.h"
#include "RaftJson.h"

static MotorDriverBasicStepper* basicStepperDriver = nullptr;

void setUp(void) {
    basicStepperDriver = new MotorDriverBasicStepper();
}

void tearDown(void) {
    if (basicStepperDriver) {
        delete basicStepperDriver;
        basicStepperDriver = nullptr;
    }
}

void test_basic_stepper_capabilities(void) {
    auto caps = basicStepperDriver->getCapabilities();
    
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, caps.motorType);
    TEST_ASSERT_TRUE(caps.supportsPositionMode);
    TEST_ASSERT_FALSE(caps.supportsVelocityMode);
    TEST_ASSERT_FALSE(caps.supportsTorqueMode);
    TEST_ASSERT_FALSE(caps.hasPositionFeedback); // Step counting only
}

void test_control_mode_support(void) {
    TEST_ASSERT_TRUE(basicStepperDriver->supportsControlMode(MotorDriverBase::ControlMode::POSITION));
    TEST_ASSERT_FALSE(basicStepperDriver->supportsControlMode(MotorDriverBase::ControlMode::VELOCITY));
    TEST_ASSERT_FALSE(basicStepperDriver->supportsControlMode(MotorDriverBase::ControlMode::TORQUE));
}

void test_setup_and_initialization(void) {
    String config = R"({
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
    basicStepperDriver->setup(jsonConfig, 1);
    
    auto caps = basicStepperDriver->getCapabilities();
    TEST_ASSERT_EQUAL(1, caps.numAxes);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1000.0f, caps.maxVelocity);
}

void test_position_control_interface(void) {
    // Setup first
    String config = R"({
        "axes": [{"name": "X", "params": {"stepsPerUnit": 80}}],
        "ramp": {"timerIndex": 0}
    })";
    RaftJson jsonConfig(config);
    basicStepperDriver->setup(jsonConfig, 1);
    
    // Test position setting
    bool result = basicStepperDriver->setPositionTarget(0, 100.0f);
    TEST_ASSERT_TRUE(result);
    
    // Test invalid axis
    result = basicStepperDriver->setPositionTarget(5, 100.0f);
    TEST_ASSERT_FALSE(result);
}

void test_velocity_control_not_supported(void) {
    String config = R"({"axes": [{"name": "X"}], "ramp": {}})";
    RaftJson jsonConfig(config);
    basicStepperDriver->setup(jsonConfig, 1);
    
    bool result = basicStepperDriver->setVelocityTarget(0, 50.0f);
    TEST_ASSERT_FALSE(result); // Should not be supported
}

void test_torque_control_not_supported(void) {
    String config = R"({"axes": [{"name": "X"}], "ramp": {}})";
    RaftJson jsonConfig(config);
    basicStepperDriver->setup(jsonConfig, 1);
    
    bool result = basicStepperDriver->setTorqueTarget(0, 0.5f);
    TEST_ASSERT_FALSE(result); // Should not be supported
}

void test_enable_disable_functionality(void) {
    TEST_ASSERT_FALSE(basicStepperDriver->isEnabled());
    
    basicStepperDriver->setEnabled(true);
    TEST_ASSERT_TRUE(basicStepperDriver->isEnabled());
    
    basicStepperDriver->setEnabled(false);
    TEST_ASSERT_FALSE(basicStepperDriver->isEnabled());
}

void test_status_reporting(void) {
    String status = basicStepperDriver->getStatusString();
    TEST_ASSERT_TRUE(status.length() > 0);
    TEST_ASSERT_TRUE(status.indexOf("BasicStepper") >= 0);
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_basic_stepper_capabilities);
    RUN_TEST(test_control_mode_support);
    RUN_TEST(test_setup_and_initialization);
    RUN_TEST(test_position_control_interface);
    RUN_TEST(test_velocity_control_not_supported);
    RUN_TEST(test_torque_control_not_supported);
    RUN_TEST(test_enable_disable_functionality);
    RUN_TEST(test_status_reporting);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

### **3.2 Validation Criteria**
- [ ] MotorDriverBasicStepper correctly implements MotorDriverBase interface
- [ ] RampGenerator functionality encapsulated and working
- [ ] Step drivers properly integrated
- [ ] Position control works as expected
- [ ] Velocity and torque control properly rejected
- [ ] Multi-axis support functional

---

## Layer 4: Factory & Integration

### **Objective**: Create factory pattern and integrate with configuration

### **4.1 Motor Driver Factory**

**File**: `components/MotorControl/MotorDriver/MotorDriverFactory.h`

#### **Step 4.1.1: Create Factory Interface**

```cpp
#pragma once

#include "MotorDriverBase.h"
#include "RaftJsonIF.h"
#include <memory>

class MotorDriverFactory {
public:
    // Factory interface
    static std::unique_ptr<MotorDriverBase> createDriver(
        const String& motorType,
        const RaftJsonIF& config,
        uint32_t numAxes
    );
    
    static std::unique_ptr<MotorDriverBase> createDriver(
        MotorDriverBase::MotorType type,
        const RaftJsonIF& config,
        uint32_t numAxes
    );
    
    // Auto-detection
    static MotorDriverBase::MotorType detectMotorType(
        const RaftJsonIF& config
    );
    
    static String getMotorTypeName(MotorDriverBase::MotorType type);
    static std::vector<String> getSupportedMotorTypes();
    
private:
    // Factory methods for different motor types
    static std::unique_ptr<MotorDriverBase> createBasicStepperDriver(
        const RaftJsonIF& config,
        uint32_t numAxes
    );
    
    // Future motor types
    // static std::unique_ptr<MotorDriverBase> createServoStepperDriver(...);
    // static std::unique_ptr<MotorDriverBase> createServoDriver(...);
    // static std::unique_ptr<MotorDriverBase> createBLDCDriver(...);
};
```

#### **Step 4.1.2: Implement Motor Driver Factory**

**File**: `components/MotorControl/MotorDriver/MotorDriverFactory.cpp`

```cpp
#include "MotorDriverFactory.h"
#include "MotorDriverBasicStepper.h"
#include "Logger.h"

static const char* MODULE_PREFIX = "MotorDriverFactory";

std::unique_ptr<MotorDriverBase> MotorDriverFactory::createDriver(
    const String& motorType,
    const RaftJsonIF& config,
    uint32_t numAxes
) {
    // Convert string to enum
    MotorDriverBase::MotorType type;
    
    if (motorType.equalsIgnoreCase("BasicStepper") || 
        motorType.equalsIgnoreCase("Stepper") ||
        motorType.equalsIgnoreCase("TMC2209")) {
        type = MotorDriverBase::MotorType::BASIC_STEPPER;
    }
    // Future motor types:
    // else if (motorType.equalsIgnoreCase("ServoStepper")) {
    //     type = MotorDriverBase::MotorType::SERVO_STEPPER;
    // }
    else {
        LOG_W(MODULE_PREFIX, "Unknown motor type '%s', defaulting to BasicStepper", 
              motorType.c_str());
        type = MotorDriverBase::MotorType::BASIC_STEPPER;
    }
    
    return createDriver(type, config, numAxes);
}

std::unique_ptr<MotorDriverBase> MotorDriverFactory::createDriver(
    MotorDriverBase::MotorType type,
    const RaftJsonIF& config,
    uint32_t numAxes
) {
    switch (type) {
        case MotorDriverBase::MotorType::BASIC_STEPPER:
            return createBasicStepperDriver(config, numAxes);
            
        // Future motor types:
        // case MotorDriverBase::MotorType::SERVO_STEPPER:
        //     return createServoStepperDriver(config, numAxes);
        
        default:
            LOG_E(MODULE_PREFIX, "Unsupported motor type: %d", static_cast<int>(type));
            return nullptr;
    }
}

MotorDriverBase::MotorType MotorDriverFactory::detectMotorType(
    const RaftJsonIF& config
) {
    // Look for clues in configuration
    String driverType = config.getString("axes[0].driver.driver", "TMC2209");
    bool hasEncoder = config.getBool("axes[0].driver.encoder", false);
    String motorType = config.getString("motorType", "");
    
    // Explicit motor type override
    if (!motorType.isEmpty()) {
        if (motorType.equalsIgnoreCase("BasicStepper")) {
            return MotorDriverBase::MotorType::BASIC_STEPPER;
        }
        if (motorType.equalsIgnoreCase("ServoStepper")) {
            return MotorDriverBase::MotorType::SERVO_STEPPER;
        }
        if (motorType.equalsIgnoreCase("Servo")) {
            return MotorDriverBase::MotorType::SERVO;
        }
        if (motorType.equalsIgnoreCase("BLDC")) {
            return MotorDriverBase::MotorType::BLDC;
        }
    }
    
    // Auto-detect based on driver configuration
    if (hasEncoder && driverType.equalsIgnoreCase("TMC2209")) {
        return MotorDriverBase::MotorType::SERVO_STEPPER;
    } else if (driverType.equalsIgnoreCase("TMC2209")) {
        return MotorDriverBase::MotorType::BASIC_STEPPER;
    }
    
    // Default to basic stepper for backward compatibility
    return MotorDriverBase::MotorType::BASIC_STEPPER;
}

String MotorDriverFactory::getMotorTypeName(MotorDriverBase::MotorType type) {
    switch (type) {
        case MotorDriverBase::MotorType::BASIC_STEPPER: return "BasicStepper";
        case MotorDriverBase::MotorType::SERVO_STEPPER: return "ServoStepper";
        case MotorDriverBase::MotorType::SERVO: return "Servo";
        case MotorDriverBase::MotorType::BLDC: return "BLDC";
        default: return "Unknown";
    }
}

std::vector<String> MotorDriverFactory::getSupportedMotorTypes() {
    return {
        "BasicStepper"
        // Future: "ServoStepper", "Servo", "BLDC"
    };
}

std::unique_ptr<MotorDriverBase> MotorDriverFactory::createBasicStepperDriver(
    const RaftJsonIF& config,
    uint32_t numAxes
) {
    auto driver = std::make_unique<MotorDriverBasicStepper>();
    
    try {
        driver->setup(config, numAxes);
        LOG_I(MODULE_PREFIX, "Created BasicStepper driver for %d axes", numAxes);
        return driver;
    }
    catch (const std::exception& e) {
        LOG_E(MODULE_PREFIX, "Failed to create BasicStepper driver: %s", e.what());
        return nullptr;
    }
}
```

#### **Step 4.1.3: Create Factory Test Harness**

**File**: `unit_tests/main/MotorDriverFactory_test.cpp`

```cpp
#include "unity.h"
#include "MotorDriverFactory.h"
#include "RaftJson.h"

void test_factory_create_basic_stepper(void) {
    String config = R"({
        "motorType": "BasicStepper",
        "axes": [
            {
                "name": "X",
                "params": {"stepsPerUnit": 80},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15}
            }
        ]
    })";
    
    RaftJson jsonConfig(config);
    auto driver = MotorDriverFactory::createDriver("BasicStepper", jsonConfig, 1);
    
    TEST_ASSERT_NOT_NULL(driver.get());
    
    auto caps = driver->getCapabilities();
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, caps.motorType);
    TEST_ASSERT_EQUAL(1, caps.numAxes);
}

void test_factory_create_by_enum(void) {
    String config = R"({
        "axes": [{"name": "X", "driver": {"driver": "TMC2209"}}]
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

void test_motor_type_detection(void) {
    // Test explicit motor type
    String explicitConfig = R"({"motorType": "BasicStepper"})";
    RaftJson explicitJson(explicitConfig);
    auto type1 = MotorDriverFactory::detectMotorType(explicitJson);
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, type1);
    
    // Test auto-detection from driver config
    String autoConfig = R"({
        "axes": [{"driver": {"driver": "TMC2209"}}]
    })";
    RaftJson autoJson(autoConfig);
    auto type2 = MotorDriverFactory::detectMotorType(autoJson);
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, type2);
    
    // Test servo stepper detection
    String servoConfig = R"({
        "axes": [{"driver": {"driver": "TMC2209", "encoder": true}}]
    })";
    RaftJson servoJson(servoConfig);
    auto type3 = MotorDriverFactory::detectMotorType(servoJson);
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::SERVO_STEPPER, type3);
}

void test_supported_motor_types(void) {
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

void test_invalid_motor_type_handling(void) {
    String config = R"({"axes": [{"driver": {"driver": "TMC2209"}}]})";
    RaftJson jsonConfig(config);
    
    auto driver = MotorDriverFactory::createDriver("NonExistentType", jsonConfig, 1);
    
    // Should default to BasicStepper
    TEST_ASSERT_NOT_NULL(driver.get());
    TEST_ASSERT_EQUAL(MotorDriverBase::MotorType::BASIC_STEPPER, 
                      driver->getCapabilities().motorType);
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_factory_create_basic_stepper);
    RUN_TEST(test_factory_create_by_enum);
    RUN_TEST(test_motor_type_detection);
    RUN_TEST(test_supported_motor_types);
    RUN_TEST(test_motor_type_name_conversion);
    RUN_TEST(test_invalid_motor_type_handling);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

### **4.2 Validation Criteria**
- [ ] Factory creates correct driver types based on configuration
- [ ] Auto-detection logic works properly
- [ ] Configuration parsing handles all scenarios
- [ ] Error handling for invalid configurations
- [ ] Extensible for future motor types

---

## Layer 5: Control Coordination Layer

### **Objective**: Integrate new architecture with MotionController

### **5.1 Enhanced Motion Controller Integration**

**File**: `components/MotorControl/Controller/MotionController.h` (modified)

#### **Step 5.1.1: Update MotionController Interface**

```cpp
// Add to existing MotionController class
class MotionController : public MotionControlIF {
private:
    // Existing components remain unchanged
    MotionBlockManager _blockManager;
    MotorEnabler _motorEnabler;
    MotionPatternManager _patternManager;
    AxesParams _axesParams;
    std::vector<EndStops*> _axisEndStops;
    
    // NEW: Replace StepDriverBase array and RampGenerator with MotorDriverBase
    std::unique_ptr<MotorDriverBase> _motorDriver;
    
    // REMOVED: These are now encapsulated in MotorDriverBasicStepper
    // std::vector<StepDriverBase*> _stepperDrivers;
    // RampGenerator _rampGenerator;
    
    // Status tracking
    bool _homingNeededBeforeAnyMove = true;
    bool _isPaused = false;
    
public:
    // Existing interface remains unchanged - backward compatibility
    bool moveTo(MotionArgs &args) override;
    void pause(bool pauseIt) override;
    bool isPaused() const override { return _isPaused; }
    bool isBusy() const override;
    
    // All other existing methods remain unchanged
    void setCurPositionAsOrigin(bool allAxes = true, uint32_t axisIdx = 0) override;
    AxesValues<AxisPosDataType> getLastCommandedPos() const override;
    AxesValues<AxisPosDataType> getLastMonitoredPos() const override;
    // ... etc
    
private:
    // Modified setup method
    void setupAxes(const RaftJsonIF& config);
    void setupMotorDriver(const RaftJsonIF& config);
    
    // Existing helper methods mostly unchanged
    void setupAxisHardware(uint32_t axisIdx, const RaftJsonIF& config);
    void setupEndStops(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig);
};
```

#### **Step 5.1.2: Update MotionController Implementation**

**File**: `components/MotorControl/Controller/MotionController.cpp` (modified)

```cpp
// Update key methods in MotionController.cpp

void MotionController::setup(const RaftJsonIF& config) {
    // De-init first
    deinit();

    // Setup axes parameters (unchanged)
    setupAxes(config);

    // NEW: Setup motor driver instead of individual stepper drivers
    setupMotorDriver(config);

    // Setup motor enabler (unchanged)
    RaftJsonPrefixed motorEnConfig(config, "motorEn");
    _motorEnabler.setup(motorEnConfig);

    // Block manager setup (modified to work with motor driver)
    RaftJsonPrefixed motionConfig(config, "motion");
    _blockManager.setup(_motorDriver->getCapabilities().maxVelocity, motionConfig);

    // If no homing required then set the current position as home
    if (!_homingNeededBeforeAnyMove)
        setCurPositionAsOrigin(true);
}

void MotionController::setupMotorDriver(const RaftJsonIF& config) {
    // Detect motor type from configuration
    auto motorType = MotorDriverFactory::detectMotorType(config);
    uint32_t numAxes = config.getArrayElems("axes").size();
    
    if (numAxes == 0) {
        LOG_E(MODULE_PREFIX, "No axes configured");
        return;
    }
    
    // Create motor driver via factory
    _motorDriver = MotorDriverFactory::createDriver(motorType, config, numAxes);
    
    if (!_motorDriver) {
        LOG_E(MODULE_PREFIX, "Failed to create motor driver");
        return;
    }
    
    LOG_I(MODULE_PREFIX, "Created %s motor driver for %d axes", 
          MotorDriverFactory::getMotorTypeName(motorType).c_str(),
          numAxes);
}

bool MotionController::moveTo(MotionArgs &args) {
    LOG_I(MODULE_PREFIX, "moveTo %s", args.toJSON().c_str());

    // Handle special commands (unchanged)
    if (args.isStopMotion()) {
        _motorDriver->stop();
        return true;
    }
    
    if (args.isClearQueue()) {
        _blockManager.clear();
        return true;
    }

    if (!args.isEnableMotors()) {
        _motorEnabler.enableMotors(false, false);
        return true;
    }

    // Route based on control mode - for Phase 1, only position mode supported
    switch (args.getControlMode()) {
        case MotorDriverBase::ControlMode::POSITION:
            return moveToPosition(args);
            
        case MotorDriverBase::ControlMode::VELOCITY:
        case MotorDriverBase::ControlMode::TORQUE:
            LOG_W(MODULE_PREFIX, "Real-time control not yet implemented");
            return false;
            
        default:
            LOG_E(MODULE_PREFIX, "Unknown control mode");
            return false;
    }
}

bool MotionController::moveToPosition(MotionArgs& args) {
    // This is the existing moveTo implementation, mostly unchanged
    
    // Check not busy
    if (_blockManager.isBusy()) {
        return false;
    }

    // Check if homing needed
    if (_blockManager.isHomingNeededBeforeMove() && (!_blockManager.isAxesStateValid())) {
        return false;
    }

    // Pre-process coordinates
    AxisPosDataType moveDistanceMM = _blockManager.preProcessCoords(args);

    // Block splitting logic (unchanged)
    uint32_t numBlocks = 1;
    double maxBlockDistMM = _axesParams.getMaxBlockDistMM();
    if (maxBlockDistMM > 0.01f && !args.dontSplitMove())
        numBlocks = int(ceil(moveDistanceMM / maxBlockDistMM));
    if (numBlocks == 0)
        numBlocks = 1;

    // Add to the block manager (unchanged)
    _blockManager.addRampedBlock(args, numBlocks);

    // Pump the block splitter to prime the pipeline
    _blockManager.pumpBlockSplitter(_motorDriver); // Changed from _rampGenerator

    return true;
}

void MotionController::loop() {
    // Motor driver loop (replaces individual stepper driver loops)
    if (_motorDriver) {
        _motorDriver->loop();
    }

    // Motor enabler loop (unchanged)
    _motorEnabler.loop();
    
    // Process split-up blocks (modified to use motor driver)
    _blockManager.pumpBlockSplitter(_motorDriver);

    // Loop motion patterns (unchanged)
    _patternManager.loop(*this);

    // Ensure motors enabled when moving (modified)
    if (_motorDriver && (_motorDriver->isBusy() || _patternManager.isPatternActive())) {
        _motorEnabler.enableMotors(true, false);
    }
}

bool MotionController::isBusy() const {
    return _motorDriver ? _motorDriver->isBusy() : false;
}

// Other methods need similar updates to work with _motorDriver instead of _rampGenerator
// This preserves the public interface while using the new architecture internally
```

### **5.2 Update MotionBlockManager Integration**

**File**: `components/MotorControl/Controller/MotionBlockManager.h` (minor update)

```cpp
// Update MotionBlockManager to work with MotorDriverBase instead of RampGenerator directly
class MotionBlockManager {
    // Most implementation unchanged, but pipeline pump method updated
public:
    void pumpBlockSplitter(MotorDriverBase* motorDriver);  // Changed parameter type
    
    // All other methods remain unchanged
};
```

#### **Step 5.2.1: Create Integration Test**

**File**: `unit_tests/main/MotionControllerIntegration_test.cpp`

```cpp
#include "unity.h"
#include "MotionController.h"
#include "MotionArgs.h"
#include "RaftJson.h"

static MotionController* motionController = nullptr;

void setUp(void) {
    motionController = new MotionController();
}

void tearDown(void) {
    if (motionController) {
        delete motionController;
        motionController = nullptr;
    }
}

void test_motion_controller_setup(void) {
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
    
    // Verify setup succeeded
    TEST_ASSERT_FALSE(motionController->isBusy());
    TEST_ASSERT_FALSE(motionController->isPaused());
}

void test_position_control_integration(void) {
    // Setup first
    String config = R"({
        "axes": [{"name": "X", "params": {"stepsPerUnit": 80}, "driver": {"driver": "TMC2209"}}],
        "ramp": {}
    })";
    RaftJson jsonConfig(config);
    motionController->setup(jsonConfig);
    
    // Test position command
    MotionArgs args;
    args.setControlMode(MotorDriverBase::ControlMode::POSITION);
    args.getAxesPos().setVal(0, 100.0f);
    args.getAxesSpecified().setVal(0, true);
    
    bool result = motionController->moveTo(args);
    TEST_ASSERT_TRUE(result);
}

void test_backward_compatibility(void) {
    // Test that existing MotionArgs without explicit control mode work
    String config = R"({
        "axes": [{"name": "X", "driver": {"driver": "TMC2209"}}]
    })";
    RaftJson jsonConfig(config);
    motionController->setup(jsonConfig);
    
    MotionArgs args;
    // Don't set control mode - should default to position
    args.getAxesPos().setVal(0, 50.0f);
    args.getAxesSpecified().setVal(0, true);
    
    bool result = motionController->moveTo(args);
    TEST_ASSERT_TRUE(result);
    
    // Control mode should have defaulted to position
    TEST_ASSERT_EQUAL(MotorDriverBase::ControlMode::POSITION, args.getControlMode());
}

void test_special_commands(void) {
    String config = R"({
        "axes": [{"name": "X", "driver": {"driver": "TMC2209"}}]
    })";
    RaftJson jsonConfig(config);
    motionController->setup(jsonConfig);
    
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
}

void test_unsupported_control_modes(void) {
    String config = R"({
        "axes": [{"name": "X", "driver": {"driver": "TMC2209"}}]
    })";
    RaftJson jsonConfig(config);
    motionController->setup(jsonConfig);
    
    // Test velocity mode (not yet supported)
    MotionArgs velArgs;
    velArgs.setControlMode(MotorDriverBase::ControlMode::VELOCITY);
    velArgs.setVelocityTarget(0, 50.0f);
    
    bool result = motionController->moveTo(velArgs);
    TEST_ASSERT_FALSE(result); // Should fail - not yet implemented
    
    // Test torque mode (not yet supported)
    MotionArgs torqueArgs;
    torqueArgs.setControlMode(MotorDriverBase::ControlMode::TORQUE);
    torqueArgs.setTorqueTarget(0, 0.5f);
    
    result = motionController->moveTo(torqueArgs);
    TEST_ASSERT_FALSE(result); // Should fail - not yet implemented
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_motion_controller_setup);
    RUN_TEST(test_position_control_integration);
    RUN_TEST(test_backward_compatibility);
    RUN_TEST(test_special_commands);
    RUN_TEST(test_unsupported_control_modes);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

### **5.3 Validation Criteria**
- [ ] MotionController integrates with new motor driver architecture
- [ ] Existing position control functionality preserved
- [ ] All public interfaces unchanged (backward compatibility)
- [ ] Motor driver factory integration works
- [ ] Configuration parsing updated appropriately
- [ ] Performance maintained or improved

---

## Layer 6: Application Interface Layer

### **Objective**: Minimal changes to MotorControl class with full backward compatibility

### **6.1 Enhanced MotorControl Interface (minimal changes)**

**File**: `components/MotorControl/MotorControl.cpp` (minimal modifications)

#### **Step 6.1.1: Update JSON Command Handling**

```cpp
// In MotorControl::sendCmdJSON() - minimal addition for enhanced MotionArgs parsing
RaftRetCode MotorControl::sendCmdJSON(const char* cmdJSON) {
    // Extract command from JSON (unchanged)
    RaftJson jsonInfo(cmdJSON);
    String cmd = jsonInfo.getString("cmd", "");
    
    if (cmd.equalsIgnoreCase("motion")) {
        MotionArgs motionArgs;
        motionArgs.fromJSON(cmdJSON);  // Enhanced parsing now handles new modes automatically
        
        #ifdef DEBUG_MOTOR_CMD_JSON
        String cmdStr = motionArgs.toJSON();
        LOG_I(MODULE_PREFIX, "sendCmdJSON %s", cmdStr.c_str());
        #endif
        
        // Route to enhanced motion controller (interface unchanged)
        bool success = _motionController.moveTo(motionArgs);
        return success ? RAFT_OK : RAFT_INVALID_OPERATION;
    }
    
    // ALL existing commands remain completely unchanged
    else if (cmd.equalsIgnoreCase("maxCurrent")) {
        float maxCurrentA = jsonInfo.getDouble("maxCurrentA", 0);
        uint32_t axisIdx = jsonInfo.getInt("axisIdx", 0);
        _motionController.setMaxMotorCurrentAmps(axisIdx, maxCurrentA);
    }
    else if (cmd.equalsIgnoreCase("offAfter")) {
        float motorOnTimeAfterMoveSecs = jsonInfo.getDouble("offAfterS", 0);
        _motionController.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
    }
    // ... all other existing commands unchanged
    
    return RAFT_OK;
}
```

#### **Step 6.1.2: Create Application Layer Test**

**File**: `unit_tests/main/MotorControlApplication_test.cpp`

```cpp
#include "unity.h"
#include "MotorControl.h"

static MotorControl* motorControl = nullptr;

void setUp(void) {
    String config = R"({
        "name": "MotorControl",
        "type": "MotorControl", 
        "motorType": "BasicStepper",
        "axes": [
            {
                "name": "X",
                "params": {"maxSpeed": 1000, "stepsPerUnit": 80},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15}
            }
        ]
    })";
    
    motorControl = new MotorControl("MotorControl", config.c_str());
    motorControl->setup();
}

void tearDown(void) {
    if (motorControl) {
        delete motorControl;
        motorControl = nullptr;
    }
}

void test_backward_compatibility_position_commands(void) {
    // Existing JSON command format should work unchanged
    String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":100}]})";
    
    RaftRetCode result = motorControl->sendCmdJSON(cmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
}

void test_enhanced_motion_args_parsing(void) {
    // New explicit position mode format should work
    String cmd = R"({
        "cmd": "motion",
        "mode": "position",
        "pos": [{"a": 0, "p": 50}]
    })";
    
    RaftRetCode result = motorControl->sendCmdJSON(cmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
}

void test_existing_non_motion_commands(void) {
    // Test that existing non-motion commands work unchanged
    String maxCurrentCmd = R"({"cmd":"maxCurrent","axisIdx":0,"maxCurrentA":1.5})";
    RaftRetCode result1 = motorControl->sendCmdJSON(maxCurrentCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result1);
    
    String offAfterCmd = R"({"cmd":"offAfter","offAfterS":15.0})";
    RaftRetCode result2 = motorControl->sendCmdJSON(offAfterCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result2);
    
    String pauseCmd = R"({"cmd":"pause"})";
    RaftRetCode result3 = motorControl->sendCmdJSON(pauseCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result3);
}

void test_status_reporting_unchanged(void) {
    // Existing status reporting should work
    String statusJson = motorControl->getDataJSON(DEVICE_JSON_LEVEL_MED);
    TEST_ASSERT_TRUE(statusJson.length() > 0);
    
    // Test named value queries
    bool isFresh;
    double busyValue = motorControl->getNamedValue("b", isFresh);
    TEST_ASSERT_TRUE(isFresh);
    TEST_ASSERT_TRUE(busyValue == 0.0 || busyValue == 1.0); // Should be boolean-like
}

void test_velocity_mode_parsing_but_not_execution(void) {
    // Phase 1: New velocity command format should parse but not execute
    String velCmd = R"({
        "cmd": "motion",
        "mode": "velocity", 
        "velocityTargets": [{"a": 0, "v": 50.0}],
        "duration": 5000
    })";
    
    // Command should parse but return error because velocity mode not implemented yet
    RaftRetCode result = motorControl->sendCmdJSON(velCmd.c_str());
    TEST_ASSERT_NOT_EQUAL(RAFT_OK, result);
}

void test_configuration_compatibility(void) {
    // Test that existing configuration format still works
    // This is tested implicitly in setUp, but verify motor control is operational
    
    TEST_ASSERT_FALSE(motorControl->isBusy());
    
    // Send a simple position command to verify system is working
    String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":10}]})";
    RaftRetCode result = motorControl->sendCmdJSON(cmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result);
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_backward_compatibility_position_commands);
    RUN_TEST(test_enhanced_motion_args_parsing);
    RUN_TEST(test_existing_non_motion_commands);
    RUN_TEST(test_status_reporting_unchanged);
    RUN_TEST(test_velocity_mode_parsing_but_not_execution);
    RUN_TEST(test_configuration_compatibility);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

### **6.2 Validation Criteria**
- [ ] All existing JSON commands work unchanged
- [ ] Enhanced MotionArgs parsing works transparently
- [ ] No changes to public MotorControl API
- [ ] Configuration format backward compatible
- [ ] Status reporting unchanged
- [ ] Performance equivalent or better

---

## Full System Integration Testing

### **System-Level Integration Test**

**File**: `unit_tests/main/FullSystemIntegration_test.cpp`

```cpp
#include "unity.h"
#include "MotorControl.h"
#include "MotorDriverFactory.h"

static MotorControl* system = nullptr;

void setUp(void) {
    // Create complete system with realistic configuration
    String config = R"({
        "name": "MotorControl",
        "type": "MotorControl",
        "motorType": "BasicStepper",
        "axes": [
            {
                "name": "X",
                "params": {"maxSpeed": 1000, "stepsPerUnit": 80, "maxAcceleration": 500},
                "driver": {"driver": "TMC2209", "stepPin": 14, "dirPin": 15, "microsteps": 16}
            },
            {
                "name": "Y", 
                "params": {"maxSpeed": 800, "stepsPerUnit": 100, "maxAcceleration": 400},
                "driver": {"driver": "TMC2209", "stepPin": 16, "dirPin": 17, "microsteps": 16}
            }
        ],
        "ramp": {"timerIndex": 0, "queueLength": 50}
    })";
    
    system = new MotorControl("MotorControl", config.c_str());
    system->setup();
}

void tearDown(void) {
    if (system) {
        delete system;
        system = nullptr;
    }
}

void test_complete_system_operation(void) {
    // Test that the complete new architecture works end-to-end
    
    // 1. Test basic position command
    String posCmd = R"({"cmd":"motion","pos":[{"a":0,"p":100}]})";
    TEST_ASSERT_EQUAL(RAFT_OK, system->sendCmdJSON(posCmd.c_str()));
    
    // 2. Test multi-axis command
    String multiAxisCmd = R"({"cmd":"motion","pos":[{"a":0,"p":50},{"a":1,"p":25}]})";
    TEST_ASSERT_EQUAL(RAFT_OK, system->sendCmdJSON(multiAxisCmd.c_str()));
    
    // 3. Test relative motion
    String relativeCmd = R"({"cmd":"motion","relative":true,"pos":[{"a":0,"p":10}]})";
    TEST_ASSERT_EQUAL(RAFT_OK, system->sendCmdJSON(relativeCmd.c_str()));
    
    // 4. Test motor control commands
    String currentCmd = R"({"cmd":"maxCurrent","axisIdx":0,"maxCurrentA":1.2})";
    TEST_ASSERT_EQUAL(RAFT_OK, system->sendCmdJSON(currentCmd.c_str()));
}

void test_architecture_component_integration(void) {
    // Verify all layers of the architecture are working together
    
    // Test factory is working
    auto supportedTypes = MotorDriverFactory::getSupportedMotorTypes();
    TEST_ASSERT_TRUE(supportedTypes.size() > 0);
    
    // Test system status
    TEST_ASSERT_FALSE(system->isBusy()); // Should start not busy
    
    // Send command and verify system becomes busy
    String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":100}]})";
    system->sendCmdJSON(cmd.c_str());
    
    // System might be busy immediately after command
    // (This depends on timing and implementation details)
}

void test_error_handling_and_recovery(void) {
    // Test various error conditions
    
    // Invalid axis index
    String invalidAxis = R"({"cmd":"motion","pos":[{"a":5,"p":100}]})";
    RaftRetCode result = system->sendCmdJSON(invalidAxis.c_str());
    // Should either succeed (ignoring invalid axis) or return error
    
    // Invalid JSON
    String invalidJson = R"({"cmd":"motion","pos":[{invalid}]})";
    result = system->sendCmdJSON(invalidJson.c_str());
    // Should handle gracefully
    
    // Unknown command
    String unknownCmd = R"({"cmd":"unknownCommand"})";
    result = system->sendCmdJSON(unknownCmd.c_str());
    TEST_ASSERT_EQUAL(RAFT_OK, result); // Should ignore unknown commands gracefully
}

void test_performance_and_timing(void) {
    // Basic performance test
    uint32_t startTime = millis();
    
    // Send multiple commands
    for (int i = 0; i < 10; i++) {
        String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":)" + String(i * 10) + R"(}]})";
        system->sendCmdJSON(cmd.c_str());
    }
    
    uint32_t endTime = millis();
    uint32_t duration = endTime - startTime;
    
    // Should be able to process 10 commands quickly
    TEST_ASSERT_TRUE(duration < 100); // Less than 100ms for 10 commands
}

void test_memory_and_resource_usage(void) {
    // Basic resource usage test
    
    // System should not leak memory during normal operation
    // (This would require more sophisticated testing in a real environment)
    
    // Send commands and verify system remains stable
    for (int i = 0; i < 50; i++) {
        String cmd = R"({"cmd":"motion","pos":[{"a":0,"p":)" + String(i) + R"(}]})";
        system->sendCmdJSON(cmd.c_str());
        
        // System should remain responsive
        TEST_ASSERT_FALSE(system->hasError()); // Assuming hasError method exists
    }
}

// Main test runner
void setup() {
    UNITY_BEGIN();
    
    RUN_TEST(test_complete_system_operation);
    RUN_TEST(test_architecture_component_integration);
    RUN_TEST(test_error_handling_and_recovery);
    RUN_TEST(test_performance_and_timing);
    RUN_TEST(test_memory_and_resource_usage);
    
    UNITY_END();
}

void loop() {
    // Empty - all tests run in setup()
}
```

---

## Overall Implementation Validation

### **Complete Validation Checklist**

#### **Layer 1: Step/Direction Hardware Interface**
- [ ] StepDriverStepDirectionBase interface clean and focused
- [ ] StepDriverTMC2209Enhanced works with new interface
- [ ] All existing step/direction functionality preserved
- [ ] No high-level motor control concepts leaked into this layer

#### **Layer 2: Abstract Motor Interface**  
- [ ] MotorDriverBase provides clean high-level abstraction
- [ ] Capability system works correctly for different motor types
- [ ] Multi-axis support properly abstracted
- [ ] Interface is extensible for future motor types

#### **Layer 3: Motor Driver Implementation**
- [ ] MotorDriverBasicStepper correctly implements MotorDriverBase
- [ ] BasicStepperRampGenerator functionality preserved
- [ ] Step drivers properly integrated
- [ ] Position control works as expected

#### **Layer 4: Factory & Integration**
- [ ] MotorDriverFactory creates correct driver types
- [ ] Configuration-driven motor type selection working
- [ ] Auto-detection logic functional
- [ ] Extensible for future motor types

#### **Layer 5: Control Coordination** 
- [ ] MotionController integrates with new architecture
- [ ] All public interfaces unchanged (backward compatibility)
- [ ] Existing functionality preserved
- [ ] Ready for future control mode extensions

#### **Layer 6: Application Interface**
- [ ] All existing JSON commands work unchanged
- [ ] Enhanced MotionArgs parsing transparent
- [ ] No changes to public MotorControl API
- [ ] Configuration format backward compatible

#### **Full System Integration**
- [ ] Complete end-to-end functionality operational
- [ ] Performance maintained or improved
- [ ] Memory usage within acceptable limits
- [ ] Error handling robust
- [ ] Backward compatibility verified

This layer-by-layer approach ensures robust implementation with comprehensive testing at each level, building confidence as you progress up the architecture stack while maintaining complete backward compatibility.