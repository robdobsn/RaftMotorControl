# RaftMotorControl Enhancement Implementation Plan

## Overview

This plan implements multi-modal motor control (position, velocity, torque) and multi-motor type support (stepper, servo, BLDC) using a bottom-up, test-driven approach that preserves existing functionality while creating proper abstraction layers.

## Architecture Evolution Strategy

### Current Architecture
```
MotorControl → MotionController → MotionBlockManager → RampGenerator → StepDriverBase
                                       ↓                    ↓              ↓
                                 MotionPipeline        Hardware Timer   TMC2209
```

### Target Architecture
```
MotorControl → MotionController → MotionBlockManager → MotorDriverFactory
                    ↓                    ↓                      ↓
                PatternManager      MotorEnabler        MotorDriverBase (abstract)
                                                              ↓
                                                    ┌─────────┼─────────────┐
                                                    ↓         ↓             ↓
                                         MotorDriverBasicStepper  MotorDriverBLDC  MotorDriverServo
                                                    ↓
                                       BasicStepperRampGenerator
                                                    ↓
                                         StepDriverStepDirectionBase[]
                                                    ↓
                                              StepDriverTMC2209Enhanced
```

## Key Architectural Changes

### **Driver Hierarchy Separation**
- **MotorDriverBase**: High-level interface (`MotorDriver/` folder) with `setPositionTarget()`, `setVelocityTarget()`, `setTorqueTarget()`
- **MotorDriverBasicStepper**: Encapsulates RampGenerator logic for step/direction motors
- **StepDriverStepDirectionBase**: Low-level step/direction hardware interface (`Steppers/` folder)
- **StepDriverTMC2209Enhanced**: Concrete hardware implementation

### **Factory Pattern**
- **MotorDriverFactory**: Creates `MotorDriverBase*` instances based on configuration
- **Configuration-driven**: Motor type selection via JSON config
- **Encapsulated complexity**: Factory handles `StepDriverStepDirectionBase` instantiation internally

### **Multi-Axis Coordination**
- **MotorDriverBase**: Handles multiple axes of same motor type
- **MotionController**: Retains block splitting, queueing, kinematics responsibilities
- **BasicStepperRampGenerator**: Coordinates timing across multiple stepper axes

## Implementation Phases

### **Phase 1: Foundation Layer (4-6 weeks)**
**Risk Level**: Low  
**Dependencies**: None  
**Objective**: Create new abstraction hierarchy and factory pattern

#### **Step 1.1: Motor Driver Abstraction** (Week 1-2)
**Location**: `components/MotorControl/MotorDriver/` (new folder)

**Tasks**:
1. **Create MotorDriverBase.h**
   ```cpp
   class MotorDriverBase {
   public:
       enum class MotorType { STEPPER, SERVO_STEPPER, SERVO, BLDC };
       enum class ControlMode { POSITION, VELOCITY, TORQUE };
       
       // Capability queries
       virtual MotorType getMotorType() const = 0;
       virtual bool supportsControlMode(ControlMode mode) const = 0;
       virtual bool hasPositionFeedback() const = 0;
       virtual bool hasVelocityFeedback() const = 0;
       virtual bool hasTorqueFeedback() const = 0;
       
       // High-level control interface (multi-axis)
       virtual void setPositionTarget(uint32_t axisIdx, float position) = 0;
       virtual void setVelocityTarget(uint32_t axisIdx, float velocity) = 0;
       virtual void setTorqueTarget(uint32_t axisIdx, float torque) = 0;
       
       // Feedback interface  
       virtual float getCurrentPosition(uint32_t axisIdx) = 0;
       virtual float getCurrentVelocity(uint32_t axisIdx) = 0;
       virtual float getCurrentTorque(uint32_t axisIdx) = 0;
       
       // Multi-axis coordination
       virtual void setup(const RaftJsonIF& config, uint32_t numAxes) = 0;
       virtual void loop() = 0;
       virtual bool isBusy() const = 0;
   };
   ```

2. **Create StepDriverStepDirectionBase.h** (rename existing StepDriverBase)
   - Move to `components/MotorControl/Steppers/`
   - Keep existing `stepStart()`, `stepEnd()`, `setDirection()` interface
   - Remove high-level methods (`setPositionTarget`, etc.)

3. **Create test harness**: `tests/MotorDriverBaseTest`

**Test Harness Requirements**:
- Mock motor driver implementations
- Capability query validation
- Interface contract verification
- Multi-axis coordination testing

**Success Criteria**:
- Clean abstraction hierarchy established
- Interface contracts defined
- Test framework operational

#### **Step 1.2: Basic Stepper Motor Driver** (Week 2-3)
**Location**: `components/MotorControl/MotorDriver/MotorDriverBasicStepper.h` (new)

**Tasks**:
1. **Implement MotorDriverBasicStepper**
   ```cpp
   class MotorDriverBasicStepper : public MotorDriverBase {
   private:
       BasicStepperRampGenerator _rampGenerator;
       std::vector<StepDriverStepDirectionBase*> _stepDrivers;
       AxesParams _axesParams;
       
   public:
       // MotorDriverBase implementation
       MotorType getMotorType() const override { return MotorType::STEPPER; }
       bool supportsControlMode(ControlMode mode) const override;
       
       void setPositionTarget(uint32_t axisIdx, float position) override;
       void setVelocityTarget(uint32_t axisIdx, float velocity) override; // Limited support
       void setTorqueTarget(uint32_t axisIdx, float torque) override;     // Not supported
       
       // Encapsulate existing RampGenerator logic
       void setup(const RaftJsonIF& config, uint32_t numAxes) override;
       void loop() override;
       bool isBusy() const override;
   };
   ```

2. **Rename RampGenerator → BasicStepperRampGenerator**
   - Keep existing multi-axis step coordination logic
   - Maintain existing MotionPipeline integration
   - Preserve timer ISR functionality

3. **Update StepDriverTMC2209Enhanced**
   - Inherit from `StepDriverStepDirectionBase`
   - Remove high-level interface methods
   - Focus on step/direction hardware control

4. **Create test harness**: `tests/MotorDriverBasicStepperTest`

**Test Harness Requirements**:
- Position target accuracy testing
- Multi-axis coordination validation
- RampGenerator integration testing
- Backward compatibility with existing stepper logic

#### **Step 1.3: Motor Driver Factory** (Week 3-4)
**Location**: `components/MotorControl/MotorDriver/MotorDriverFactory.h` (new)

**Tasks**:
1. **Implement MotorDriverFactory**
   ```cpp
   class MotorDriverFactory {
   private:
       static std::map<String, CreateDriverFn> _registeredTypes;
       
   public:
       // Factory interface
       static MotorDriverBase* createDriver(
           const String& motorType,
           const RaftJsonIF& config,
           uint32_t numAxes
       );
       
       // Registration interface
       static void registerDriverType(
           const String& typeName,
           CreateDriverFn createFn
       );
       
       // Auto-detection
       static String detectMotorType(const RaftJsonIF& config);
   };
   ```

2. **Register basic stepper type**
   ```cpp
   MotorDriverFactory::registerDriverType("BasicStepper", 
       [](const RaftJsonIF& config, uint32_t numAxes) -> MotorDriverBase* {
           auto* driver = new MotorDriverBasicStepper();
           driver->setup(config, numAxes);
           return driver;
       });
   ```

3. **Create test harness**: `tests/MotorDriverFactoryTest`

**Test Harness Requirements**:
- Driver creation validation
- Configuration parsing testing
- Registration system verification
- Error handling validation

**Success Criteria**:
- Factory pattern operational
- Configuration-driven driver creation
- Extensible for future motor types

### **Phase 2: Integration Layer (3-4 weeks)**
**Risk Level**: Medium  
**Dependencies**: Phase 1 complete  
**Objective**: Integrate new architecture with MotionController

#### **Step 2.1: MotionController Refactoring** (Week 5-6)
**Location**: `components/MotorControl/Controller/MotionController.cpp`

**Tasks**:
1. **Replace StepDriverBase with MotorDriverBase**
   ```cpp
   class MotionController : public MotionControlIF {
   private:
       MotorDriverBase* _motorDriver = nullptr;  // Single multi-axis driver
       // Remove: std::vector<StepDriverBase*> _stepperDrivers;
       // Remove: RampGenerator _rampGenerator;
       
       MotionBlockManager _blockManager;
       MotorEnabler _motorEnabler;
       MotionPatternManager _patternManager;
   };
   ```

2. **Update setup method**
   ```cpp
   void MotionController::setup(const RaftJsonIF& config) {
       // Extract motor configuration
       String motorType = config.getString("motorType", "BasicStepper");
       uint32_t numAxes = config.getInt("numAxes", 3);
       
       // Create motor driver via factory
       _motorDriver = MotorDriverFactory::createDriver(motorType, config, numAxes);
       
       // Setup other components
       _blockManager.setup(config);
       _motorEnabler.setup(config);
       _patternManager.setup(config);
   }
   ```

3. **Update moveTo implementation**
   ```cpp
   bool MotionController::moveTo(MotionArgs& args) {
       // Handle special commands (stop, clear, disable)
       if (args.isStopMotion()) {
           _motorDriver->stop();
           return true;
       }
       
       // Route to appropriate control mode
       switch (args.getControlMode()) {
           case ControlMode::POSITION:
               return moveToPosition(args);     // Block-based planning
           case ControlMode::VELOCITY:
               return setVelocityTarget(args);  // Direct driver command
           case ControlMode::TORQUE:
               return setTorqueTarget(args);    // Direct driver command
       }
   }
   ```

4. **Create test harness**: `tests/MotionControllerIntegrationTest`

**Test Harness Requirements**:
- Factory integration testing
- Multi-mode command routing
- Backward compatibility validation
- Performance regression testing

#### **Step 2.2: Enhanced Motion Arguments** (Week 6-7)
**Location**: `components/MotorControl/Controller/MotionArgs.h`

**Tasks**:
1. **Extend MotionArgs class**
   ```cpp
   class MotionArgs {
   private:
       ControlMode _controlMode = ControlMode::POSITION; // Default backward compatibility
       AxesValues<float> _velocityTargets;
       AxesValues<float> _torqueTargets;
       uint32_t _commandDuration = 0; // For velocity/torque commands
       
   public:
       // Control mode management
       void setControlMode(ControlMode mode);
       ControlMode getControlMode() const;
       
       // Velocity control
       void setVelocityTarget(uint32_t axisIdx, float velocity);
       float getVelocityTarget(uint32_t axisIdx) const;
       
       // Torque control
       void setTorqueTarget(uint32_t axisIdx, float torque);
       float getTorqueTarget(uint32_t axisIdx) const;
       
       // Command timing
       void setCommandDuration(uint32_t durationMs);
       uint32_t getCommandDuration() const;
   };
   ```

2. **Enhanced JSON parsing**
   - Parse control mode from JSON
   - Parse velocity/torque targets
   - Maintain backward compatibility for existing commands

3. **Create test harness**: `tests/MotionArgsTest`

#### **Step 2.3: JSON Command Interface** (Week 7-8)
**Location**: `components/MotorControl/MotorControl.cpp`

**Tasks**:
1. **Enhanced sendCmdJSON()**
   ```cpp
   RaftRetCode MotorControl::sendCmdJSON(const char* cmdJSON) {
       RaftJson jsonInfo(cmdJSON);
       String cmd = jsonInfo.getString("cmd", "");
       
       if (cmd.equalsIgnoreCase("motion")) {
           MotionArgs motionArgs;
           motionArgs.fromJSON(cmdJSON);
           
           // Command routing based on control mode
           return _motionController.moveTo(motionArgs);
       }
       // ... existing commands unchanged
   }
   ```

2. **JSON command examples**
   ```json
   // Position mode (existing, unchanged)
   {"cmd":"motion","pos":[{"a":0,"p":100}]}
   
   // Velocity mode (new)
   {"cmd":"motion","mode":"velocity","targets":[{"a":0,"vel":50}],"duration":5000}
   
   // Torque mode (new)  
   {"cmd":"motion","mode":"torque","targets":[{"a":0,"torque":0.5}],"duration":1000}
   ```

3. **Create test harness**: `tests/JSONCommandTest`

**Success Criteria**:
- All existing JSON commands work unchanged
- New command formats parsed correctly
- Integration with new architecture complete
- Performance maintained

### **Phase 3: Motor Type Extension (4-5 weeks)**
**Risk Level**: Medium-High  
**Dependencies**: Phase 2 complete  
**Objective**: Add servo and smart stepper support

#### **Step 3.1: Servo Stepper Driver** (Week 9-10)
**Location**: `components/MotorControl/MotorDriver/MotorDriverServoStepper.h` (new)

**Tasks**:
1. **Create MotorDriverServoStepper class**
   ```cpp
   class MotorDriverServoStepper : public MotorDriverBase {
   private:
       std::vector<StepDriverStepDirectionBase*> _stepDrivers;
       std::vector<EncoderInterface*> _encoders;
       std::vector<PIDController> _positionPIDs;
       std::vector<PIDController> _velocityPIDs;
       
   public:
       // MotorDriverBase implementation with closed-loop control
       MotorType getMotorType() const override { return MotorType::SERVO_STEPPER; }
       bool supportsControlMode(ControlMode mode) const override;
       
       void setPositionTarget(uint32_t axisIdx, float position) override;
       void setVelocityTarget(uint32_t axisIdx, float velocity) override;
       void setTorqueTarget(uint32_t axisIdx, float torque) override;
       
       float getCurrentPosition(uint32_t axisIdx) override;
       float getCurrentVelocity(uint32_t axisIdx) override;
       float getCurrentTorque(uint32_t axisIdx) override;
   };
   ```

2. **Encoder interface abstraction**
3. **Basic PID controller implementation**
4. **Factory registration**
   ```cpp
   MotorDriverFactory::registerDriverType("ServoStepper", createServoStepperDriver);
   ```

5. **Create test harness**: `tests/MotorDriverServoStepperTest`

#### **Step 3.2: Basic Servo Motor Support** (Week 10-11)
**Location**: `components/MotorControl/MotorDriver/MotorDriverServo.h` (new)

**Tasks**:
1. **Create MotorDriverServo class**
   - PWM signal generation for position control
   - Analog feedback for position sensing
   - Basic velocity estimation

2. **Hardware abstraction**
   - PWM output interface
   - Analog input interface
   - Timing coordination

3. **Factory registration and configuration**
4. **Create test harness**: `tests/MotorDriverServoTest`

#### **Step 3.3: Smart Stepper Support** (Week 11-12)
**Location**: `components/MotorControl/MotorDriver/MotorDriverSmartStepper.h` (new)

**Tasks**:
1. **Create MotorDriverSmartStepper**
   ```cpp
   class MotorDriverSmartStepper : public MotorDriverBase {
   private:
       std::vector<SmartStepperInterface*> _smartSteppers; // Direct position/velocity control
       
   public:
       // Direct command pass-through to smart steppers
       void setPositionTarget(uint32_t axisIdx, float position) override;
       void setVelocityTarget(uint32_t axisIdx, float velocity) override;
       
       // No RampGenerator needed - stepper handles ramping internally
   };
   ```

2. **Smart stepper interface abstraction**
3. **Configuration and factory registration**
4. **Create test harness**: `tests/MotorDriverSmartStepperTest`

**Success Criteria**:
- Multiple motor types supported simultaneously
- Configuration-driven motor selection working
- All motor types integrate with MotionController

### **Phase 4: Advanced Features (3-4 weeks)**
**Risk Level**: Medium  
**Dependencies**: Phase 3 complete  
**Objective**: Add BLDC support and advanced control features

#### **Step 4.1: BLDC Motor Support** (Week 13-14)
**Location**: `components/MotorControl/MotorDriver/MotorDriverBLDC.h` (new)

**Tasks**:
1. **Create MotorDriverBLDC class**
   - Direct velocity control
   - Hall sensor/encoder feedback
   - Current/torque control

2. **Basic commutation control**
3. **Factory registration**
4. **Create test harness**: `tests/MotorDriverBLDCTest`

#### **Step 4.2: Multi-Mode Coordination** (Week 15-16)
**Location**: `components/MotorControl/Controller/` (updates)

**Tasks**:
1. **Enhanced MotionController**
   - Support different motor types per system
   - Mode coordination and safety
   - Performance optimization

2. **Advanced testing and validation**
3. **Documentation and examples**

**Success Criteria**:
- All motor types operational
- Advanced control modes working
- System performance maintained
- Comprehensive testing complete

## Risk Mitigation Strategies

### **Technical Risks**
1. **Performance Degradation**
   - Mitigation: Comprehensive benchmarking at each phase
   - Monitoring: Real-time performance metrics

2. **Real-Time Constraints**
   - Mitigation: Dedicated timing analysis and ISR preservation
   - Validation: Timer accuracy testing

3. **Memory Usage**
   - Mitigation: Memory usage monitoring and optimization
   - Validation: Embedded system resource constraints

### **Integration Risks**
1. **Backward Compatibility**
   - Mitigation: Comprehensive regression testing
   - Validation: Existing application compatibility

2. **Configuration Complexity**
   - Mitigation: Clear configuration examples and validation
   - Documentation: Migration guides and tutorials

## Success Metrics

### **Phase 1 Success**
- [ ] New abstraction hierarchy established
- [ ] Factory pattern operational
- [ ] Basic stepper driver working through new architecture
- [ ] No performance regression

### **Phase 2 Success**  
- [ ] MotionController integration complete
- [ ] Multi-mode command routing working
- [ ] JSON API enhanced and backward compatible
- [ ] Existing functionality preserved

### **Phase 3 Success**
- [ ] Multiple motor types supported
- [ ] Servo stepper control functional
- [ ] Configuration-driven motor selection working
- [ ] Smart stepper integration complete

### **Phase 4 Success**
- [ ] BLDC motor support operational
- [ ] Advanced control features working
- [ ] Multi-mode coordination successful
- [ ] System fully validated

## Implementation Guidelines

### **Code Quality Standards**
- Comprehensive unit tests for all new components
- Integration tests at each interface boundary
- Performance benchmarks maintained
- Code coverage > 85% for new modules

### **Documentation Requirements**
- API documentation for all public interfaces
- Configuration examples and tutorials
- Migration guides for API changes
- Architecture decision records

### **Testing Strategy**
- Unit tests: Individual component validation
- Integration tests: Cross-layer functionality
- System tests: End-to-end scenarios
- Hardware-in-the-loop: Real hardware validation

This implementation plan provides a structured, risk-managed approach to creating proper abstraction layers while preserving existing RaftMotorControl functionality and performance.