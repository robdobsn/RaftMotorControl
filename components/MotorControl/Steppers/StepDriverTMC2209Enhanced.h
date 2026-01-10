/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverTMC2209Enhanced - Enhanced TMC2209 Driver with MotorDriverBase Interface
//
// Rob Dobson 2025 - Motor Control Enhancement
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "StepDriverTMC2209.h"
#include "MotorDriverBase.h"

class StepDriverTMC2209Enhanced : public StepDriverTMC2209, public MotorDriverBase
{
public:
    StepDriverTMC2209Enhanced();
    virtual ~StepDriverTMC2209Enhanced() = default;

    // MotorDriverBase interface implementation
    MotorCapabilities getCapabilities() const override;
    bool supportsControlMode(ControlMode mode) const override;
    
    // Control interface
    bool setPositionTarget(float position) override;
    bool setVelocityTarget(float velocity) override;
    bool setTorqueTarget(float torque) override;
    
    // Feedback interface
    float getCurrentPosition() const override;
    float getCurrentVelocity() const override;
    float getCurrentTorque() const override;
    
    // Status interface (delegates to StepDriverTMC2209)
    bool isEnabled() const override;
    void setEnabled(bool enabled) override;
    bool isMoving() const override;
    bool hasError() const override;
    String getStatusString() const override;
    
    // Configuration interface
    bool setup(const RaftJsonIF& config) override;
    void calibrate() override;
    
    // Legacy stepper interface (delegates to StepDriverTMC2209)
    void step(bool direction) override;
    void setDirection(bool direction) override;
    
    // Enhanced diagnostic interface (non-virtual since base is final)
    String getEnhancedDebugJSON(bool includeBraces = true, bool detailed = false) const;

private:
    // Position tracking (step counting for open-loop stepper)
    mutable float _currentPosition = 0.0f;
    mutable int32_t _totalSteps = 0;
    float _stepsPerUnit = 80.0f; // Will be configured from setup
    bool _lastDirection = true;   // True = positive direction
    
    // Velocity tracking (basic feedrate-based estimation)
    mutable float _currentVelocity = 0.0f;
    mutable uint32_t _lastStepTime = 0;
    mutable uint32_t _stepInterval = 0;
    
    // Torque estimation (basic current sensing if available)
    mutable float _currentTorque = 0.0f;
    
    // Target tracking for status
    float _positionTarget = 0.0f;
    float _velocityTarget = 0.0f;
    
    // Enable state tracking
    bool _isEnabledState = false;
    
    // Error state tracking  
    bool _hasErrorState = false;
    String _lastErrorMsg;
    
    // Configuration
    bool _configComplete = false;
    
    // Helper methods
    void updatePositionFromSteps() const;
    void updateVelocityEstimate() const;
    float estimateTorqueFromCurrent() const;
    bool validatePositionTarget(float position) const;
    bool validateVelocityTarget(float velocity) const;
    String getCapabilitiesJSON() const;
    
    // Constants
    static constexpr float DEFAULT_MAX_VELOCITY = 1000.0f; // units/sec
    static constexpr float DEFAULT_MAX_TORQUE = 1.0f;      // Nm estimate
    static constexpr float DEFAULT_POSITION_RESOLUTION = 1.0f/80.0f; // 1/stepsPerUnit
    static constexpr float VELOCITY_SMOOTHING_FACTOR = 0.1f; // For velocity estimation
    static constexpr uint32_t MIN_STEP_INTERVAL_US = 100;    // 10kHz max step rate
    
    // Debug
    static constexpr const char* MODULE_PREFIX = "StepDriverTMC2209Enhanced";
};