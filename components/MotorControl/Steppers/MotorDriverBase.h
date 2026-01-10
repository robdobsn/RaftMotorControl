/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotorDriverBase - Abstract Motor Driver Interface
//
// Rob Dobson 2025 - Motor Control Enhancement
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <vector>
#include "RaftJsonIF.h"
#include "RaftUtils.h"

class MotorDriverBase {
public:
    enum class MotorType { 
        STEPPER_OPEN_LOOP,      // Traditional steppers (current TMC2209 implementation)
        STEPPER_CLOSED_LOOP,    // Steppers with encoder feedback
        SERVO,                  // RC/Industrial servos with position feedback
        BLDC                    // Brushless DC motors with FOC
    };
    
    enum class ControlMode { 
        POSITION,               // Position control (existing trajectory path)
        VELOCITY,               // Velocity control (new real-time path)
        TORQUE                  // Torque/current control (new real-time path)
    };
    
    enum class FeedbackType {
        NONE,                   // Open loop (step counting only)
        ENCODER,                // Rotary encoder feedback
        POTENTIOMETER,          // Analog position feedback
        CURRENT,                // Current sensing for torque
        HALL                    // Hall effect sensors (BLDC)
    };
    
    struct MotorCapabilities {
        MotorType motorType = MotorType::STEPPER_OPEN_LOOP;
        std::vector<ControlMode> supportedModes;
        std::vector<FeedbackType> feedbackTypes;
        bool hasPositionFeedback = false;
        bool hasVelocityFeedback = false;
        bool hasTorqueFeedback = false;
        float maxVelocity = 1000.0f;          // units/sec
        float maxTorque = 1.0f;               // Nm
        float positionResolution = 0.0125f;   // units per step/count
        float velocityResolution = 0.1f;      // units/sec resolution
        float torqueResolution = 0.01f;       // Nm resolution
        
        // Constructor
        MotorCapabilities() {
            supportedModes.clear();
            feedbackTypes.clear();
        }
    };
    
    // Virtual destructor
    virtual ~MotorDriverBase() = default;
    
    // Capability queries
    virtual MotorCapabilities getCapabilities() const = 0;
    virtual bool supportsControlMode(ControlMode mode) const = 0;
    
    // Control interface - all drivers must implement these
    virtual bool setPositionTarget(float position) = 0;
    virtual bool setVelocityTarget(float velocity) = 0;
    virtual bool setTorqueTarget(float torque) = 0;
    
    // Feedback interface
    virtual float getCurrentPosition() const = 0;
    virtual float getCurrentVelocity() const = 0;
    virtual float getCurrentTorque() const = 0;
    
    // Status and control
    virtual bool isEnabled() const = 0;
    virtual void setEnabled(bool enabled) = 0;
    virtual bool isMoving() const = 0;
    virtual bool hasError() const = 0;
    virtual String getStatusString() const = 0;
    
    // Configuration and calibration
    virtual bool setup(const RaftJsonIF& config) = 0;
    virtual void calibrate() = 0;
    
    // Legacy stepper interface (for backward compatibility with existing code)
    // These will be implemented by stepper drivers but may be no-ops for other motor types
    virtual void step(bool direction) = 0;
    virtual void setDirection(bool direction) = 0;
    
    // Note: getDebugJSON is not required here since stepper base classes already provide it
    // Enhanced drivers can provide their own enhanced debug methods
    
    // Helper methods for common functionality (public for testing)
    static String controlModeToString(ControlMode mode) {
        switch (mode) {
            case ControlMode::POSITION: return "position";
            case ControlMode::VELOCITY: return "velocity";
            case ControlMode::TORQUE: return "torque";
            default: return "unknown";
        }
    }
    
    static String motorTypeToString(MotorType type) {
        switch (type) {
            case MotorType::STEPPER_OPEN_LOOP: return "stepperOpenLoop";
            case MotorType::STEPPER_CLOSED_LOOP: return "stepperClosedLoop";
            case MotorType::SERVO: return "servo";
            case MotorType::BLDC: return "bldc";
            default: return "unknown";
        }
    }
    
protected:
    
    // Common state tracking
    bool _isEnabled = false;
    bool _hasError = false;
    String _lastErrorMessage;
    
    // Target tracking for status reporting
    float _positionTarget = 0.0f;
    float _velocityTarget = 0.0f;
    float _torqueTarget = 0.0f;
    
    // Debug
    static constexpr const char* MODULE_PREFIX = "MotorDriverBase";
};