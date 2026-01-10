/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverTMC2209Enhanced - Enhanced TMC2209 Driver Implementation
//
// Rob Dobson 2025 - Motor Control Enhancement
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "StepDriverTMC2209Enhanced.h"
#include <cmath>
#include "Logger.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
StepDriverTMC2209Enhanced::StepDriverTMC2209Enhanced()
{
    // Initialize state
    _currentPosition = 0.0f;
    _totalSteps = 0;
    _currentVelocity = 0.0f;
    _currentTorque = 0.0f;
    _isEnabledState = false;
    _hasErrorState = false;
    _configComplete = false;
    _lastStepTime = micros();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get motor capabilities
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MotorDriverBase::MotorCapabilities StepDriverTMC2209Enhanced::getCapabilities() const
{
    MotorCapabilities caps; // Constructor initializes defaults
    
    // Configure stepper-specific capabilities
    caps.motorType = MotorType::STEPPER_OPEN_LOOP;
    
    // Open-loop stepper supports position control primarily
    caps.supportedModes.push_back(ControlMode::POSITION);
    // Note: velocity control could be added with feedrate-based control
    // caps.supportedModes.push_back(ControlMode::VELOCITY); // Future enhancement
    
    caps.feedbackTypes.push_back(FeedbackType::NONE); // Step counting only
    
    // Override defaults with stepper-specific values
    caps.positionResolution = _configComplete ? (1.0f / _stepsPerUnit) : DEFAULT_POSITION_RESOLUTION;
    caps.maxVelocity = DEFAULT_MAX_VELOCITY;
    caps.maxTorque = DEFAULT_MAX_TORQUE;
    
    
    return caps;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check if control mode is supported
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::supportsControlMode(ControlMode mode) const
{
    switch (mode) {
        case ControlMode::POSITION:
            return true;
        case ControlMode::VELOCITY:
            // Could support basic velocity control via feedrate
            return false; // Not implemented in Phase 1
        case ControlMode::TORQUE:
            // Could support basic current control via TMC2209 current setting
            return false; // Not implemented in Phase 1
        default:
            return false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set position target
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::setPositionTarget(float position)
{
    if (!validatePositionTarget(position)) {
        return false;
    }
    
    _positionTarget = position;
    
    // For open-loop stepper, position control is handled by the motion planning system
    // This method primarily updates our target tracking for status reporting
    
    LOG_I(MODULE_PREFIX, "Position target set to %.2f", position);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set velocity target  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::setVelocityTarget(float velocity)
{
    if (!validateVelocityTarget(velocity)) {
        return false;
    }
    
    _velocityTarget = velocity;
    
    // Phase 1: Not implemented for open-loop steppers
    // Future: Could implement via step rate control
    
    LOG_W(MODULE_PREFIX, "Velocity control not supported in Phase 1");
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set torque target
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::setTorqueTarget(float torque)
{
    // Phase 1: Not implemented for open-loop steppers  
    // Future: Could implement via TMC2209 current control
    
    LOG_W(MODULE_PREFIX, "Torque control not supported in Phase 1");
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get current position
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float StepDriverTMC2209Enhanced::getCurrentPosition() const
{
    updatePositionFromSteps();
    return _currentPosition;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get current velocity
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float StepDriverTMC2209Enhanced::getCurrentVelocity() const
{
    // Phase 1: No velocity feedback for open-loop stepper
    // In future phases, this would use: updateVelocityEstimate(); return _currentVelocity;
    return 0.0f;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get current torque
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float StepDriverTMC2209Enhanced::getCurrentTorque() const
{
    _currentTorque = estimateTorqueFromCurrent();
    return _currentTorque;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check if enabled
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::isEnabled() const
{
    return _isEnabledState;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set enabled state
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepDriverTMC2209Enhanced::setEnabled(bool enabled)
{
    _isEnabledState = enabled;
    
    // Note: Actual motor enabling is handled by the RampGenerator and MotorEnabler
    // This tracks our logical enabled state for the MotorDriverBase interface
    
    LOG_I(MODULE_PREFIX, "Motor enabled state: %s", enabled ? "true" : "false");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check if moving
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::isMoving() const
{
    // Check if velocity estimate indicates movement
    return fabs(_currentVelocity) > 0.1f; // Arbitrary threshold
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check for errors
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::hasError() const
{
    // Phase 1: For interface testing without hardware, only check software error state
    // In future phases with hardware, would also check: if (!isOperatingOk()) return true;
    return _hasErrorState;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get status string
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String StepDriverTMC2209Enhanced::getStatusString() const
{
    String status = "TMC2209Enhanced: ";
    
    if (!_configComplete) {
        status += "Not Configured";
    } else if (hasError()) {
        status += "Error - " + _lastErrorMsg;
    } else if (isMoving()) {
        status += "Moving";
    } else if (isEnabled()) {
        status += "Enabled/Idle";
    } else {
        status += "Disabled";
    }
    
    return status;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup with JSON configuration
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool StepDriverTMC2209Enhanced::setup(const RaftJsonIF& config)
{
    // Extract stepper-specific parameters for legacy setup
    StepDriverParams stepperParams(const_cast<RaftJsonIF&>(config));
    String stepperName = config.getString("name", "TMC2209");
    bool usingISR = config.getBool("usingISR", false);
    
    // Get steps per unit for position calculations
    _stepsPerUnit = config.getDouble("stepsPerUnit", 80.0f);
    if (_stepsPerUnit <= 0.0f) {
        LOG_E(MODULE_PREFIX, "Invalid stepsPerUnit: %.2f", _stepsPerUnit);
        return false;
    }
    
    // Call parent setup
    bool result = StepDriverTMC2209::setup(stepperName, stepperParams, usingISR);
    
    if (result) {
        _configComplete = true;
        _hasErrorState = false;
        _lastErrorMsg = "";
        
        LOG_I(MODULE_PREFIX, "Setup complete - stepsPerUnit: %.2f", _stepsPerUnit);
    } else {
        _configComplete = false;
        _hasErrorState = true;
        _lastErrorMsg = "Setup failed";
        
        LOG_E(MODULE_PREFIX, "Setup failed");
    }
    
    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Calibrate (no-op for open-loop stepper)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepDriverTMC2209Enhanced::calibrate()
{
    // For open-loop steppers, calibration typically means:
    // 1. Reset position counter to zero
    // 2. Clear any accumulated errors
    
    _currentPosition = 0.0f;
    _totalSteps = 0;
    _currentVelocity = 0.0f;
    _hasErrorState = false;
    _lastErrorMsg = "";
    
    LOG_I(MODULE_PREFIX, "Calibration complete - position reset to zero");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Step function (legacy interface)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepDriverTMC2209Enhanced::step(bool direction)
{
    // Call parent implementation
    StepDriverTMC2209::setDirection(direction);
    StepDriverTMC2209::stepStart();
    // Note: stepEnd() is called by the RampGenerator
    
    // Update our position tracking
    if (direction != _lastDirection) {
        _lastDirection = direction;
    }
    
    // Update step count and timing for position/velocity tracking
    _totalSteps += direction ? 1 : -1;
    _lastStepTime = micros();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set direction (legacy interface)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StepDriverTMC2209Enhanced::setDirection(bool direction)
{
    StepDriverTMC2209::setDirection(direction);
    _lastDirection = direction;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get enhanced debug JSON
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String StepDriverTMC2209Enhanced::getEnhancedDebugJSON(bool includeBraces, bool detailed) const
{
    // Get base TMC2209 debug info
    String baseJson = StepDriverTMC2209::getDebugJSON(false, detailed);
    
    // Add our enhanced information
    String enhancedJson = "";
    enhancedJson += "\"motorType\":\"" + motorTypeToString(MotorType::STEPPER_OPEN_LOOP) + "\"";
    enhancedJson += ",\"position\":" + String(_currentPosition, 3);
    enhancedJson += ",\"velocity\":" + String(_currentVelocity, 3);
    enhancedJson += ",\"torque\":" + String(_currentTorque, 3);
    enhancedJson += ",\"enabled\":" + String(_isEnabledState ? "true" : "false");
    enhancedJson += ",\"moving\":" + String(isMoving() ? "true" : "false");
    enhancedJson += ",\"hasError\":" + String(_hasErrorState ? "true" : "false");
    enhancedJson += ",\"totalSteps\":" + String(_totalSteps);
    enhancedJson += ",\"stepsPerUnit\":" + String(_stepsPerUnit, 1);
    
    if (detailed) {
        enhancedJson += ",\"positionTarget\":" + String(_positionTarget, 3);
        enhancedJson += ",\"velocityTarget\":" + String(_velocityTarget, 3);
        enhancedJson += ",\"capabilities\":" + getCapabilitiesJSON();
    }
    
    // Combine with base JSON
    String combinedJson = baseJson;
    if (baseJson.length() > 0) {
        combinedJson += "," + enhancedJson;
    } else {
        combinedJson = enhancedJson;
    }
    
    return includeBraces ? "{" + combinedJson + "}" : combinedJson;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Private helper methods
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StepDriverTMC2209Enhanced::updatePositionFromSteps() const
{
    _currentPosition = static_cast<float>(_totalSteps) / _stepsPerUnit;
}

void StepDriverTMC2209Enhanced::updateVelocityEstimate() const
{
    uint32_t currentTime = micros();
    uint32_t timeDelta = currentTime - _lastStepTime;
    
    if (timeDelta < 1000000) { // Less than 1 second since last step
        // Estimate velocity from step interval
        if (timeDelta > 0) {
            float instantVelocity = (1.0f / _stepsPerUnit) / (timeDelta * 1e-6f); // units per second
            
            // Apply simple smoothing filter
            _currentVelocity = (1.0f - VELOCITY_SMOOTHING_FACTOR) * _currentVelocity + 
                              VELOCITY_SMOOTHING_FACTOR * instantVelocity;
        }
    } else {
        // No recent steps - assume stopped
        _currentVelocity = 0.0f;
    }
}

float StepDriverTMC2209Enhanced::estimateTorqueFromCurrent() const
{
    // Very rough torque estimate based on configured RMS current
    // This is a placeholder for Phase 1 - would need motor constants for accurate calculation
    // Use a default current estimate since we can't access private getMaxRMSAmps()
    float rmsAmps = 1.0f; // Default estimate, would be configured from setup in future
    
    // Rough approximation: assume 0.1 Nm per amp for typical NEMA17 stepper
    // This is highly motor-dependent and just for status reporting
    return rmsAmps * 0.1f;
}

bool StepDriverTMC2209Enhanced::validatePositionTarget(float position) const
{
    // Basic range checking - could be enhanced with axis limits
    const float MAX_POSITION = 1000000.0f; // Arbitrary large limit
    const float MIN_POSITION = -1000000.0f;
    
    if (position < MIN_POSITION || position > MAX_POSITION) {
        LOG_E(MODULE_PREFIX, "Position target %.2f out of range [%.0f, %.0f]", 
              position, MIN_POSITION, MAX_POSITION);
        return false;
    }
    
    return true;
}

bool StepDriverTMC2209Enhanced::validateVelocityTarget(float velocity) const
{
    // Basic range checking
    const float MAX_VELOCITY = 10000.0f; // units/sec
    
    if (fabs(velocity) > MAX_VELOCITY) {
        LOG_E(MODULE_PREFIX, "Velocity target %.2f exceeds maximum %.0f", 
              fabs(velocity), MAX_VELOCITY);
        return false;
    }
    
    return true;
}

String StepDriverTMC2209Enhanced::getCapabilitiesJSON() const
{
    auto caps = getCapabilities();
    String json = "{";
    json += "\"motorType\":\"" + motorTypeToString(caps.motorType) + "\"";
    json += ",\"supportedModes\":[";
    for (size_t i = 0; i < caps.supportedModes.size(); i++) {
        if (i > 0) json += ",";
        json += "\"" + controlModeToString(caps.supportedModes[i]) + "\"";
    }
    json += "]";
    json += ",\"hasPositionFeedback\":" + String(caps.hasPositionFeedback ? "true" : "false");
    json += ",\"hasVelocityFeedback\":" + String(caps.hasVelocityFeedback ? "true" : "false");
    json += ",\"hasTorqueFeedback\":" + String(caps.hasTorqueFeedback ? "true" : "false");
    json += ",\"maxVelocity\":" + String(caps.maxVelocity, 1);
    json += ",\"maxTorque\":" + String(caps.maxTorque, 2);
    json += ",\"positionResolution\":" + String(caps.positionResolution, 6);
    json += "}";
    return json;
}