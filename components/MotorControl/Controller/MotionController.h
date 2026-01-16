/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionController
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesParams.h"
#include "RaftBus.h"
#include "MotionPlanner.h"
#include "MotionBlockManager.h"
#include "RampGenerator.h"
#include "RaftDeviceJSONLevel.h"
#include "RaftKinematics.h"
#include "MotionControlIF.h"
#include "MotionPatternManager.h"

class StepDriverBase;
class EndStops;

// #define DEBUG_MOTION_CONTROL_TIMER

class MotionController : public MotionControlIF
{
public:
    /// @brief Constructor
    MotionController();

    /// @brief Destructor
    ~MotionController();

    /// @brief Setup the motion controller
    /// @param config JSON configuration
    void setup(const RaftJsonIF& config);

    /// @brief Deinit the motion controller
    void deinit();

    /// @brief Setup bus to use for serial comms with driver and whether to use it for direction reversal
    void setupSerialBus(RaftBus* pBus, bool useBusForDirectionReversal);

    /// @brief Main loop for the device (called frequently)
    void loop();

    /// @brief Move to a specific location (flat or ramped and relative or absolute)
    /// @param args MotionArgs specify the motion to be performed
    /// @param respMsg Optional pointer to string for error message (default nullptr)
    /// @return RaftRetCode
    /// @note The args may be modified so cannot be const
    virtual RaftRetCode moveTo(MotionArgs &args, String* respMsg = nullptr) override;

    /// @brief Pause (or resume) all motion
    /// @param pauseIt true to pause, false to resume
    virtual void pause(bool pauseIt) override;

    /// @brief Check if the motion controller is paused
    /// @return true if paused
    virtual bool isPaused() const override
    {
        return _isPaused;
    }

    /// @brief Check if the motion controller is busy
    /// @return true if any motion is in the pipeline
    virtual bool isBusy() const override;

    // Set current position as origin
    virtual void setCurPositionAsOrigin(bool allAxes = true, uint32_t axisIdx = 0) override;

    // Go to previously set origin position
    void goToOrigin(const MotionArgs &args);

    // Get last commanded position
    virtual AxesValues<AxisPosDataType> getLastCommandedPos() const override;

    // Get last monitored position
    virtual AxesValues<AxisPosDataType> getLastMonitoredPos() const override;

    // Get total step counts recorded by ramp generator
    AxesValues<AxisStepsDataType> getAxisTotalSteps() const;

    // Get data (diagnostics)
    String getDataJSON(RaftDeviceJSONLevel level) const;

    // Get queue slots (buffers) available for streaming
    uint32_t streamGetQueueSlots() const;

    // Motor on time after move
    void setMotorOnTimeAfterMoveSecs(float motorOnTimeAfterMoveSecs)
    {
        _motorEnabler.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
    }

    // Set max motor current (amps)
    void setMaxMotorCurrentAmps(uint32_t axisIdx, float maxMotorCurrent);

    // Get debug JSON
    String getDebugJSON(bool includeBraces) const;

    // Get end-stop state for an axis (min or max)
    // Returns true if triggered, false otherwise. Sets isFresh to true if valid, false if not configured.
    virtual bool getEndStopState(uint32_t axisIdx, bool max, bool& isFresh) const override;

    /// @brief Stop current motion pattern
    virtual void stopPattern() override;

    /// @brief Add a motion pattern to the registry
    /// @param patternName Name of the pattern
    /// @param createFn Factory function to create pattern instance
    void addMotionPattern(const String& patternName, MotionPatternCreateFn createFn);

    /// @brief Start a motion pattern
    /// @param patternName Name of pattern to start
    /// @param patternRunTimeDefaultMs Default runtime in milliseconds (0 = run forever)
    /// @param pParamsJson Optional JSON parameters for pattern
    void setMotionPattern(const String& patternName, uint32_t patternRunTimeDefaultMs = 0, const char* pParamsJson = nullptr);

    /// @brief Check if motion pattern is currently active
    /// @return true if pattern is running
    bool isMotionPatternActive() const;

    /// @brief Get name of current motion pattern
    /// @return Current pattern name (empty if none)
    const String& getCurrentMotionPatternName() const;

    /// @brief Set named value provider for patterns
    /// @param pNamedValueProvider Pointer to named value provider
    void setPatternNamedValueProvider(NamedValueProvider* pNamedValueProvider);

private:
    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionController";

    // Axis stepper motors
    std::vector<StepDriverBase*> _stepperDrivers;

    // Axis end-stops
    std::vector<EndStops*> _axisEndStops;

    // Axes parameters
    AxesParams _axesParams;

    // Ramp generator
    RampGenerator _rampGenerator;

    // Motion block manager
    MotionBlockManager _blockManager;

    // Motor enabler - handles timeout of motor movement
    MotorEnabler _motorEnabler;

    // Motion pattern manager
    MotionPatternManager _patternManager;

    // Homing needed
    bool _homingNeededBeforeAnyMove = true;

    // Pause status
    bool _isPaused = false;

    // Helpers
    void setupAxes(const RaftJsonIF& config);
    void setupAxisHardware(uint32_t axisIdx, const RaftJsonIF& config);
    void setupStepDriver(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig);
    void setupEndStops(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig);
    void setupRampGenerator(const RaftJsonIF& config);
    bool moveToNonRamped(const MotionArgs& args);

    /// @brief Move to a specific location (relative or absolute) using ramped motion
    /// @param args MotionArgs specify the motion to be performed
    /// @param respMsg Optional pointer to string for error message (default nullptr)
    /// @return RaftRetCode
    /// @note The args may be modified so cannot be const
    RaftRetCode moveToRamped(MotionArgs& args, String* respMsg = nullptr);

    // Defaults
    static constexpr const char* DEFAULT_DRIVER_CHIP = "TMC2209";
    static constexpr const char* DEFAULT_HARDWARE_LOCATION = "local";
    static constexpr double distToTravel_ignoreBelow = 0.01f;
    static constexpr uint32_t MAX_TIME_BEFORE_STOP_COMPLETE_MS = 500;
};
