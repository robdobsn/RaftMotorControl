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

class StepDriverBase;
class EndStops;

// #define DEBUG_MOTION_CONTROL_TIMER

class MotionController
{
public:
    // Constructor / Destructor
    MotionController();
    ~MotionController();

    // Setup / deinit
    void setup(const RaftJsonIF& config);
    void deinit();

    // Set serial bus and whether to use bus for direction reversal 
    void setupSerialBus(RaftBus* pBus, bool useBusForDirectionReversal);

    // Loop - called frequently
    void loop();

    // Move to a specific location
    bool moveTo(const MotionArgs &args);

    // Pause (or un-pause) all motion
    void pause(bool pauseIt);

    // Check if paused
    bool isPaused() const
    {
        return _isPaused;
    }

    // Check if busy (moving)
    bool isBusy() const;

    // Set current position as home
    void setCurPositionAsOrigin(bool allAxes = true, uint32_t axisIdx = 0);

    // Go to previously set home position
    void goToOrigin(const MotionArgs &args);

    // Get last commanded position
    AxesValues<AxisPosDataType> getLastCommandedPos() const;

    // Get last monitored position
    AxesValues<AxisPosDataType> getLastMonitoredPos() const;

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

    // Get debug str
    String getDebugStr() const;

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
    
    // Homing needed
    bool _homingNeededBeforeAnyMove = true;

    // Block distance
    double _maxBlockDistMM = 0.0f;

    // Pause status
    bool _isPaused = false;

    // Helpers
    void setupAxes(const RaftJsonIF& config);
    void setupAxisHardware(uint32_t axisIdx, const RaftJsonIF& config);
    void setupStepDriver(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig);
    void setupEndStops(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig);
    void setupRampGenerator(const RaftJsonIF& config);
    bool moveToNonRamped(const MotionArgs& args);
    bool moveToRamped(const MotionArgs& args);

    // Defaults
    static constexpr const char* DEFAULT_DRIVER_CHIP = "TMC2209";
    static constexpr const char* DEFAULT_HARDWARE_LOCATION = "local";
    static constexpr double distToTravel_ignoreBelow = 0.01f;
    static constexpr uint32_t MAX_TIME_BEFORE_STOP_COMPLETE_MS = 500;
};
