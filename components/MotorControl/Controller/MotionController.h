/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionController
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AxesParams.h"
#include <BusBase.h>
#include <HWElemConsts.h>

class StepDriverBase;
class ConfigBase;
class EndStops;

// #define DEBUG_MOTION_CONTROL_TIMER

#include "Controller/MotionPlanner.h"
#include "Controller/MotionBlockManager.h"
#include "RampGenerator/RampGenerator.h"
#include "RampGenerator/RampGenTimer.h"

class AxisGeomBase;

class MotionController
{
public:
    // Constructor / Destructor
    MotionController();
    ~MotionController();

    // Setup / teardown
    void setup(const ConfigBase& config, const char* pConfigPrefix);
    void teardown();

    // Set serial bus and whether to use bus for direction reversal 
    void setupSerialBus(BusBase* pBus, bool useBusForDirectionReversal);

    // Service - called frequently
    void service();

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
    void setCurPositionAsHome(bool allAxes = true, uint32_t axisIdx = 0);

    // Go to previously set home position
    void goHome(const MotionArgs &args);

    // Get last commanded position
    AxesPosValues getLastCommandedPos() const;

    // Get last monitored position
    AxesPosValues getLastMonitoredPos() const;

    // Get data (diagnostics)
    String getDataJSON(HWElemStatusLevel_t level);

    // Get queue slots (buffers) available for streaming
    uint32_t streamGetQueueSlots();

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
    // Ramp generation
    RampGenTimer _rampGenTimer;

    // Axis stepper motors
    std::vector<StepDriverBase*> _stepperDrivers;

    // Axis end-stops
    std::vector<EndStops*> _axisEndStops;

    // Axes parameters
    AxesParams _axesParams;

    // Ramp generator
    RampGenerator _rampGenerator;

    // Motion pipeline to add blocks to
    MotionPipeline _motionPipeline;

    // Motion block manager
    MotionBlockManager _blockManager;

    // Motor enabler - handles timeout of motor movement
    MotorEnabler _motorEnabler;

    // Ramp timer enabled
    bool _rampTimerEn = false;

    // Homing needed
    bool _homingNeededBeforeAnyMove = true;

    // Block distance
    double _blockDistance = 0.0f;

    // Pause status
    bool _isPaused = false;

    // Helpers
    void deinit();
    void setupAxes(const ConfigBase& config, const char* pConfigPrefix);
    void setupAxisHardware(uint32_t axisIdx, const ConfigBase& config);
    void setupStepDriver(uint32_t axisIdx, const String& axisName, const char* jsonElem, const ConfigBase& mainConfig);
    void setupEndStops(uint32_t axisIdx, const String& axisName, const char* jsonElem, const ConfigBase& mainConfig);
    void setupRampGenerator(const char* jsonElem, const ConfigBase& config, const char* pConfigPrefix);
    void setupMotorEnabler(const char* jsonElem, const ConfigBase& config, const char* pConfigPrefix);
    void setupMotionControl(const char* jsonElem, const ConfigBase& config, const char* pConfigPrefix);
    bool moveToLinear(const MotionArgs& args);
    bool moveToRamped(const MotionArgs& args);

    // Defaults
    static constexpr const char* DEFAULT_DRIVER_CHIP = "TMC2209";
    static constexpr const char* DEFAULT_HARDWARE_LOCATION = "local";
    static constexpr double _blockDistance_default = 0.0f;
    static constexpr double junctionDeviation_default = 0.05f;
    static constexpr double distToTravel_ignoreBelow = 0.01f;
    static constexpr uint32_t pipelineLen_default = 100;
    static constexpr uint32_t MAX_TIME_BEFORE_STOP_COMPLETE_MS = 500;

    // Debug
    uint32_t _debugLastLoopMs = 0;

#ifdef DEBUG_MOTION_CONTROL_TIMER
    volatile uint32_t _testRampGenCount;
    IRAM_ATTR void rampGenTimerCallback(void* pObj)
    {
        if (pObj)
        {
            MotionController* pMotionController = (MotionController*)pObj;
            pMotionController->_testRampGenCount++;
        }        
    }
#endif

};
