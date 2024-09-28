/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionController
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "MotionController.h"
#include "ConfigPinMap.h"
#include "RaftUtils.h"
#include "AxesValues.h"
#include "StepDriverBase.h"
#include "StepDriverTMC2209.h"
#include "EndStops.h"
#include "RaftArduino.h"
#include "RaftJsonPrefixed.h"

#define DEBUG_STEPPER_SETUP_CONFIG
#define DEBUG_RAMP_SETUP_CONFIG
#define DEBUG_MOTION_CONTROLLER
#define INFO_LOG_AXES_PARAMS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
MotionController::MotionController() : 
            _blockManager(_motorEnabler, _axesParams)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
MotionController::~MotionController()
{
    // Clear all steppers
    deinit();

    // Clear block manager
    _blockManager.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup
/// @param config Configuration (from JSON)
void MotionController::setup(const RaftJsonIF& config)
{
    // De-init first
    deinit();

    // Setup axes (and associated hardware)
    setupAxes(config);
#ifdef INFO_LOG_AXES_PARAMS
    _axesParams.debugLog();
#endif

    // Setup ramp generator and pipeline
    RaftJsonPrefixed rampConfig(config, "ramp");
    _rampGenerator.setup(rampConfig, _stepperDrivers, _axisEndStops);
    _rampGenerator.start();

    // Setup motor enabler
    RaftJsonPrefixed motorEnConfig(config, "motorEn");
    _motorEnabler.setup(motorEnConfig);

    // Block manager
    RaftJsonPrefixed motionConfig(config, "motion");
    _blockManager.setup(_rampGenerator.getPeriodUs(), motionConfig);

    // If no homing required then set the current position as home
    if (!_homingNeededBeforeAnyMove)
        setCurPositionAsOrigin(true);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief De-init (teardown)
void MotionController::deinit()
{
    // Stop any motion
    _rampGenerator.stop();
    _motorEnabler.deinit();

    // Clear block manager
    _blockManager.clear();

    // Remote steppers
    for (StepDriverBase*& pDriver : _stepperDrivers)
    {
        if (pDriver)
            delete pDriver;
        pDriver = nullptr;
    }

    // Remote endstops
    for (EndStops*& pEndStops : _axisEndStops)
    {
        if (pEndStops)
            delete pEndStops;
    }
    _axisEndStops.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Loop
/// @note Called frequently to allow the MotionController to do background work such as adding split-up blocks to the
/// pipeline and checking if motors should be disabled after a period of no motion
void MotionController::loop()
{
    // LOG_I(MODULE_PREFIX, "loop stepper drivers %d", _stepperDrivers.size());

    // Loop stepper drivers
    for (StepDriverBase* pStepDriver : _stepperDrivers)
    {
        // LOG_I(MODULE_PREFIX, "loop stepper driver %d", pStepDriver->getSerialAddress());
        if (pStepDriver)
            pStepDriver->loop();
    }

    // // Check if stop requested
    // if (_stopRequested)
    // {
    //     if (Raft::isTimeout(millis(), _stopRequestTimeMs, MAX_TIME_BEFORE_STOP_COMPLETE_MS))
    //     {
    //         _blocksToAddTotal = 0;
    //         _rampGenerator.stop();
    //         _trinamicsController.stop();
    //         _motionPipeline.clear();
    //         pause(false);
    //         setCurPosActualPosition();
    //         _stopRequested = false;
    //     }
    // }

    // Loop motor enabler
    _motorEnabler.loop();
    
    // Call process on motion actuator - only really used for testing as
    // motion is handled by ISR
    _rampGenerator.loop();

    // Process for trinamic devices
    // TODO
    // _trinamicsController.process();

    // Process any split-up blocks to be added to the pipeline
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline());

    // Loop homing
    // TODO
    // _motionHoming.loop(_axesParams);

    // Ensure motors enabled when homing or moving
    if ((_rampGenerator.getMotionPipeline().count() > 0) 
        // TODO 2021
        //  || _motionHoming.isHomingInProgress()
         )
    {
        _motorEnabler.enableMotors(true, false);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Check if the motion controller is busy
/// @return true if any motion is in the pipeline
bool MotionController::isBusy() const
{
    return _rampGenerator.getMotionPipelineConst().count() > 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Move to a specific location (flat or ramped and relative or absolute)
/// @param args MotionArgs specify the motion to be performed
/// @return true if the motion was successfully added to the pipeline
/// @note The args may be modified so cannot be const
bool MotionController::moveTo(MotionArgs &args)
{
    LOG_I(MODULE_PREFIX, "moveTo %s args %s", 
            args.getAxesPos().getDebugJSON("axes").c_str(),
            args.toJSON().c_str());

    // Handle stop
    if (args.isStopMotion())
    {
        _rampGenerator.stop();
    }
    
    // Handle clear queue
    if (args.isClearQueue())
    {
        _blockManager.clear();
    }

    // Handle disable motors
    if (!args.isEnableMotors())
    {
        _motorEnabler.enableMotors(false, false);
        return true;
    }

    // Check motion type
    if (args.isRamped())
    {
        // Ramped (variable speed) motion
        return moveToRamped(args);
    }

    // Handle flat motion (no ramp) - motion is defined in terms of steps (not mm)
    return _blockManager.addNonRampedBlock(args, _rampGenerator.getMotionPipeline());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Pause (or resume) all motion
/// @param pauseIt true to pause, false to resume
void MotionController::pause(bool pauseIt)
{
    _rampGenerator.pause(pauseIt);
    // _trinamicsController.pause(pauseIt);
    _isPaused = pauseIt;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Move to a specific location (relative or absolute) using ramped motion
/// @param args MotionArgs specify the motion to be performed
/// @return true if the motion was successfully added to the pipeline
/// @note The args may be modified so cannot be const
bool MotionController::moveToRamped(MotionArgs& args)
{
    // Check not busy
    if (_blockManager.isBusy())
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo busy");
#endif
        return false;
    }

    // Check the last commanded position is valid (homing complete, no stepwise movement, etc)
    if (_blockManager.isHomingNeededBeforeMove() && (!_blockManager.isAxesStateValid()))
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo lastPos invalid - need to home (initially and after non-ramped moves)");
#endif
        return false;
    }

    // Pre-process coordinates - this fills in unspecified values for axes and handles relative motion
    AxisPosDataType moveDistanceMM = _blockManager.preProcessCoords(args);

    // Ensure at least one block
    uint32_t numBlocks = 1;
    double maxBlockDistMM = _axesParams.getMaxBlockDistMM();
    if (maxBlockDistMM > 0.01f && !args.dontSplitMove())
        numBlocks = int(ceil(moveDistanceMM / maxBlockDistMM));
    if (numBlocks == 0)
        numBlocks = 1;

#ifdef DEBUG_MOTION_CONTROLLER
    LOG_I(MODULE_PREFIX, "moveToRamped %s moveDistanceMM %.2f maxBlockDist %.2f numBlocks %d",
                args.getAxesPos().getDebugJSON("pos").c_str(), moveDistanceMM, maxBlockDistMM, numBlocks);
#endif

    // Add to the block splitter
    _blockManager.addRampedBlock(args, numBlocks);

    // Pump the block splitter to prime the pipeline with blocks
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline());

    // Ok
    return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set current position of the axes as the origin
/// @param allAxes true to set all axes, false to set a specific axis
/// @param axisIdx if allAxes is false, the axis to set
void MotionController::setCurPositionAsOrigin(bool allAxes, uint32_t axisIdx)
{
    if (!allAxes && (axisIdx >= AXIS_VALUES_MAX_AXES))
        return;
    for (uint32_t i = (allAxes ? 0 : axisIdx); i < (allAxes ? AXIS_VALUES_MAX_AXES : axisIdx+1); i++)
    {
        _rampGenerator.setTotalStepPosition(i, 0);
        _blockManager.setCurPositionAsOrigin(i);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Go to previously set origin position
void MotionController::goToOrigin(const MotionArgs &args)
{
    // TODO - implement
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get status data in JSON format
/// @param level Level of detail required
/// @return JSON string
String MotionController::getDataJSON(RaftDeviceJSONLevel level) const
{
    String jsonBody;
    if (level >= DEVICE_JSON_LEVEL_MIN)
    {
        jsonBody += "\"ramp\":" + _rampGenerator.getStats().getJSON();
        String driverJson;
        for (StepDriverBase* pStepDriver : _stepperDrivers)
        {
            if (pStepDriver)
            {
                driverJson += driverJson.length() > 0 ? "," : "";
                driverJson += pStepDriver->getStatusJSON(true, level == DEVICE_JSON_LEVEL_FULL);
            }
        }
        if (driverJson.length() > 0)
            jsonBody += ",\"drivers\":[" + driverJson + "]";
        return "{" + jsonBody + "}";
    }
    return "{}";
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get number of queue slots available for streaming
/// @return Number of slots
uint32_t MotionController::streamGetQueueSlots() const
{
    return _rampGenerator.getMotionPipelineConst().remaining();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup axes
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupAxes(const RaftJsonIF& config)
{
    // Setup stepper driver array
    _stepperDrivers.resize(AXIS_VALUES_MAX_AXES);
    for (auto& pDriver : _stepperDrivers)
        pDriver = nullptr;

    // Setup axes params
    _axesParams.setupAxes(config);

    // Extract hardware related to axes
    std::vector<String> axesVec;
    if (config.getArrayElems("axes", axesVec))
    {
        uint32_t axisIdx = 0;
        for (RaftJson axisConfig : axesVec)
        {
            // Setup driver
            setupAxisHardware(axisIdx, axisConfig);

            // Next
            axisIdx++;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup axis hardware
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupAxisHardware(uint32_t axisIdx, const RaftJsonIF& config)
{
    // Axis name
    String axisName = config.getString("name", "");

    // Configure the driver
    setupStepDriver(axisIdx, axisName, "driver", config);

    // Configure the endstops
    setupEndStops(axisIdx, axisName, "endstops", config);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup stepper driver
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupStepDriver(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig)
{
    // Get config with prefix
    RaftJsonPrefixed config(mainConfig, jsonElem);

    // Stepper handler
    String hwLocation = config.getString("hw", DEFAULT_HARDWARE_LOCATION);
    String driverType = config.getString("driver", DEFAULT_DRIVER_CHIP);

    // Stepper parameters
    StepDriverParams stepperParams(config);

    // Handle location
    StepDriverBase* pStepDriver = nullptr; 
    if (hwLocation.equalsIgnoreCase("local"))
    {
        // Check driver type
        if (driverType.equalsIgnoreCase("tmc2209"))
        {
            pStepDriver = new StepDriverTMC2209();
        }
        if (pStepDriver)
        {
            pStepDriver->setup(axisName, stepperParams, _rampGenerator.isUsingTimerISR());
        }
        // Debug
#ifdef DEBUG_STEPPER_SETUP_CONFIG
        LOG_I(MODULE_PREFIX, "setupStepDriver %s axisName %s driver %s %s",
                pStepDriver ? "local" : "FAILED",
                axisName.c_str(), 
                driverType.c_str(),
                stepperParams.getDebugJSON().c_str());
    #endif
    }

    // Add driver
    if (axisIdx < _stepperDrivers.size())
        _stepperDrivers[axisIdx] = pStepDriver;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup end stops
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupEndStops(uint32_t axisIdx, const String& axisName, const char* jsonElem, const RaftJsonIF& mainConfig)
{
    // Endstops
    EndStops* pEndStops = new EndStops();

    // Config
    std::vector<String> endstopVec;
    if (mainConfig.getArrayElems(jsonElem, endstopVec))
    {
        for (RaftJson endstopConfig : endstopVec)
        {
            // Debug
            // LOG_I(MODULE_PREFIX, "setupEndStops %s", endstopConfigStr.c_str());

            // Setup endstop
            bool isMax = endstopConfig.getBool("isMax", false);
            String name = endstopConfig.getString("name", "");
            String endstopPinName = endstopConfig.getString("sensePin", "-1");
            int pin = ConfigPinMap::getPinFromName(endstopPinName.c_str());
            bool activeLevel = endstopConfig.getBool("actLvl", false);
            String inputTypeStr = endstopConfig.getString("inputType", "INPUT_PULLUP");
            int inputType = ConfigPinMap::getInputType(inputTypeStr.c_str());
            if (pEndStops)
                pEndStops->add(isMax, name.c_str(), pin, activeLevel, inputType);
            LOG_I(MODULE_PREFIX, "setupEndStops isMax %d name %s pin %d, activeLevel %d, pinMode %d", 
                        isMax, name.c_str(), pin, activeLevel, inputType);
        }
    }

    // Add
    _axisEndStops.push_back(pEndStops);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup serial bus and bus reversal
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupSerialBus(RaftBus* pBus, bool useBusForDirectionReversal)
{
    // Setup bus
    for (StepDriverBase* pStepDriver : _stepperDrivers)
    {
        if (pStepDriver)
            pStepDriver->setupSerialBus(pBus, useBusForDirectionReversal);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set max motor current
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setMaxMotorCurrentAmps(uint32_t axisIdx, float maxMotorCurrentAmps)
{
    // Set max motor current
    if (axisIdx < _stepperDrivers.size())
        _stepperDrivers[axisIdx]->setMaxMotorCurrentAmps(maxMotorCurrentAmps);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get last monitored position
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AxesValues<AxisPosDataType> MotionController::getLastMonitoredPos() const
{
    // Get current position
    AxesValues<AxisStepsDataType> curActuatorPos;
    _rampGenerator.getTotalStepPosition(curActuatorPos);
    // Use reverse kinematics to get location
    AxesValues<AxisPosDataType> lastMonitoredPos;
    _blockManager.actuatorToPt(curActuatorPos, lastMonitoredPos);
    return lastMonitoredPos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get debug string
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String MotionController::getDebugJSON(bool includeBraces) const
{
    String jsonStr = _rampGenerator.getDebugJSON(false) + ",";
    jsonStr += getLastMonitoredPos().getDebugJSON("pos", false);
    for (StepDriverBase* pStepDriver : _stepperDrivers)
    {
        if (pStepDriver)
        {
            jsonStr += ",";
            jsonStr += pStepDriver->getStatusJSON(true, true);
        }
    }
    for (EndStops* pEndStops : _axisEndStops)
    {
        if (pEndStops)
        {
            jsonStr += ",";
            jsonStr += pEndStops->getDebugJSON(true, true);
        }
    }
    return includeBraces ? "{" + jsonStr + "}" : jsonStr;
}
