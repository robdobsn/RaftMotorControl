/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionController
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <math.h>
#include "RaftCore.h"
#include "MotionController.h"
#include "AxesValues.h"
#include "StepDriverBase.h"
#include "StepDriverTMC2209.h"
#include "EndStops.h"

// #define DEBUG_STEPPER_SETUP_CONFIG
// #define DEBUG_RAMP_SETUP_CONFIG
// #define DEBUG_MOTION_CONTROLLER
// #define INFO_LOG_AXES_PARAMS
// #define DEBUG_MOVE_TO_COMMAND
// #define DEBUG_LOOP_STEPPER_DRIVER

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
/// @param timeNowMs Current system time in milliseconds
void MotionController::setup(const RaftJsonIF& config, uint32_t timeNowMs)
{
    // De-init first
    deinit();

    // Setup axes (and associated hardware)
    setupAxes(config, timeNowMs);
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
/// @param timeNowMs Current system time in milliseconds (only relevant for debug or non-timer ISR)
/// @param nonTimerIntervalMs Interval between calls if not using timer ISR
/// @note Called frequently to allow the MotionController to do background work such as adding split-up blocks to the
/// pipeline and checking if motors should be disabled after a period of no motion
void MotionController::loop(uint32_t timeNowMs, uint32_t nonTimerIntervalMs)
{
    // Loop stepper drivers
    for (StepDriverBase* pStepDriver : _stepperDrivers)
    {
#ifdef DEBUG_LOOP_STEPPER_DRIVER
        LOG_I(MODULE_PREFIX, "loop stepper driver %d", pStepDriver ? pStepDriver->getSerialAddress() : -1);
#endif
        if (pStepDriver)
            pStepDriver->loop(timeNowMs);
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
    _motorEnabler.loop(timeNowMs);
    
    // Call process on motion actuator - only really used for testing as
    // motion is handled by ISR
    _rampGenerator.loop(timeNowMs, nonTimerIntervalMs);

    // Process for trinamic devices
    // TODO
    // _trinamicsController.process();

    // Process any split-up blocks to be added to the pipeline
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline(), timeNowMs);

    // Loop homing
    // TODO
    // _motionHoming.loop(_axesParams);

    // Ensure motors enabled when homing or moving
    if ((_rampGenerator.getMotionPipeline().count() > 0) 
        // TODO 2021
        //  || _motionHoming.isHomingInProgress()
         )
    {
        _motorEnabler.enableMotors(true, false, timeNowMs);
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
/// @param timeNowMs Current system time in milliseconds
/// @return RaftRetCode
/// - RAFT_OK if the motion was successfully added to the pipeline
/// - RAFT_BUSY if the pipeline is full
/// - RAFT_INVALID_DATA if geometry not set
/// - RAFT_INVALID_OPERATION if homing is needed
/// - RAFT_CANNOT_START if no movement
/// @note The args may be modified so cannot be const
RaftRetCode MotionController::moveTo(MotionArgs &args, uint32_t timeNowMs)
{
#ifdef DEBUG_MOVE_TO_COMMAND
    LOG_I(MODULE_PREFIX, "moveTo %s args %s", 
            args.getAxesPos().getDebugJSON("axes").c_str(),
            args.toJSON().c_str());
#endif

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
        _motorEnabler.enableMotors(false, false, timeNowMs);
        return RAFT_OK;
    }

    // Check motion type
    if (args.isRamped())
    {
        // Ramped (variable speed) motion
        return moveToRamped(args, timeNowMs);
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
/// @param timeNowMs Current system time in milliseconds
/// @return RaftRetCode
/// - RAFT_OK if the motion was successfully added to the pipeline
/// - RAFT_BUSY if the pipeline is full
/// - RAFT_INVALID_DATA if geometry not set
/// - RAFT_INVALID_OPERATION if homing is needed
/// - RAFT_CANNOT_START if no movement
/// @note The args may be modified so cannot be const
RaftRetCode MotionController::moveToRamped(MotionArgs& args, uint32_t timeNowMs)
{
    // Check not busy
    if (_blockManager.isBusy())
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo busy");
#endif
        return RAFT_BUSY;
    }

    // Check the last commanded position is valid (homing complete, no stepwise movement, etc)
    if (_blockManager.isHomingNeededBeforeMove() && (!_blockManager.isAxesStateValid()))
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo lastPos invalid - need to home (initially and after non-ramped moves)");
#endif
        return RAFT_INVALID_OPERATION;
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
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline(), timeNowMs);

    // Ok
    return RAFT_OK;
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

void MotionController::setupAxes(const RaftJsonIF& config, uint32_t timeNowMs)
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
            setupAxisHardware(axisIdx, axisConfig, timeNowMs);

            // Next
            axisIdx++;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup axis hardware
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupAxisHardware(uint32_t axisIdx, const RaftJsonIF& config, uint32_t timeNowMs)
{
    // Axis name
    String axisName = config.getString("name", "");

    // Configure the driver
    setupStepDriver(axisIdx, axisName, "driver", config, timeNowMs);

    // Configure the endstops
    setupEndStops(axisIdx, axisName, "endstops", config);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup stepper driver
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupStepDriver(uint32_t axisIdx, const String& axisName, const char* jsonElem, 
            const RaftJsonIF& mainConfig, uint32_t timeNowMs)
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
            pStepDriver->setup(axisName, stepperParams, _rampGenerator.isUsingTimerISR(), timeNowMs);
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
/// @brief Set max motor current (amps)
/// @param axisIdx Axis index
/// @param maxMotorCurrent Max motor current (amps)
/// @param timeNowMs Current time in milliseconds
/// @return RaftRetCode
/// - RAFT_OK if the current was set
/// - RAFT_INVALID_DATA if the axis index is invalid
RaftRetCode MotionController::setMaxMotorCurrentAmps(uint32_t axisIdx, float maxMotorCurrentAmps, uint32_t timeNowMs)
{
    // Set max motor current
    if (axisIdx < _stepperDrivers.size())
        return _stepperDrivers[axisIdx]->setMaxMotorCurrentAmps(maxMotorCurrentAmps, timeNowMs);
    return RAFT_INVALID_DATA;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get last monitored position
/// @param actuatorPos Actuator position
/// @param realWorldPos Real world position
void MotionController::getLastMonitoredPos(AxesValues<AxisStepsDataType>& actuatorPos, AxesValues<AxisPosDataType>& realWorldPos) const
{
    // Get current position
    _rampGenerator.getTotalStepPosition(actuatorPos);
    // Use reverse kinematics to get location
    _blockManager.actuatorToPt(actuatorPos, realWorldPos);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get debug string
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String MotionController::getDebugJSON(bool includeBraces) const
{
    String jsonStr = _rampGenerator.getDebugJSON(false) + ",";
    AxesValues<AxisStepsDataType> actuatorPos;
    AxesValues<AxisPosDataType> realWorldPos;
    getLastMonitoredPos(actuatorPos, realWorldPos);
    jsonStr += realWorldPos.getDebugJSON("pos", false);
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
