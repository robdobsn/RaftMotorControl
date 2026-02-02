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

// #define DEBUG_STEPPER_SETUP_CONFIG
// #define DEBUG_RAMP_SETUP_CONFIG
// #define DEBUG_MOTION_CONTROLLER
// #define DEBUG_MOTION_CONTROLLER_TIMINGS
// #define INFO_LOG_AXES_PARAMS
// #define DEBUG_ENDSTOP_STATUS

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
        setCurPositionAsOrigin();
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
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline(), nullptr);

    // Loop motion patterns
    _patternManager.loop(*this);

    // Ensure motors enabled when homing or moving
    if ((_rampGenerator.getMotionPipeline().count() > 0) || _patternManager.isPatternActive())
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
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
/// @note The args may be modified so cannot be const
RaftRetCode MotionController::moveTo(MotionArgs &args, String* respMsg)
{
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    uint64_t startTimeUs = micros();
#endif

#ifdef DEBUG_MOTION_CONTROLLER
    LOG_I(MODULE_PREFIX, "moveTo %s args %s", 
            args.getAxesPos().getDebugJSON("axes").c_str(),
            args.toJSON().c_str());
#endif

    // Handle immediate execution (stop current motion, clear queue, then execute this motion)
    if (args.isImmediateExecution())
    {
        _rampGenerator.stop();
        _blockManager.clear();
    }

    // Check for velocity mode
    if (args.isVelocityMode())
    {
        RaftRetCode rc = moveToVelocity(args, respMsg);
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
        uint64_t totalTimeUs = micros() - startTimeUs;
        LOG_I(MODULE_PREFIX, "moveTo TIMING: total=%lluus cmd=velocity", totalTimeUs);
#endif
        return rc;
    }

    // Check motion type
    if (args.isRamped())
    {
        // Ramped (variable speed) motion
        RaftRetCode rc = moveToRamped(args, respMsg);
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
        uint64_t totalTimeUs = micros() - startTimeUs;
        LOG_I(MODULE_PREFIX, "moveTo TIMING: total=%lluus cmd=ramped", totalTimeUs);
#endif
        return rc;
    }

    // Handle flat motion (no ramp) - motion is defined in terms of steps (not mm)
    bool success = _blockManager.addNonRampedBlock(args, _rampGenerator.getMotionPipeline());
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "moveTo TIMING: total=%lluus cmd=nonRamped", totalTimeUs);
#endif
    return success ? RAFT_OK : RAFT_OTHER_FAILURE;
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
/// @brief Stop all motion immediately (clears queue and stops ramp generator)
/// @param disableMotors If true, disable motors after stopping
void MotionController::stopAll(bool disableMotors)
{
    // Stop the ramp generator (halts current step generation)
    _rampGenerator.stop();
    
    // Clear the motion queue (removes all pending motion blocks)
    _blockManager.clear();
    
    // Optionally disable motors
    if (disableMotors)
    {
        _motorEnabler.enableMotors(false, false);
    }
    
    _isPaused = false;  // Not paused, fully stopped
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Move to a specific location (relative or absolute) using ramped motion
/// @param args MotionArgs specify the motion to be performed
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
/// @note The args may be modified so cannot be const
RaftRetCode MotionController::moveToRamped(MotionArgs& args, String* respMsg)
{
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    uint64_t startTimeUs = micros();
    uint64_t preProcessStartUs, addBlockStartUs, pumpStartUs;
#endif

    // Check not busy
    if (_blockManager.isBusy())
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo busy");
#endif
        return RAFT_MOTION_BUSY;
    }

    // Check the last commanded position is valid (homing complete, no stepwise movement, etc)
    if (_blockManager.isHomingNeededBeforeMove() && (!_blockManager.isAxesStateValid()))
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo lastPos invalid - need to home (initially and after non-ramped moves)");
#endif
        return RAFT_MOTION_HOMING_REQUIRED;
    }

    // Pre-process coordinates - this fills in unspecified values for axes and handles relative motion
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    preProcessStartUs = micros();
#endif
    AxisPosDataType moveDistanceMM = _blockManager.preProcessCoords(args);
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    uint64_t preProcessTimeUs = micros() - preProcessStartUs;
#endif

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
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    addBlockStartUs = micros();
#endif
    _blockManager.addRampedBlock(args, numBlocks);
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    uint64_t addBlockTimeUs = micros() - addBlockStartUs;
#endif

    // Pump the block splitter to prime the pipeline with blocks
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    pumpStartUs = micros();
#endif
    RaftRetCode rc = _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline(), respMsg);
#ifdef DEBUG_MOTION_CONTROLLER_TIMINGS
    uint64_t pumpTimeUs = micros() - pumpStartUs;
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "moveToRamped TIMING: total=%lluus preProcess=%lluus addBlock=%lluus pump=%lluus numBlocks=%d",
          totalTimeUs, preProcessTimeUs, addBlockTimeUs, pumpTimeUs, numBlocks);
#endif
    if (rc != RAFT_OK)
        return rc;

    // Ok
    return RAFT_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Start velocity mode motion
/// @param args MotionArgs specify the velocities for each axis
/// @param respMsg Optional pointer to string for error message
/// @return RaftRetCode
RaftRetCode MotionController::moveToVelocity(MotionArgs& args, String* respMsg)
{
    // For velocity mode, we need to stop any existing motion and replace with velocity block
    // This ensures immediate response to velocity commands
    _rampGenerator.stop();
    _blockManager.clear();

    // Add velocity block
    RaftRetCode rc = _blockManager.addVelocityBlock(args, 
                                                     _rampGenerator.getMotionPipeline(),
                                                     _rampGenerator.getMinStepRatePerTTicks(),
                                                     respMsg);
    
    if (rc != RAFT_OK)
    {
        LOG_W(MODULE_PREFIX, "moveToVelocity failed: %s", Raft::getRetCodeStr(rc));
        return rc;
    }

    LOG_I(MODULE_PREFIX, "moveToVelocity started - vel %s", 
          args.getVelocitiesConst().getDebugJSON("vel").c_str());
    
    return RAFT_OK;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set current position of the axes as the origin
void MotionController::setCurPositionAsOrigin()
{
    _rampGenerator.resetTotalStepPosition();
    _blockManager.setCurPositionAsOrigin();
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
    // Check for publish level - return operational state data
    if (level == DEVICE_JSON_LEVEL_PUBLISH)
    {
        // Get current state
        AxesValues<AxisPosDataType> monPos = getLastMonitoredPos();
        AxesValues<AxisStepsDataType> steps = getAxisTotalSteps();
        bool busy = isBusy();
        bool paused = isPaused();
        const String& pattern = getCurrentMotionPatternName();
        
        // Build JSON string
        String json = "{";
        
        // Position array
        json += "\"pos\":[";
        for (uint32_t i = 0; i < monPos.numAxes(); i++)
        {
            if (i > 0) json += ",";
            json += String(monPos.getVal(i), 2);  // 2 decimal places
        }
        json += "],";
        
        // Step counts array
        json += "\"steps\":[";
        for (uint32_t i = 0; i < steps.numAxes(); i++)
        {
            if (i > 0) json += ",";
            json += String((long)steps.getVal(i));
        }
        json += "],";
        
        // Status fields
        json += "\"busy\":";
        json += busy ? "true" : "false";
        json += ",\"paused\":";
        json += paused ? "true" : "false";
        json += ",\"pattern\":\"";
        json += pattern;
        json += "\"}";
        
        return json;
    }

    // Other levels - return diagnostic info
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
/// @brief Form binary data response for publishing
/// @param data (out) Data buffer to fill with binary data
void MotionController::formBinaryDataResponse(std::vector<uint8_t>& data) const
{
    // Timestamp (16-bit, lower bits of millis)
    uint16_t timeVal = (uint16_t)(millis() & 0xFFFF);
    data.push_back((timeVal >> 8) & 0xFF);
    data.push_back(timeVal & 0xFF);
    
    // Position values (3 x float, big-endian)
    AxesValues<AxisPosDataType> pos = getLastMonitoredPos();
    for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        float val = pos.getVal(i);
        uint32_t floatBits;
        memcpy(&floatBits, &val, sizeof(float));
        data.push_back((floatBits >> 24) & 0xFF);
        data.push_back((floatBits >> 16) & 0xFF);
        data.push_back((floatBits >> 8) & 0xFF);
        data.push_back(floatBits & 0xFF);
    }
    
    // Step counts (3 x int32, big-endian)
    AxesValues<AxisStepsDataType> steps = getAxisTotalSteps();
    for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        int32_t val = steps.getVal(i);
        data.push_back((val >> 24) & 0xFF);
        data.push_back((val >> 16) & 0xFF);
        data.push_back((val >> 8) & 0xFF);
        data.push_back(val & 0xFF);
    }
    
    // Flags byte (bit 0: busy, bit 1: paused)
    uint8_t flags = 0;
    if (isBusy()) flags |= 0x01;
    if (isPaused()) flags |= 0x02;
    data.push_back(flags);
    
    // Pattern name (first 4 bytes as identifier)
    const String& pattern = getCurrentMotionPatternName();
    for (uint32_t i = 0; i < 4; i++)
    {
        data.push_back(i < pattern.length() ? pattern[i] : 0);
    }
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
/// @brief Get last commanded position for all axes
/// @return Position in axes units (falls back to monitored position if needed)
AxesValues<AxisPosDataType> MotionController::getLastCommandedPos() const
{
    const AxesState& axesState = _blockManager.getAxesState();
    if (axesState.isValid())
        return axesState.getUnitsFromOrigin();

    // If commanded units are not valid fall back to monitored position
    AxesValues<AxisStepsDataType> actuatorPos;
    _rampGenerator.getTotalStepPosition(actuatorPos);
    AxesValues<AxisPosDataType> fallbackPos;
    _blockManager.actuatorToPt(actuatorPos, fallbackPos);
    return fallbackPos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get last monitored position (based on total steps recorded)
/// @return Position in axes units converted from actuator steps
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

AxesValues<AxisStepsDataType> MotionController::getAxisTotalSteps() const
{
    AxesValues<AxisStepsDataType> steps;
    _rampGenerator.getTotalStepPosition(steps);
    return steps;
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get end-stop state for an axis (min or max)
bool MotionController::getEndStopState(uint32_t axisIdx, bool max, bool& isFresh) const
{
    // Check axis index validity
    if (axisIdx >= _axisEndStops.size() || !_axisEndStops[axisIdx]) {
        isFresh = false;
#ifdef DEBUG_ENDSTOP_STATUS
        LOG_I(MODULE_PREFIX, "EndStop axis %u %s: INVALID axis", axisIdx, max ? "max" : "min");
#endif
        return false;
    }
    // Check if the end-stop is configured
    if (!_axisEndStops[axisIdx]->isValid(max)) {
        isFresh = false;
#ifdef DEBUG_ENDSTOP_STATUS
        LOG_I(MODULE_PREFIX, "EndStop axis %u %s: not configured", axisIdx, max ? "max" : "min");
#endif
        return false;
    }
    isFresh = true;
    bool triggered = _axisEndStops[axisIdx]->isAtEndStop(max);
#ifdef DEBUG_ENDSTOP_STATUS
    LOG_I(MODULE_PREFIX, "EndStop axis %u %s: isFresh=%d, triggered=%d", axisIdx, max ? "max" : "min", (int)isFresh, (int)triggered);
#endif
    return triggered;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Stop current motion pattern
void MotionController::stopPattern()
{
    _patternManager.stopPattern(true);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add a motion pattern to the registry
/// @param patternName Name of the pattern
/// @param createFn Factory function to create pattern instance
void MotionController::addMotionPattern(const String& patternName, MotionPatternCreateFn createFn)
{
    _patternManager.addPattern(patternName, createFn);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Start a motion pattern
/// @param patternName Name of pattern to start
/// @param patternRunTimeDefaultMs Default runtime in milliseconds (0 = run forever)
/// @param pParamsJson Optional JSON parameters for pattern
void MotionController::setMotionPattern(const String& patternName, uint32_t patternRunTimeDefaultMs, const char* pParamsJson)
{
    _patternManager.setPattern(*this, patternName, patternRunTimeDefaultMs, pParamsJson);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Check if motion pattern is currently active
/// @return true if pattern is running
bool MotionController::isMotionPatternActive() const
{
    return _patternManager.isPatternActive();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get name of current motion pattern
/// @return Current pattern name (empty if none)
const String& MotionController::getCurrentMotionPatternName() const
{
    return _patternManager.getCurrentPatternName();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set named value provider for patterns
/// @param pNamedValueProvider Pointer to named value provider
void MotionController::setPatternNamedValueProvider(NamedValueProvider* pNamedValueProvider)
{
    _patternManager.setNamedValueProvider(pNamedValueProvider);
}
