/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionController
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionController.h"
#include <ConfigPinMap.h>
#include <math.h>
#include <RaftUtils.h>
#include "AxisValues.h"
#include <StepDriverBase.h>
#include <StepDriverTMC2209.h>
#include "EndStops.h"
#include <RaftArduino.h>

#define DEBUG_STEPPER_SETUP_CONFIG
#define DEBUG_RAMP_SETUP_CONFIG
#define DEBUG_MOTION_CONTROLLER
#define INFO_LOG_AXES_PARAMS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Static
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const char* MODULE_PREFIX = "MotionController";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup serial bus and bus reversal
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupSerialBus(BusBase* pBus, bool useBusForDirectionReversal)
{
    // Setup bus
    for (StepDriverBase* pStepDriver : _stepperDrivers)
    {
        if (pStepDriver)
            pStepDriver->setupSerialBus(pBus, useBusForDirectionReversal);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionController::MotionController() : 
            _rampGenerator(_motionPipeline, _rampGenTimer),
            _blockManager(_motionPipeline, _motorEnabler, _axesParams)
{
}

MotionController::~MotionController()
{
    // Clear all steppers
    deinit();

    // Clear block manager
    _blockManager.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
// Setup the motor and pipeline parameters using a JSON input string
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setup(const ConfigBase& config, const char* pConfigPrefix)
{
    // Stop any motion
    _rampGenerator.stop();
    _motorEnabler.deinit();

    // Remove any config
    deinit();

    // Check if using ramp timer
    _rampTimerEn = config.getBool("ramp/rampTimerEn", 0, pConfigPrefix);

    // Setup axes (and associated hardware)
    setupAxes(config, pConfigPrefix);
#ifdef INFO_LOG_AXES_PARAMS
    _axesParams.debugLog();
#endif

    // Setup ramp generator and pipeline
    setupRampGenerator("ramp", config, pConfigPrefix);
    _rampGenerator.pause(false);
    _rampGenerator.enable(true);

    // Setup motor enabler
    setupMotorEnabler("motorEn", config, pConfigPrefix);

    // Setup motion control
    setupMotionControl("motion", config, pConfigPrefix);

#ifdef DEBUG_MOTION_CONTROL_TIMER
    // Debug
    _rampGenTimer.hookTimer(rampGenTimerCallback, this);
#endif

    // Start timer if required
    _rampGenTimer.enable(_rampTimerEn);

    // If no homing required then set the current position as home
    if (!_homingNeededBeforeAnyMove)
        setCurPositionAsHome(true);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Teardown
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::teardown()
{
    // Stop timer
    _rampGenTimer.enable(false);

    // Stop any motion
    _rampGenerator.stop();
    _motorEnabler.deinit();

    // Clear block manager
    _blockManager.clear();

    // Remove any config
    deinit();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Service
// Called regularly to allow the MotionController to do background work such as adding split-up blocks to the
// pipeline and checking if motors should be disabled after a period of no motion
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::service()
{
    // LOG_I(MODULE_PREFIX, "service stepper drivers %d", _stepperDrivers.size());

    // Service stepper drivers
    for (StepDriverBase* pStepDriver : _stepperDrivers)
    {
        // LOG_I(MODULE_PREFIX, "service stepper driver %d", pStepDriver->getSerialAddress());
        if (pStepDriver)
            pStepDriver->service();
    }

    // 
    if (Raft::isTimeout(millis(), _debugLastLoopMs, 1000))
    {
#ifdef DEBUG_MOTION_CONTROL_TIMER
        LOG_I(MODULE_PREFIX, "test count %d", _testRampGenCount);
#endif
        _debugLastLoopMs = millis();
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

    // Service motor enabler
    _motorEnabler.service();
    
    // Call process on motion actuator - only really used for testing as
    // motion is handled by ISR
    _rampGenerator.service();

    // // Process for trinamic devices
    // _trinamicsController.process();

    // Process any split-up blocks to be added to the pipeline
    _blockManager.pumpBlockSplitter();

    // // Service homing
    // _motionHoming.service(_axesParams);

    // Ensure motors enabled when homing or moving
    if ((_motionPipeline.count() > 0) 
        // TODO 2021
        //  || _motionHoming.isHomingInProgress()
         )
    {
        _motorEnabler.enableMotors(true, false);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check if busy
// Returns true if the motion controller is busy (i.e. moving)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionController::isBusy() const
{
    return _motionPipeline.count() > 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Handle MoveTo
// Command the robot to move (adding a command to the pipeline of motion)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionController::moveTo(const MotionArgs &args)
{
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

    // Handle linear motion (no ramp) - motion is defined in terms of steps (not mm)
    if (args.isLinear())
        return _blockManager.addLinearBlock(args);
    return moveToRamped(args);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pause (or un-pause) all motion
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::pause(bool pauseIt)
{
    _rampGenerator.pause(pauseIt);
    // _trinamicsController.pause(pauseIt);
    _isPaused = pauseIt;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Motion helper for ramped motion
// Ramped motion (variable acceleration) is used for normal motion
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionController::moveToRamped(const MotionArgs& args)
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
    if (_blockManager.isHomingNeededBeforeMove() && (!_blockManager.lastPosValid()))
    {
#ifdef DEBUG_MOTION_CONTROLLER
        LOG_I(MODULE_PREFIX, "moveTo lastPos invalid - need to home (initially and after linear moves)");
#endif
        return false;
    }

    // Get the target axis position
    AxesPosValues targetAxisPos = args.getAxesPositions();

    // Convert coords to real-world if required - this depends on the coordinate type
    // This doesn't convert coords - just checks for things like wrap around in circular coordinate systems
    _blockManager.preProcessCoords(targetAxisPos, _axesParams);

    // Fill in the targetAxisPos for axes for which values not specified
    // Handle relative motion calculation if required
    // Setup flags to indicate if each axis should be included in distance calculation
    bool includeAxisDistCalc[AXIS_VALUES_MAX_AXES];
    for (int i = 0; i < AXIS_VALUES_MAX_AXES; i++)
    {
        if (!targetAxisPos.isValid(i))
        {
            targetAxisPos.setVal(i, _blockManager.getLastPos().getVal(i));
#ifdef DEBUG_MOTION_CONTROLLER
            LOG_I(MODULE_PREFIX, "moveTo ax %d, pos %0.2f NoMovementOnThisAxis", 
                    i, 
                    targetAxisPos.getVal(i));
#endif
        }
        else
        {
            // Check relative motion - override current options if this command
            // explicitly states a moveType
            if (args.isRelative())
            {
                targetAxisPos.setVal(i, _blockManager.getLastPos().getVal(i) + targetAxisPos.getVal(i));
            }
#ifdef DEBUG_MOTION_CONTROLLER
            LOG_I(MODULE_PREFIX, "moveTo ax %d, pos %0.2f %s", 
                    i, 
                    targetAxisPos.getVal(i), 
                    args.isRelative() ? "RELATIVE" : "ABSOLUTE");
#endif
        }
        includeAxisDistCalc[i] = _axesParams.isPrimaryAxis(i);
    }

    // Get maximum length of block (for splitting up into blocks if required)
    double lineLen = targetAxisPos.distanceTo(_blockManager.getLastPos(), includeAxisDistCalc);

    // Ensure at least one block
    uint32_t numBlocks = 1;
    if (_blockDistance > 0.01f && !args.dontSplitMove())
        numBlocks = int(ceil(lineLen / _blockDistance));
    if (numBlocks == 0)
        numBlocks = 1;

    // Add to the block splitter
    _blockManager.addRampedBlock(args, targetAxisPos, numBlocks);

    // Pump the block splitter to prime the pipeline with blocks
    _blockManager.pumpBlockSplitter();

    // Ok
    return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set current position as home
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setCurPositionAsHome(bool allAxes, uint32_t axisIdx)
{
    if (!allAxes && (axisIdx >= AXIS_VALUES_MAX_AXES))
        return;
    for (uint32_t i = (allAxes ? 0 : axisIdx); i < (allAxes ? AXIS_VALUES_MAX_AXES : axisIdx+1); i++)
    {
        _rampGenerator.setTotalStepPosition(i, _axesParams.gethomeOffSteps(i));
        _blockManager.setCurPositionAsHome(i);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Go to home position
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::goHome(const MotionArgs &args)
{
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Data JSON
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String MotionController::getDataJSON(HWElemStatusLevel_t level)
{
    if (level >= ELEM_STATUS_LEVEL_MIN)
    {
        return _rampGenerator.getStats().getStatsStr();
    }
    return "{}";
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get queue slots (buffers) available for streaming
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t MotionController::streamGetQueueSlots()
{
    return _motionPipeline.remaining();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// De-init
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::deinit()
{
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
// Setup axes
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupAxes(const ConfigBase& config, const char* pConfigPrefix)
{
    // Setup stepper driver array
    _stepperDrivers.resize(AXIS_VALUES_MAX_AXES);
    for (auto& pDriver : _stepperDrivers)
        pDriver = nullptr;

    // Setup axes params
    _axesParams.setupAxes(config, pConfigPrefix);

    // Extract hardware related to axes
    std::vector<String> axesVec;
    if (config.getArrayElems("axes", axesVec, pConfigPrefix))
    {
        uint32_t axisIdx = 0;
        for (String& axisConfigStr : axesVec)
        {
            // Setup driver
            setupAxisHardware(axisIdx, axisConfigStr);

            // Next
            axisIdx++;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup axis hardware
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupAxisHardware(uint32_t axisIdx, const ConfigBase& config)
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

void MotionController::setupStepDriver(uint32_t axisIdx, const String& axisName, const char* jsonPrefix, const ConfigBase& config)
{
    // Stepper handler
    String hwLocation = config.getString("hw", DEFAULT_HARDWARE_LOCATION, jsonPrefix);
    String driverType = config.getString("driver", DEFAULT_DRIVER_CHIP, jsonPrefix);

    // Stepper parameters
    StepDriverParams stepperParams;

    // Get step controller settings
    stepperParams.microsteps = config.getLong("microsteps", StepDriverParams::MICROSTEPS_DEFAULT, jsonPrefix);
    stepperParams.writeOnly = config.getBool("writeOnly", 0, jsonPrefix);

    // Get hardware stepper params
    String stepPinName = config.getString("stepPin", "-1", jsonPrefix);
    stepperParams.stepPin = ConfigPinMap::getPinFromName(stepPinName.c_str());
    String dirnPinName = config.getString("dirnPin", "-1", jsonPrefix);
    stepperParams.dirnPin = ConfigPinMap::getPinFromName(dirnPinName.c_str());
    stepperParams.invDirn = config.getBool("invDirn", 0, jsonPrefix);
    stepperParams.extSenseOhms = config.getDouble("extSenseOhms", StepDriverParams::EXT_SENSE_OHMS_DEFAULT, jsonPrefix);
    stepperParams.extVRef = config.getBool("extVRef", false, jsonPrefix);
    stepperParams.extMStep = config.getBool("extMStep", false, jsonPrefix);
    stepperParams.intpol = config.getBool("intpol", false, jsonPrefix);
    stepperParams.minPulseWidthUs = config.getLong("minPulseWidthUs", 1, jsonPrefix);
    stepperParams.rmsAmps = config.getDouble("rmsAmps", StepDriverParams::RMS_AMPS_DEFAULT, jsonPrefix);
    stepperParams.holdDelay = config.getLong("holdDelay", StepDriverParams::IHOLD_DELAY_DEFAULT, jsonPrefix);
    stepperParams.pwmFreqKHz = config.getDouble("pwmFreqKHz", StepDriverParams::PWM_FREQ_KHZ_DEFAULT, jsonPrefix);
    stepperParams.address = config.getLong("addr", 0, jsonPrefix);

    // Hold mode
    String holdModeStr = config.getString("holdModeOrFactor", "1.0", jsonPrefix);
    if (holdModeStr.equalsIgnoreCase("freewheel"))
    {
        stepperParams.holdMode = StepDriverParams::HOLD_MODE_FREEWHEEL;
        stepperParams.holdFactor = 0;
    }
    else if (holdModeStr.equalsIgnoreCase("passive"))
    {
        stepperParams.holdMode = StepDriverParams::HOLD_MODE_PASSIVE_BREAKING;
        stepperParams.holdFactor = 0;
    }
    else
    {
        stepperParams.holdMode = StepDriverParams::HOLD_MODE_FACTOR;
        stepperParams.holdFactor = strtof(holdModeStr.c_str(), NULL);
    }

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
            pStepDriver->setup(axisName, stepperParams, _rampTimerEn);
        }
        // Debug
#ifdef DEBUG_STEPPER_SETUP_CONFIG
        LOG_I(MODULE_PREFIX, "setupStepDriver %s axisName %s address %02x driver %s stepPin %d(%s) dirnPin %d(%s) invDirn %s microsteps %d writeOnly %s extSenseOhms %.2f extVRef %s extMStep %s intpol %s rmsAmps %0.2f holdMode %d holdFactor %0.2f holdDelay %d pwmFreqKHz %0.2f",
                pStepDriver ? "local" : "FAILED",
                axisName.c_str(), 
                stepperParams.address,
                driverType.c_str(),
                stepperParams.stepPin, stepPinName.c_str(), 
                stepperParams.dirnPin, dirnPinName.c_str(), 
                stepperParams.invDirn ? "Y" : "N",
                stepperParams.microsteps,
                stepperParams.writeOnly ? "Y" : "N",
                stepperParams.extSenseOhms,
                stepperParams.extVRef ? "Y" : "N",
                stepperParams.extMStep ? "Y" : "N",
                stepperParams.intpol ? "Y" : "N",
                stepperParams.rmsAmps,
                uint32_t(stepperParams.holdMode),
                stepperParams.holdFactor,
                stepperParams.holdDelay,
                stepperParams.pwmFreqKHz);
    #endif
    }

    // Add driver
    if (axisIdx < _stepperDrivers.size())
        _stepperDrivers[axisIdx] = pStepDriver;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup end stops
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupEndStops(uint32_t axisIdx, const String& axisName, const char* jsonElem, const ConfigBase& mainConfig)
{
    // Endstops
    EndStops* pEndStops = new EndStops();

    // Config
    std::vector<String> endstopVec;
    if (mainConfig.getArrayElems(jsonElem, endstopVec))
    {
        for (String& endstopConfigStr : endstopVec)
        {
            // Debug
            // LOG_I(MODULE_PREFIX, "setupEndStops %s", endstopConfigStr.c_str());

            // Setup endstop
            ConfigBase config = endstopConfigStr;
            bool isMax = config.getBool("isMax", false);
            String name = config.getString("name", "");
            String endstopPinName = config.getString("sensePin", "-1");
            int pin = ConfigPinMap::getPinFromName(endstopPinName.c_str());
            bool activeLevel = config.getBool("actLvl", false);
            String inputTypeStr = config.getString("inputType", "INPUT_PULLUP");
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
// Setup ramp generator
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupRampGenerator(const char* jsonElem, const ConfigBase& mainConfig, const char* pConfigPrefix)
{
    // TODO refactor to use JSON paths

    // Configure the driver
    ConfigBase rampGenConfig = mainConfig.getString(jsonElem, "{}", pConfigPrefix);

    // Ramp generator config
    long rampTimerUs = rampGenConfig.getLong("rampTimerUs", RampGenTimer::RAMP_GEN_PERIOD_US_DEFAULT);

    // Ramp generator timer
    bool timerSetupOk = _rampGenTimer.setup(rampTimerUs);

    // Ramp generator
    _rampGenerator.setup(_rampTimerEn, _stepperDrivers, _axisEndStops);

    // Pipeline
    uint32_t pipelineLen = rampGenConfig.getLong("pipelineLen", pipelineLen_default);
    _motionPipeline.setup(pipelineLen);

#ifdef DEBUG_RAMP_SETUP_CONFIG
    LOG_I(MODULE_PREFIX, "setupRampGenerator timerEn %d timerUs %ld timertimerSetupOk %d pipelineLen %d",
        _rampTimerEn, rampTimerUs, timerSetupOk, pipelineLen);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup motor enabler
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupMotorEnabler(const char* jsonElem, const ConfigBase& config, const char* pConfigPrefix)
{
    // Path string
    String pathStr = String(pConfigPrefix) + "/" + String(jsonElem);

    // Setup
    _motorEnabler.setup(config, pathStr.c_str());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup motion control
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupMotionControl(const char* jsonElem, const ConfigBase& mainConfig, const char* pConfigPrefix)
{
    // TODO refactor to use JSON paths

    // Configuration
    ConfigBase motionConfig = mainConfig.getString(jsonElem, "{}", pConfigPrefix);

    // Params
    String geometry = motionConfig.getString("geom", "XYZ");
    _blockDistance = motionConfig.getDouble("blockDist", _blockDistance_default);
    bool allowAllOutOfBounds = motionConfig.getLong("allowOutOfBounds", false) != 0;
    double junctionDeviation = motionConfig.getDouble("junctionDeviation", junctionDeviation_default);
    _homingNeededBeforeAnyMove = motionConfig.getLong("homeBeforeMove", true) != 0;

    // Debug
    LOG_I(MODULE_PREFIX, "setupMotion geom %s blockDist %0.2f (0=no-max) allowOoB %s homeBefMove %s jnDev %0.2f",
               geometry.c_str(), _blockDistance, allowAllOutOfBounds ? "Y" : "N", 
               _homingNeededBeforeAnyMove ? "Y" : "N",
               junctionDeviation);

    // Block manager
    _blockManager.setup(geometry, allowAllOutOfBounds, junctionDeviation, 
            _homingNeededBeforeAnyMove, _rampGenTimer.getPeriodUs());
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
// Get last commanded position
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AxesPosValues MotionController::getLastCommandedPos() const
{
    return _blockManager.getLastPos();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get last monitored position
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AxesPosValues MotionController::getLastMonitoredPos() const
{
    // Get current position
    AxesParamVals<AxisStepsDataType> curActuatorPos;
    _rampGenerator.getTotalStepPosition(curActuatorPos);
    // Use reverse kinematics to get location
    AxesPosValues lastMonitoredPos;
    _blockManager.coordsActuatorToRealWorld(curActuatorPos, lastMonitoredPos);
    return lastMonitoredPos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get debug string
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String MotionController::getDebugStr() const
{
    String str;
    str += _rampGenTimer.getDebugStr() + ", ";
    str += getLastMonitoredPos().getDebugStr();
    return str;
}
