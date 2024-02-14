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
#include "AxisValues.h"
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
// Static
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const char* MODULE_PREFIX = "MotionController";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionController::MotionController() : 
            _blockManager(_motorEnabler, _axesParams)
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

void MotionController::setup(const RaftJsonIF& config)
{
    // Teardown first
    teardown();

    // Setup axes (and associated hardware)
    setupAxes(config);
#ifdef INFO_LOG_AXES_PARAMS
    _axesParams.debugLog();
#endif

    // Setup ramp generator and pipeline
    RaftJsonPrefixed rampConfig(config, "ramp");
    _rampGenerator.setup(config, _stepperDrivers, _axisEndStops);
    _rampGenerator.start();

    // Setup motor enabler
    RaftJsonPrefixed motorEnConfig(config, "motorEn");
    _motorEnabler.setup(motorEnConfig);

    // Setup motion control
    RaftJsonPrefixed motionConfig(config, "motion");
    setupMotionControl(motionConfig);

    // If no homing required then set the current position as home
    if (!_homingNeededBeforeAnyMove)
        setCurPositionAsHome(true);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Teardown
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::teardown()
{
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
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline());

    // // Service homing
    // _motionHoming.service(_axesParams);

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
// Check if busy
// Returns true if the motion controller is busy (i.e. moving)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionController::isBusy() const
{
    return _rampGenerator.getMotionPipelineConst().count() > 0;
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
        return _blockManager.addLinearBlock(args, _rampGenerator.getMotionPipeline());
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
    _blockManager.pumpBlockSplitter(_rampGenerator.getMotionPipeline());

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

uint32_t MotionController::streamGetQueueSlots() const
{
    return _rampGenerator.getMotionPipelineConst().remaining();
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
    StepDriverParams stepperParams;

    // Get step controller settings
    stepperParams.microsteps = config.getLong("microsteps", StepDriverParams::MICROSTEPS_DEFAULT);
    stepperParams.writeOnly = config.getBool("writeOnly", 0);

    // Get hardware stepper params
    String stepPinName = config.getString("stepPin", "-1");
    stepperParams.stepPin = ConfigPinMap::getPinFromName(stepPinName.c_str());
    String dirnPinName = config.getString("dirnPin", "-1");
    stepperParams.dirnPin = ConfigPinMap::getPinFromName(dirnPinName.c_str());
    stepperParams.invDirn = config.getBool("invDirn", 0);
    stepperParams.extSenseOhms = config.getDouble("extSenseOhms", StepDriverParams::EXT_SENSE_OHMS_DEFAULT);
    stepperParams.extVRef = config.getBool("extVRef", false);
    stepperParams.extMStep = config.getBool("extMStep", false);
    stepperParams.intpol = config.getBool("intpol", false);
    stepperParams.minPulseWidthUs = config.getLong("minPulseWidthUs", 1);
    stepperParams.rmsAmps = config.getDouble("rmsAmps", StepDriverParams::RMS_AMPS_DEFAULT);
    stepperParams.holdDelay = config.getLong("holdDelay", StepDriverParams::IHOLD_DELAY_DEFAULT);
    stepperParams.pwmFreqKHz = config.getDouble("pwmFreqKHz", StepDriverParams::PWM_FREQ_KHZ_DEFAULT);
    stepperParams.address = config.getLong("addr", 0);

    // Hold mode
    String holdModeStr = config.getString("holdModeOrFactor", "1.0");
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
            pStepDriver->setup(axisName, stepperParams, _rampGenerator.isUsingTimerISR());
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
// Setup motion control
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionController::setupMotionControl(const RaftJsonIF& config)
{
    // Params
    String geometry = config.getString("geom", "XYZ");
    _blockDistance = config.getDouble("blockDist", _blockDistance_default);
    bool allowAllOutOfBounds = config.getLong("allowOutOfBounds", false) != 0;
    double junctionDeviation = config.getDouble("junctionDeviation", junctionDeviation_default);
    _homingNeededBeforeAnyMove = config.getLong("homeBeforeMove", true) != 0;

    // Debug
    LOG_I(MODULE_PREFIX, "setupMotion geom %s blockDist %0.2f (0=no-max) allowOoB %s homeBefMove %s jnDev %0.2f",
               geometry.c_str(), _blockDistance, allowAllOutOfBounds ? "Y" : "N", 
               _homingNeededBeforeAnyMove ? "Y" : "N",
               junctionDeviation);

    // Block manager
    _blockManager.setup(geometry, allowAllOutOfBounds, junctionDeviation, 
            _homingNeededBeforeAnyMove, _rampGenerator.getPeriodUs());
}

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
    str += _rampGenerator.getDebugStr() + ", ";
    str += getLastMonitoredPos().getDebugStr();
    return str;
}
