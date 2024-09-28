/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverTMC2209
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "StepDriverTMC2209.h"
#include "RaftArduino.h"
#include <math.h>

#define WARN_ON_DRIVER_BUSY

// #define DEBUG_IHOLD_IRUN_CALCS
// #define DEBUG_REGISTER_WRITE_PROCESS
// #define DEBUG_IHOLD_IRUN
// #define DEBUG_REGISTER_READ_PROCESS
// #define DEBUG_REGISTER_READ_START
// #define DEBUG_REGISTER_READ_IN_PROGRESS
// #define DEBUG_STEPPING_ONLY_IF_NOT_ISR
// #define DEBUG_DIRECTION_ONLY_IF_NOT_ISR

// PWM frequency calculations
// #define DEBUG_PWM_FREQ_CALCS_DETAIL

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
StepDriverTMC2209::StepDriverTMC2209()
{
    // Sync byte
    _tmcSyncByte = TMC_2209_SYNC_BYTE;

    // Add TMC registers
    // **** Do not rearrange this without changing the order of ***
    // DriverRegisterCodes enumeration as that is used for indexing

    // Add GCONF register
    _driverRegisters.push_back({"GCONF", 0, 0x000001C0, 0x000003ff, true});
    // Add GSTAT register
    _driverRegisters.push_back({"GSTAT", 1, 0x00000000, 0x00000007, false});
    // Add CHOPCONF register
    _driverRegisters.push_back({"CHOPCONF", 0x6c, 0x10000053, 0xff0387f, true});
    // Add IHOLD_IRUN register
    _driverRegisters.push_back({"IHOLD_RUN", 0x10, 0x00001f00, 0x000f1f1f, true});
    // Add PWMCONF register
    _driverRegisters.push_back({"PWMCONF", 0x70, 0xC10D0024, 0xc001f0ff, true});
    // Add DRV_STATUS register
    _driverRegisters.push_back({"DRV_STATUS", 0x6F, 0x00000000, 0xff3fffff, false});

    // Vars
    _dirnCurValue = false;
    _stepCurActive = false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup
/// @param stepperName - name of stepper
/// @param stepperParams - parameters for the stepper
/// @param usingISR - true if using ISR
/// @return true if successful
bool StepDriverTMC2209::setup(const String& stepperName, const StepDriverParams& stepperParams, bool usingISR)
{
    // Debug
    LOG_I(MODULE_PREFIX, "setup %s", stepperName.c_str());

    // Configure base
    StepDriverBase::setup(stepperName, stepperParams, usingISR);
    _singleWireReadWrite = true;

    // Check if UART is used (otherwise assume that configuration is done in hardware)
    if (stepperParams.noUART)
    {
        LOG_I(MODULE_PREFIX, "setup NO UART so hardware configuration only");
    }
    else
    {
        // Setup read timing for status registers
        if (stepperParams.statusIntvMs > 0)
        {
            _statusReadIntervalMs = stepperParams.statusIntvMs;
        }

        // Set main registers
        setMainRegs();
    }

    // Setup step pin
    if (stepperParams.stepPin >= 0)
    {
        // Setup the pin
        pinMode(stepperParams.stepPin, OUTPUT);
        digitalWrite(stepperParams.stepPin, false);
    }

    // Setup dirn pin
    if (stepperParams.dirnPin >= 0)
    {
        pinMode(stepperParams.dirnPin, OUTPUT);
    }
    
    // Hardware is not initialised
    _hwIsSetup = true;

    // Set initial direction arbitrarily
    setDirection(false, true);

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Loop - called frequently
void StepDriverTMC2209::loop()
{
    // Loop base
    StepDriverBase::loop();

    // Check if driver is ready
    if (isBusy())
    {
#ifdef WARN_ON_DRIVER_BUSY
        if (!_warnOnDriverBusyDone)
        {
            if (_warnOnDriverBusyStartTimeMs == 0)
            {
                _warnOnDriverBusyStartTimeMs = millis();
            }
            else if (Raft::isTimeout(millis(), _warnOnDriverBusyStartTimeMs, WARN_ON_DRIVER_BUSY_AFTER_MS))
            {
                LOG_E(MODULE_PREFIX, "%s loop driver busy for too long", _name.c_str());
                _warnOnDriverBusyStartTimeMs = 0;
                _warnOnDriverBusyDone = true;
            }
        }
#endif
        return;
    }
    else
    {
        _warnOnDriverBusyStartTimeMs = 0;
        _warnOnDriverBusyDone = false;
    }

    // Check if ready for loop checks
    if (!Raft::isTimeout(millis(), _loopLastTimeMs, LOOP_INTERVAL_MS))
        return;
    _loopLastTimeMs = millis();

    // Check read in progress
    if (isReadInProgress())
    {
#ifdef DEBUG_REGISTER_READ_IN_PROGRESS
        LOG_I(MODULE_PREFIX, "%s loop readinprogress", _name.c_str());
#endif
        return;
    }


    // Check for write pending
    int wrPendRegIdx = writePendingRegIdx();
    if (wrPendRegIdx >= 0)
    {
#ifdef DEBUG_REGISTER_WRITE_PROCESS
        LOG_I(MODULE_PREFIX, "%s loop start write regCode %s(%d) val %08x", 
                _name.c_str(), _driverRegisters[wrPendRegIdx].regName.c_str(), 
                _driverRegisters[wrPendRegIdx].regAddr, 
                _driverRegisters[wrPendRegIdx].regWriteVal);
#endif
        writeTrinamicsRegister(_driverRegisters[wrPendRegIdx].regName.c_str(), 
                                _driverRegisters[wrPendRegIdx].regAddr, 
                                _driverRegisters[wrPendRegIdx].regWriteVal);
        if (_requestedParams.writeOnly)
        {        
            _driverRegisters[wrPendRegIdx].regValCur = _driverRegisters[wrPendRegIdx].regWriteVal;
        }
        _driverRegisters[wrPendRegIdx].writePending = false;
        return;
    }

    // Check for read pending
    int rdPendRegIdx = readPendingRegIdx();
    if (rdPendRegIdx >= 0)
    {
        // Start reading register
#ifdef DEBUG_REGISTER_READ_START
        LOG_I(MODULE_PREFIX, "%s loop start read regCode %d", _name.c_str(), rdPendRegIdx);
#endif
        startReadTrinamicsRegister(rdPendRegIdx);
        _driverRegisters[rdPendRegIdx].readPending = false;
        return;
    }

    // Check for time to read status registers
    if ((_statusReadIntervalMs != 0) && Raft::isTimeout(millis(), _statusReadLastTimeMs, _statusReadIntervalMs))
    {
        // Check if GSTAT register has been read successfully
        if (_driverRegisters[DRIVER_REGISTER_CODE_GSTAT].readValid)
        {
            // Check if driver has been reset
            if (_driverRegisters[DRIVER_REGISTER_CODE_GSTAT].regValCur & TMC_2209_GSTAT_RESET_MASK)
            {
                LOG_E(MODULE_PREFIX, "%s loop driver reconfig required", _name.c_str());
                _configResetRequired = true;
            }
        }
        // Request read
        _driverRegisters[DRIVER_REGISTER_CODE_DRV_STATUS].readPending = true;
        _driverRegisters[DRIVER_REGISTER_CODE_GSTAT].readPending = true;
        _statusReadLastTimeMs = millis();
    }

    // Check for config reset
    if (_configResetRequired)
    {
        if (Raft::isTimeout(millis(), _configSetLastTimeMs, CONFIG_RESET_AFTER_MS))
        {
            // Reset config
            LOG_I(MODULE_PREFIX, "%s loop reset config registers", _name.c_str());
            setMainRegs();
            _configResetRequired = false;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set motor microsteps
/// @param microsteps - number of microsteps
void StepDriverTMC2209::setMicrosteps(uint32_t microsteps)
{
    // Set the max motor current
    _requestedParams.microsteps = microsteps;

    // Set mres value into CHOPCONF register
    uint32_t mres = getMRESFieldValue(microsteps);

    // Set the mres value in the write register
    _driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].writePending = true;
    _driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].regWriteVal = 
                (_driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].regWriteVal & ~TMC_2209_CHOPCONF_MRES_MASK) |
                (mres << TMC_2209_CHOPCONF_MRES_BIT);
    _driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].readPending = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get MRES value from microsteps
/// @param microsteps - number of microsteps
/// @return MRES value
uint32_t StepDriverTMC2209::getMRESFieldValue(uint32_t microsteps) const
{
    // Get MRES value
    uint8_t mres = TMC_2209_CHOPCONF_MRES_DEFAULT;
    switch(microsteps) 
    {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case  64: mres = 2; break;
        case  32: mres = 3; break;
        case  16: mres = 4; break;
        case   8: mres = 5; break;
        case   4: mres = 6; break;
        case   2: mres = 7; break;
        case   0: mres = 8; break;
        default: mres = TMC_2209_CHOPCONF_MRES_DEFAULT; break;
    }
    return mres;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Convert RMS current setting to register values
/// @param reqCurrentAmps - required current in Amps
/// @param holdFactor - hold factor
/// @param holdMode - hold mode
/// @param vsenseOut - vsense value
/// @param irunOut - irun value
/// @param iholdOut - ihold value
void StepDriverTMC2209::convertRMSCurrentToRegs(double reqCurrentAmps, double holdFactor, 
            StepDriverParams::HoldModeEnum holdMode, bool& vsenseOut, uint32_t& irunOut, uint32_t& iholdOut) const
{
    // Calculate IRUN
    double irunDouble = _maxSenseVoltage / _vfsValue;
    irunOut = uint32_t(ceil(irunDouble * 32)-1);
    if (irunOut < 8)
        irunOut = 8;
    else if (irunOut > 31)
        irunOut = 31;

    // IHOLD value
    iholdOut = 0;
    if (holdMode == StepDriverParams::HOLD_MODE_FACTOR)
    {
        uint32_t iholdVal = irunOut * holdFactor;
        if (iholdVal < 1)
            iholdOut = 1;
        else if (iholdVal > 31)
            iholdOut = 31;
        else
            iholdOut = iholdVal;
    }

#ifdef DEBUG_IHOLD_IRUN_CALCS
    LOG_I(MODULE_PREFIX, "convertRMSCurrentToRegs %s reqCurAmps %0.2f holdMode %d holdFactor %0.2f vsenseOut %d irunOut %d iholdOut %d maxSenseVoltage %.2f Vfs %.2f",
            _name.c_str(), reqCurrentAmps, holdMode, holdFactor, vsenseOut, irunOut, iholdOut, _maxSenseVoltage, _vfsValue);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set direction
/// @param dirn - direction
/// @param forceSet - force set
void IRAM_ATTR StepDriverTMC2209::setDirection(bool dirn, bool forceSet)
{
    // Check valid
    if (!_hwIsSetup)
        return;

    // Check for direction changed (or forced)
    if ((dirn != _dirnCurValue) || forceSet)
    {
        // Check if direction reversal is done via the serial bus
        bool hwDirn = dirn;
        if (_requestedParams.invDirn)
            hwDirn = !dirn;
#ifdef DEBUG_DIRECTION_ONLY_IF_NOT_ISR
        if (!_usingISR)
        {
            LOG_I(MODULE_PREFIX, "setDirection %s CHANGE pin %d logicalDirection %d hwDirn %d (use %s inv %s) forceSet %s", 
                    _name.c_str(), 
                    _requestedParams.dirnPin, 
                    dirn, 
                    hwDirn,
                    _useBusForDirectionReversal ? "bus" : "hw",
                    _requestedParams.invDirn ? "Y" : "N",
                    forceSet ? "Y" : "N");
        }
#endif        
        // Set the pin value if valid and direction is not done via serial bus
        if ((_requestedParams.dirnPin >= 0) && !_useBusForDirectionReversal)
        {
            digitalWrite(_requestedParams.dirnPin, hwDirn);
        }
    }
    else
    {
#ifdef DEBUG_DIRECTION_ONLY_IF_NOT_ISR
         if (!_usingISR)
        {
            LOG_I(MODULE_PREFIX, "setDirection %s NO CHANGE pin %d logicalDirection %d forceSet %s", 
                    _name.c_str(), 
                    _requestedParams.dirnPin, 
                    dirn, 
                    forceSet ? "Y" : "N");
        }
#endif        
    }

    _dirnCurValue = dirn;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Start a step
void IRAM_ATTR StepDriverTMC2209::stepStart()
{
    // Check hardware pin
    if (_hwIsSetup && (_requestedParams.stepPin >= 0))
    {
#ifdef DEBUG_STEPPING_ONLY_IF_NOT_ISR
        if (!_usingISR)
        {
            LOG_I(MODULE_PREFIX, "stepStart %s pin %d", _name.c_str(), _requestedParams.stepPin);
        }
#endif
        // Set the pin value
        digitalWrite(_requestedParams.stepPin, true);
        _stepCurActive = true;
    }
    else
    {
#ifdef DEBUG_STEPPING_ONLY_IF_NOT_ISR
        if (!_usingISR)
        {
            LOG_W(MODULE_PREFIX, "stepStart FAILED %s pin %d hwIsSetup %d", 
                    _name.c_str(), _requestedParams.stepPin, _hwIsSetup);
        }
#endif
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief End a step
bool IRAM_ATTR StepDriverTMC2209::stepEnd()
{
    if (_stepCurActive && (_requestedParams.stepPin >= 0))
    {
        _stepCurActive = false;
        digitalWrite(_requestedParams.stepPin, false);
#ifdef DEBUG_STEPPING_ONLY_IF_NOT_ISR
        if (!_usingISR)
        {
            LOG_I(MODULE_PREFIX, "stepEnd %s pin %d", _name.c_str(), _requestedParams.stepPin);
        }
#endif
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set the max motor current
/// @param maxMotorCurrentAmps - max motor current in Amps
void StepDriverTMC2209::setMaxMotorCurrentAmps(float maxMotorCurrentAmps)
{
    // Set the max motor current
    _requestedParams.rmsAmps = maxMotorCurrentAmps;

    // setMaxMotorCurrentAmps
    LOG_I(MODULE_PREFIX, "setMaxMotorCurrentAmps %s %0.2fA", _name.c_str(), maxMotorCurrentAmps);

    // Set the current
    setMainRegs();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Set the main registers with stored values
void StepDriverTMC2209::setMainRegs()
{
    // Set time of config setting
    _configSetLastTimeMs = millis();

    // Calculate Vfs and VMaxSense
    calculateVfsAndMaxSenseV(_requestedParams.rmsAmps, _vfsValue, _maxSenseVoltage);
    
    // Get CHOPCONF vsense value and IRUN, IHOLD values from required RMS current
    bool vsenseValue = false;
    uint32_t irunValue = 0;
    uint32_t iholdValue = 0;
    convertRMSCurrentToRegs(_requestedParams.rmsAmps, _requestedParams.holdFactor, _requestedParams.holdMode, 
                    vsenseValue, irunValue, iholdValue);

    // Clear the reset flag in the GSTAT register
    _driverRegisters[DRIVER_REGISTER_CODE_GSTAT].regWriteVal = TMC_2209_GSTAT_RESET_MASK;
    _driverRegisters[DRIVER_REGISTER_CODE_GSTAT].writePending = true;

    // Init the GCONF register
    _driverRegisters[DRIVER_REGISTER_CODE_GCONF].writePending = true;
    _driverRegisters[DRIVER_REGISTER_CODE_GCONF].regWriteVal =
                (1 << TMC_2209_GCONF_MULTISTEP_FILT_BIT) |
                (1 << TMC_2209_GCONF_PDN_UART_BIT) |
                (_requestedParams.invDirn ? (1 << TMC_2209_GCONF_INV_DIRN_BIT) : 0) |
                ((_requestedParams.extSenseOhms < 0.01) ? (1 << TMC_2209_GCONF_EXT_SENSE_RES_BIT) : 0) |
                (_requestedParams.extVRef ? (1 << TMC_2209_GCONF_EXT_VREF_BIT) : 0) |
                (_requestedParams.extMStep ? 0 : (1 << TMC_2209_GCONF_MSTEP_REG_SELECT_BIT));

    // Init the CHOPCONF register
    _driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].writePending = true;
    _driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].regWriteVal =
                (getMRESFieldValue(_requestedParams.microsteps) << TMC_2209_CHOPCONF_MRES_BIT) |
                (StepDriverParams::TOFF_VALUE_DEFAULT << TMC_2209_CHOPCONF_TOFF_BIT) |
                (_requestedParams.intpol ? (1 << TMC_2209_CHOPCONF_INTPOL_BIT) : 0) |
                (vsenseValue ? (1 << TMC_2209_CHOPCONF_VSENSE_BIT) : 0);

    // Init the IHOLD_IRUN register
    _driverRegisters[DRIVER_REGISTER_CODE_IHOLD_IRUN].writePending = true;
    _driverRegisters[DRIVER_REGISTER_CODE_IHOLD_IRUN].regWriteVal = 
                (irunValue << TMC_2209_IRUN_BIT) |
                (iholdValue << TMC_2209_IHOLD_BIT) |
                (_requestedParams.holdDelay << TMC_2209_IHOLD_DELAY_BIT);
#ifdef DEBUG_IHOLD_IRUN
    LOG_I(MODULE_PREFIX, "setMainRegs %s irunValue %d iholdValue %d, reg %s(0x%02x), val %08x", 
                    _name.c_str(),
                    irunValue, iholdValue, 
                    _driverRegisters[DRIVER_REGISTER_CODE_IHOLD_IRUN].regName,
                    DRIVER_REGISTER_CODE_IHOLD_IRUN,
                    _driverRegisters[DRIVER_REGISTER_CODE_IHOLD_IRUN].regWriteVal);
#endif

    // Calculate PWMCONF_PWM_FREQ
    double clockDiv = _requestedParams.pwmFreqKHz * 1000.0 / TMC_2209_CLOCK_FREQ_HZ;
    uint32_t clockFactors[] = {1024,683,512,410};
    uint32_t pwmClockVal = 0;
    for (uint32_t i = 1; i < sizeof(clockFactors)/sizeof(clockFactors[0]); i++)
    {
        double midVal = (clockFactors[i-1] + clockFactors[i]) / 2;
#ifdef DEBUG_PWM_FREQ_CALCS
        LOG_I(MODULE_PREFIX, "setMainRegs %s pwmFreq %0.2f clockDiv %f pwmClockVal %d midVal %f 2/midVal %f", 
                _name.c_str(),
                _requestedParams.pwmFreqKHz, clockDiv, pwmClockVal, midVal, 2/midVal);
#endif
        if (clockDiv > (2 / midVal))
            pwmClockVal++;
    }

    // Init the PWMCONF register
    _driverRegisters[DRIVER_REGISTER_CODE_PWMCONF].writePending = true;
    _driverRegisters[DRIVER_REGISTER_CODE_PWMCONF].regWriteVal = 
                (12 << TMC_2209_PWMCONF_PWM_LIM_BIT) |
                (1 << TMC_2209_PWMCONF_PWM_REG_BIT) |
                ((_requestedParams.holdMode == StepDriverParams::HOLD_MODE_FREEWHEEL) ? 1 :
                    (_requestedParams.holdMode == StepDriverParams::HOLD_MODE_PASSIVE_BREAKING) ? 2 : 0) << TMC_2209_PWMCONF_FREEWHEEL_BIT |
                (1 << TMC_2209_PWMCONF_AUTOGRAD_BIT) |
                (1 << TMC_2209_PWMCONF_AUTOSCALE_BIT) |
                (pwmClockVal << TMC_2209_PWMCONF_PWM_FREQ_BIT) |
                (TMC_2209_PWMCONF_PWM_GRAD << TMC_2209_PWMCONF_PWM_GRAD_BIT) |
                (TMC_2209_PWMCONF_PWM_OFS << TMC_2209_PWMCONF_PWM_OFS_BIT);

    // Set flags to indicate that registers should be read back to confirm
    _driverRegisters[DRIVER_REGISTER_CODE_GCONF].readPending = true;
    _driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].readPending = true;
    // Note that IHOLD_IRUN is not read back as it is read only
    _driverRegisters[DRIVER_REGISTER_CODE_PWMCONF].readPending = true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get debug JSON
/// @param includeBraces - include braces
/// @param detailed - detailed
/// @return JSON string
virtual String getDebugJSON(bool includeBraces, bool detailed) const
{
    return getStatusJSON(includeBraces, detailed
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// @brief Get status JSON
// @param includeBraces - include braces
// @param detailed - detailed
// @return JSON string
String StepDriverTMC2209::getStatusJSON(bool includeBraces, bool detailed) const
{
    bool anyWritePending = false;
    bool anyReadPending = false;
    for (uint32_t i = 0; i < _driverRegisters.size(); i++)
    {
        anyWritePending |= _driverRegisters[i].writePending;
        anyReadPending |= _driverRegisters[i].readPending;
    }
    String retStr;
    if (includeBraces)
        retStr = "{";
    retStr += "\"n\":\"" + _name + "\",";
    retStr += "\"t\":\"TMC2209\",";
    retStr += "\"uSt\":" + String(getMicrosteps()) + ",";
    retStr += "\"rms\":" + String(getRMSAmps(), 2) + ",";
    retStr += "\"gStat\":" + getGSTATJson() + ",";
    retStr += "\"drvSt\":" + getDriverStatusJson();
    if (detailed)
    {
        retStr += ",\"hldF\":" + String(_requestedParams.holdFactor, 2) + ",";
        retStr += "\"hldM\":" + String(_requestedParams.holdMode) + ",";
        retStr += "\"dly\":" + String(_requestedParams.holdDelay) + ",";
        retStr += "\"inv\":" + String(_requestedParams.invDirn) + ",";
        retStr += "\"int\":" + String(_requestedParams.intpol) + ",";
        retStr += "\"ohms\":" + String(_requestedParams.extSenseOhms, 2) + ",";
        retStr += "\"xVRf\":" + String(_requestedParams.extVRef) + ",";
        retStr += "\"xuSt\":" + String(_requestedParams.extMStep) + ",";
        retStr += "\"sPin\":" + String(_requestedParams.stepPin) + ",";
        retStr += "\"dPin\":" + String(_requestedParams.dirnPin) + ",";
        retStr += "\"wrPnd\":" + String(anyWritePending) + ",";
        retStr += "\"rdPnd\":" + String(anyReadPending) + ",";
        retStr += "\"GS\":\"" + getRegValHex(DRIVER_REGISTER_CODE_GSTAT) + "\",";
        retStr += "\"DV\":\"" + getRegValHex(DRIVER_REGISTER_CODE_DRV_STATUS) + "\",";
        retStr += "\"GC\":\"" + getRegValHex(DRIVER_REGISTER_CODE_GCONF) + "\",";
        retStr += "\"CH\":\"" + getRegValHex(DRIVER_REGISTER_CODE_CHOPCONF) + "\",";
        retStr += "\"IH\":\"" + getRegValHex(DRIVER_REGISTER_CODE_IHOLD_IRUN) + "\",";
        retStr += "\"PW\":\"" + getRegValHex(DRIVER_REGISTER_CODE_PWMCONF) + "\",";
        retStr += "\"wr\":\"" + getWriteResultStr(_lastWriteResultOk) + "\",";
        retStr += "\"rd\":\"" + getReadResultStr(_lastReadResult) + "\"";
    }
    if (includeBraces)
        retStr += "}";
    return retStr;
}