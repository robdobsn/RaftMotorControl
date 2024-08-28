/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverBase
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "StepDriverBase.h"

class StepDriverTMC2209 : public StepDriverBase
{
public:
    StepDriverTMC2209();

    // Setup
    virtual bool setup(const String& stepperName, const StepDriverParams& stepperParams, bool usingISR) override final;
    
    // Loop - called frequently
    virtual void loop() override final;

    // Microsteps
    virtual void setMicrosteps(uint32_t microsteps) override final;

    // Set direction
    virtual void setDirection(bool dirn, bool forceSet = false) override final;

    // Start and end a single step
    virtual void stepStart() override final;
    virtual bool stepEnd() override final;

    virtual String getDriverType() const override final
    {
        return "TMC2209";
    }

    virtual String getStatusJSON(bool includeBraces, bool detailed) const override final;

    virtual void setMaxMotorCurrentAmps(float maxMotorCurrentAmps) override final;

    virtual bool isOperatingOk() const override final
    {
        return busValid() && _driverRegisters[DRIVER_REGISTER_CODE_GSTAT].readValid;
    }

private:
    // Driver register codes
    // These must be in the same order as added to _driverRegisters
    // in the base class
    enum DriverRegisterCodes {
        DRIVER_REGISTER_CODE_GCONF,
        DRIVER_REGISTER_CODE_GSTAT,
        DRIVER_REGISTER_CODE_CHOPCONF,
        DRIVER_REGISTER_CODE_IHOLD_IRUN,
        DRIVER_REGISTER_CODE_PWMCONF,
        DRIVER_REGISTER_CODE_DRV_STATUS,
    };

    // Current direction and step values
    bool _dirnCurValue;
    bool _stepCurActive;

    // Loop timer
    static const uint32_t LOOP_INTERVAL_MS = 100;
    uint32_t _loopLastTimeMs = 0;

    // Interval for reading driver status and config
    uint32_t _statusReadIntervalMs = 0;
    uint32_t _statusReadLastTimeMs = 0;

    // Config reset
    static const uint32_t CONFIG_RESET_AFTER_MS = 1000;
    uint32_t _configSetLastTimeMs = 0;
    bool _configResetRequired = false;

    // Driver busy warnings
    uint32_t _warnOnDriverBusyStartTimeMs = 0;
    bool _warnOnDriverBusyDone;
    static const uint32_t WARN_ON_DRIVER_BUSY_AFTER_MS = 100;

    // Vfs value calculated when setting current
    static const constexpr double VREF_LOW_SENSE = 0.325;
    static const constexpr double VREF_HIGH_SENSE = 0.180;
    double _vfsValue = VREF_LOW_SENSE;
    double _maxSenseVoltage = 0.0;

    // Calculate Vfs
    void calculateVfsAndMaxSenseV(double reqCurrentAmps, double &vfsValue, double &maxSenseVoltage)
    {
        // Total resistance
        double totalResOhms = _requestedParams.extSenseOhms + 0.02;

        // IRUN value calculation
        maxSenseVoltage = reqCurrentAmps * totalResOhms * 1.41;

        // Calculate a vsense value
        bool vsenseOut = maxSenseVoltage < VREF_HIGH_SENSE;

        // Vfs
        vfsValue = vsenseOut ? VREF_HIGH_SENSE : VREF_LOW_SENSE;
    }
 
    // Get microsteps (either from request, default or from read value)
    uint32_t getMicrosteps() const
    {
        if (!_driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].readValid)
            return _requestedParams.microsteps;
        uint32_t microstepsCode = (_driverRegisters[DRIVER_REGISTER_CODE_CHOPCONF].regValCur & TMC_2209_CHOPCONF_MRES_MASK) >> TMC_2209_CHOPCONF_MRES_BIT;
        switch(microstepsCode)
        {
            case 0: return 256;
            case 1: return 2;
            case 2: return 4;
            case 3: return 8;
            case 4: return 16;
            case 5: return 32;
            case 6: return 64;
            case 7: return 128;
            case 8: return 256;
            case 9: return 512;
            case 10: return 1024;
            default: return 256;
        }
    }

    // Get RMS amps
    double getRMSAmps() const
    {
        if (!_driverRegisters[DRIVER_REGISTER_CODE_IHOLD_IRUN].readValid)
            return _requestedParams.rmsAmps;
        uint32_t irun = (_driverRegisters[DRIVER_REGISTER_CODE_IHOLD_IRUN].regValCur & TMC_2209_IRUN_MASK) >> TMC_2209_IRUN_BIT;
        double rmsAmps = ((irun + 1) / 32.0) * _vfsValue / (_requestedParams.extSenseOhms + 0.02) / 1.414;
        return rmsAmps;
    }

    // GSTAT Json
    String getGSTATJson(bool includeBraces = true) const
    {
        // Check bus and driver status
        if (!busValid() || !_driverRegisters[DRIVER_REGISTER_CODE_GSTAT].readValid)
            return includeBraces ? "[]" : "";

        // Get status
        String jsonStr = "";
        uint32_t gstat = _driverRegisters[DRIVER_REGISTER_CODE_GSTAT].regValCur;
        if (gstat & (1 << TMC_2209_GSTAT_RESET_BIT))
            jsonStr += "\"RESET\"";
        if (gstat & (1 << TMC_2209_GSTAT_DRV_ERR_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"DRV_ERR\"");
        if (gstat & (1 << TMC_2209_GSTAT_UV_CP_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"UV_CP\"");

        // Braces
        if (includeBraces)
            return "[" + jsonStr + "]";
        return jsonStr;
    }

    // Driver status
    String getDriverStatusJson(bool includeBraces = true) const
    {
        // Check bus and driver status
        if (!busValid() || !_driverRegisters[DRIVER_REGISTER_CODE_DRV_STATUS].readValid)
            return includeBraces ? "[]" : "";

        // Get status
        String jsonStr = "";
        uint32_t drvStatus = _driverRegisters[DRIVER_REGISTER_CODE_DRV_STATUS].regValCur;
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_OTPW_BIT))
            jsonStr += "\"OVER_TEMP_PRE_WARNING\"";
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_OT_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"OVER_TEMP\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_S2GA_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"SHORT_TO_GND_A\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_S2GB_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"SHORT_TO_GND_B\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_S2VSA_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"SHORT_TO_VCC_A\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_S2VSB_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"SHORT_TO_VCC_B\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_OLA_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"OPEN_LOAD_A\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_OLB_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"OPEN_LOAD_B\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_T157_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"T>157\"");
        else if (drvStatus & (1 << TMC_2209_DRV_STATUS_T150_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"T>150\"");
        else if (drvStatus & (1 << TMC_2209_DRV_STATUS_T143_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"T>143\"");
        else if (drvStatus & (1 << TMC_2209_DRV_STATUS_T120_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"T>120\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_STCHOP_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"ST_CHOP\"");
        if (drvStatus & (1 << TMC_2209_DRV_STATUS_STST_BIT))
            jsonStr += (jsonStr.length() > 0 ? "," : "") + String("\"STANDSTILL\"");

        // Braces
        if (includeBraces)
            return "[" + jsonStr + "]";        
        return jsonStr;
    }

    // Helpers
    void setPDNDisable(bool disable);
    uint32_t getMRESFieldValue(uint32_t microsteps) const;
    void convertRMSCurrentToRegs(double reqCurrentAmps, double holdFactor, 
            StepDriverParams::HoldModeEnum holdMode, bool& vsenseOut, uint32_t& irunOut, uint32_t& iholdOut) const;
    void setMainRegs();

    // TMC2209 Defs
    static const uint8_t TMC_2209_SYNC_BYTE = 5;
    static constexpr double TMC_2209_CLOCK_FREQ_HZ = 12000000.0;

    // PDN_UART_BIT should be 1 to enable UART operation
    // MSTEP_REG_SELECT_BIT should be 1 to indicate microstepping to be controlled by MSTEP register
    // and software pulse optimization (which is the default in any case)
    static const uint32_t TMC_2209_GCONF_EXT_VREF_BIT = 0;
    static const uint32_t TMC_2209_GCONF_EXT_SENSE_RES_BIT = 1;
    static const uint32_t TMC_2209_GCONF_INV_DIRN_BIT = 3;
    static const uint32_t TMC_2209_GCONF_PDN_UART_BIT = 6;
    static const uint32_t TMC_2209_GCONF_MSTEP_REG_SELECT_BIT = 7;
    static const uint32_t TMC_2209_GCONF_MULTISTEP_FILT_BIT = 8;

    // GSTAT register consts
    static const uint32_t TMC_2209_GSTAT_RESET_BIT = 0;
    static const uint32_t TMC_2209_GSTAT_DRV_ERR_BIT = 1;
    static const uint32_t TMC_2209_GSTAT_UV_CP_BIT = 2;
    static const uint32_t TMC_2209_GSTAT_RESET_MASK = (1 << TMC_2209_GSTAT_RESET_BIT);

    // CHOPCONF register consts
    static const uint32_t TMC_2209_CHOPCONF_TOFF_BIT = 0;
    static const uint32_t TMC_2209_CHOPCONF_VSENSE_BIT = 17;
    static const uint32_t TMC_2209_CHOPCONF_MRES_BIT = 24;
    static const uint32_t TMC_2209_CHOPCONF_MRES_MASK = 0x0F000000;
    static const uint32_t TMC_2209_CHOPCONF_MRES_DEFAULT = 8;
    static const uint32_t TMC_2209_CHOPCONF_INTPOL_BIT = 28;

    // IHOLD_IRUN register consts
    static const uint32_t TMC_2209_IHOLD_BIT = 0;
    static const uint32_t TMC_2209_IRUN_BIT = 8;
    static const uint32_t TMC_2209_IRUN_MASK = 0x1F00;
    static const uint32_t TMC_2209_IHOLD_DELAY_BIT = 16;

    // PWMCONF register consts
    static const uint32_t TMC_2209_PWMCONF_PWM_OFS_BIT = 0;
    static const uint32_t TMC_2209_PWMCONF_PWM_GRAD_BIT = 8;
    static const uint32_t TMC_2209_PWMCONF_PWM_FREQ_BIT = 16;
    static const uint32_t TMC_2209_PWMCONF_AUTOSCALE_BIT = 18;
    static const uint32_t TMC_2209_PWMCONF_AUTOGRAD_BIT = 19;
    static const uint32_t TMC_2209_PWMCONF_FREEWHEEL_BIT = 20;
    static const uint32_t TMC_2209_PWMCONF_PWM_REG_BIT = 24;
    static const uint32_t TMC_2209_PWMCONF_PWM_LIM_BIT = 28;

    // PWMCONF DEFAULTS
    static const uint32_t TMC_2209_PWMCONF_PWM_OFS = 36;
    static const uint32_t TMC_2209_PWMCONF_PWM_GRAD = 0;

    // DRV_STATUS register consts
    static const uint32_t TMC_2209_DRV_STATUS_OTPW_BIT = 0;
    static const uint32_t TMC_2209_DRV_STATUS_OT_BIT = 1;
    static const uint32_t TMC_2209_DRV_STATUS_S2GA_BIT = 2;
    static const uint32_t TMC_2209_DRV_STATUS_S2GB_BIT = 3;
    static const uint32_t TMC_2209_DRV_STATUS_S2VSA_BIT = 4;
    static const uint32_t TMC_2209_DRV_STATUS_S2VSB_BIT = 5;
    static const uint32_t TMC_2209_DRV_STATUS_OLA_BIT = 6;
    static const uint32_t TMC_2209_DRV_STATUS_OLB_BIT = 7;
    static const uint32_t TMC_2209_DRV_STATUS_T120_BIT = 8;
    static const uint32_t TMC_2209_DRV_STATUS_T143_BIT = 9;
    static const uint32_t TMC_2209_DRV_STATUS_T150_BIT = 10;
    static const uint32_t TMC_2209_DRV_STATUS_T157_BIT = 11;
    static const uint32_t TMC_2209_DRV_STATUS_CS_ACTUAL_BIT = 16;
    static const uint32_t TMC_2209_DRV_STATUS_CS_ACTUAL_MASK = 0x001F0000;
    static const uint32_t TMC_2209_DRV_STATUS_STCHOP_BIT = 30;
    static const uint32_t TMC_2209_DRV_STATUS_STST_BIT = 31;

    // Debug
    static constexpr const char* MODULE_PREFIX = "StepDriverTMC2209";    
};
