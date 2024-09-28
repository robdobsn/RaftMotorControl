/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverParams
//
// Rob Dobson 2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "ConfigPinMap.h"

class StepDriverParams
{
public:
    static const uint32_t MICROSTEPS_DEFAULT = 256;
    static constexpr float EXT_SENSE_OHMS_DEFAULT = 0.11;
    static constexpr float RMS_AMPS_DEFAULT = 1.0;
    static constexpr float HOLD_MULT_DEFAULT = 1.0;
    static const uint32_t IHOLD_DELAY_DEFAULT = 0;
    static constexpr uint32_t TOFF_VALUE_DEFAULT = 5;
    static const uint32_t PWM_FREQ_KHZ_DEFAULT = 35;
    static const uint32_t STATUS_INTERVAL_MS_DEFAULT = 100;

    enum HoldModeEnum
    {
        HOLD_MODE_FACTOR,
        HOLD_MODE_FREEWHEEL,
        HOLD_MODE_PASSIVE_BREAKING
    };
    static const HoldModeEnum HOLD_MODE_DEFAULT = HOLD_MODE_FACTOR;
   
    bool noUART : 1 = false;
    bool invDirn : 1 = false;
    bool writeOnly : 1 = false;
    bool extVRef : 1 = false;
    bool extMStep : 1 = false;
    bool intpol : 1 = false;
    float extSenseOhms = EXT_SENSE_OHMS_DEFAULT;
    uint16_t microsteps = MICROSTEPS_DEFAULT;
    uint16_t minPulseWidthUs = 1;
    int stepPin = -1;
    int dirnPin = -1;
    float rmsAmps = RMS_AMPS_DEFAULT;
    float holdFactor = HOLD_MULT_DEFAULT;
    HoldModeEnum holdMode = HOLD_MODE_DEFAULT;
    uint32_t holdDelay = IHOLD_DELAY_DEFAULT;
    float pwmFreqKHz = PWM_FREQ_KHZ_DEFAULT;
    uint8_t address = 0;
    uint32_t statusIntvMs = STATUS_INTERVAL_MS_DEFAULT;

    StepDriverParams()
    {
    }

    StepDriverParams(RaftJsonIF& config)
    {
        // Get step controller settings
        microsteps = config.getLong("microsteps", StepDriverParams::MICROSTEPS_DEFAULT);
        writeOnly = config.getBool("writeOnly", 0);

        // Get hardware stepper params
        String stepPinName = config.getString("stepPin", "-1");
        stepPin = ConfigPinMap::getPinFromName(stepPinName.c_str());
        String dirnPinName = config.getString("dirnPin", "-1");
        dirnPin = ConfigPinMap::getPinFromName(dirnPinName.c_str());
        noUART = config.getBool("noUART", 0);
        invDirn = config.getBool("invDirn", 0);
        extSenseOhms = config.getDouble("extSenseOhms", StepDriverParams::EXT_SENSE_OHMS_DEFAULT);
        extVRef = config.getBool("extVRef", false);
        extMStep = config.getBool("extMStep", false);
        intpol = config.getBool("intpol", false);
        minPulseWidthUs = config.getLong("minPulseWidthUs", 1);
        rmsAmps = config.getDouble("rmsAmps", StepDriverParams::RMS_AMPS_DEFAULT);
        holdDelay = config.getLong("holdDelay", StepDriverParams::IHOLD_DELAY_DEFAULT);
        pwmFreqKHz = config.getDouble("pwmFreqKHz", StepDriverParams::PWM_FREQ_KHZ_DEFAULT);
        address = config.getLong("addr", 0);

        // Get status read frequency
        double statusFreqHz = config.getDouble("statusFreqHz", 0);
        statusIntvMs = statusFreqHz > 0 ? 1000.0 / statusFreqHz : 0;

        // Hold mode
        String holdModeStr = config.getString("holdModeOrFactor", "1.0");
        if (holdModeStr.equalsIgnoreCase("freewheel"))
        {
            holdMode = StepDriverParams::HOLD_MODE_FREEWHEEL;
            holdFactor = 0;
        }
        else if (holdModeStr.equalsIgnoreCase("passive"))
        {
            holdMode = StepDriverParams::HOLD_MODE_PASSIVE_BREAKING;
            holdFactor = 0;
        }
        else
        {
            holdMode = StepDriverParams::HOLD_MODE_FACTOR;
            holdFactor = strtof(holdModeStr.c_str(), NULL);
        }
    }

    String getDebugJSON(bool includeBraces = true) const
    {
        String jsonStr;
        jsonStr += "\"ad\":" + String(address);
        jsonStr += ",\"sP\":" + String(stepPin);
        jsonStr += ",\"dP\":" + String(dirnPin);
        jsonStr += ",\"iD\":" + String(invDirn);
        jsonStr += ",\"mS\":" + String(microsteps);
        jsonStr += ",\"wO\":" + String(writeOnly ? 1 : 0);
        jsonStr += ",\"eSO\":" + String(extSenseOhms);
        jsonStr += ",\"exV\":" + String(extVRef ? 1 : 0);
        jsonStr += ",\"exM\":" + String(extMStep ? 1 : 0);
        jsonStr += ",\"int\":" + String(intpol ? 1 : 0);
        jsonStr += ",\"rms\":" + String(rmsAmps, 2);
        jsonStr += ",\"hldM\":" + String(holdMode);
        jsonStr += ",\"hldF\":" + String(holdFactor, 2);
        jsonStr += ",\"hldD\":" + String(holdDelay);
        jsonStr += ",\"pwm\":" + String(pwmFreqKHz, 2);
        return includeBraces ? "{" + jsonStr + "}" : jsonStr;
    }
};

