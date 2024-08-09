/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverParams
//
// Rob Dobson 2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>

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
};

