/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotorEnabler - enable/disable stepper motors
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftCore.h"
#include "time.h"

class MotorEnabler
{
public:
    static constexpr float stepDisableSecs_default = 60.0f;

    MotorEnabler()
    {
    }
    ~MotorEnabler()
    {
        deinit();
    }
    void deinit()
    {
#if defined(ARDUINO) || defined(ESP_PLATFORM)
        // disable
        if (_stepEnablePin >= 0)
            pinMode(_stepEnablePin, INPUT);
#endif
    }

    bool setup(const RaftJsonIF& config)
    {
        static const char* MODULE_PREFIX = "MotorEnabler";

        // Get motor enable info
        String stepEnablePinName = config.getString("stepEnablePin", "-1");
        _stepEnLev = config.getLong("stepEnLev", 1);
        _stepEnablePin = ConfigPinMap::getPinFromName(stepEnablePinName.c_str());
        _stepDisableSecs = config.getDouble("stepDisableSecs", stepDisableSecs_default);
        LOG_I(MODULE_PREFIX, "setup pin %d, actLvl %d, disableAfter %fs", 
                    _stepEnablePin, _stepEnLev, _stepDisableSecs);

#if defined(ARDUINO) || defined(ESP_PLATFORM)
        // Enable pin - initially disable
        if (_stepEnablePin >= 0)
        {
            pinMode(_stepEnablePin, OUTPUT);
            digitalWrite(_stepEnablePin, !_stepEnLev);
        }
#endif
        return true;
    }

    void enableMotors(bool en, bool timeout)
    {
        static const char* MODULE_PREFIX = "MotorEnabler";
        // LOG_I(MODULE_PREFIX, "Enable %d currentlyEn %d pin %d disable level %d, disable after time %f",
        // 							en, _motorsAreEnabled, _stepEnablePin, !_stepEnLev, _stepDisableSecs);
        if (en)
        {
            if (_stepEnablePin >= 0)
            {
                if (!_motorsAreEnabled)
                    LOG_I(MODULE_PREFIX, "MotorEnabler: enabled, disable after idle %fs (enPin %d level %d)", 
                                _stepDisableSecs, _stepEnablePin, _stepEnLev);
#if defined(ARDUINO) || defined(ESP_PLATFORM)
                digitalWrite(_stepEnablePin, _stepEnLev);
#endif
            }
            _motorsAreEnabled = true;
            _motorEnLastMillis = millis();
            time(&_motorEnLastUnixTime);
        }
        else
        {
            if (_stepEnablePin >= 0)
            {
                if (_motorsAreEnabled)
                {
                    LOG_I(MODULE_PREFIX, "MotorEnabler: motors disabled by %s", timeout ? ("timeout(" + String(_stepDisableSecs) + "s)").c_str() : "command");
                }
#if defined(ARDUINO) || defined(ESP_PLATFORM)
                digitalWrite(_stepEnablePin, !_stepEnLev);
#endif
            }
            _motorsAreEnabled = false;
        }
    }

    unsigned long getLastActiveUnixTime()
    {
        return _motorEnLastUnixTime;
    }

    void loop()
    {
        // Check for motor enable timeout
        if (_motorsAreEnabled && Raft::isTimeout(millis(), _motorEnLastMillis,
                                                    (unsigned long)(_stepDisableSecs * 1000)))
            enableMotors(false, true);
    }

    void setMotorOnTimeAfterMoveSecs(float motorOnTimeAfterMoveSecs)
    {
        if (motorOnTimeAfterMoveSecs <= 0)
            motorOnTimeAfterMoveSecs = 1;
        _stepDisableSecs = motorOnTimeAfterMoveSecs;
    }

private:
    // Step enable
    int _stepEnablePin = -1;
    bool _stepEnLev = true;
    // Motor enable
    float _stepDisableSecs = 10;
    bool _motorsAreEnabled = false;
    unsigned long _motorEnLastMillis = 0;
    time_t _motorEnLastUnixTime = 0;
};
