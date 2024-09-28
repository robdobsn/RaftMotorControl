/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// EndStops
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "EndStops.h"
#include "RaftArduino.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
EndStops::EndStops()
{
    // Set initial unused value
    _maxEndStopPin = -1;
    _minEndStopPin = -1;

    // Clear
    clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
EndStops::~EndStops()
{
    clear();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Clear endstops
void EndStops::clear()
{
    // Check if inputs should be restored (may have had pullup)
    if (_maxEndStopPin >= 0)
        pinMode(_maxEndStopPin, INPUT);
    if (_minEndStopPin >= 0)
        pinMode(_minEndStopPin, INPUT);

    // Max endstop
    _maxEndStopPin = -1;
    _maxActLevel = false;
    _maxInputType = INPUT;

    // Min endstop
    _minEndStopPin = -1;
    _minActLevel = false;
    _minInputType = INPUT;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Add endstop
/// @param isMax True if max endstop (false for min)
/// @param name Name of endstop
/// @param endStopPin Pin number
/// @param actvLevel Active level
/// @param inputType Input type
void EndStops::add(bool isMax, const char* name, int endStopPin, bool actvLevel, int inputType)
{
    if (isMax)
    {
        _maxName = name;
        _maxEndStopPin = endStopPin;
        _maxActLevel = actvLevel;
        _maxInputType = inputType;
        if (_maxEndStopPin < 0)
            return;
        pinMode(_maxEndStopPin, inputType);
    }
    else
    {
        _minName = name;
        _minEndStopPin = endStopPin;
        _minActLevel = actvLevel;
        _minInputType = inputType;
        if (_minEndStopPin < 0)
            return;
        pinMode(_minEndStopPin, inputType);
    }
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Loop function - called frequently
void EndStops::loop()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Check if at end stop
/// @param max True if checking max endstop (false for min)
/// @return true if at endstop
bool IRAM_ATTR EndStops::isAtEndStop(bool max)
{
    if (max)
    {
        if (_maxEndStopPin < 0)
            return false;
        bool val = digitalRead(_maxEndStopPin);
        return val == _maxActLevel;
    }
    else
    {
        if (_minEndStopPin < 0)
            return false;
        bool val = digitalRead(_minEndStopPin);
        return val == _minActLevel;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check endstop valid
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IRAM_ATTR EndStops::isValid(bool max)
{
    if (max)
    {
        return _maxEndStopPin >= 0;
    }
    else
    {
        return _minEndStopPin >= 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get pin and level
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IRAM_ATTR EndStops::getPinAndLevel(bool max, int& pin, bool& actvLevel)
{
    if (max)
    {
        pin = _maxEndStopPin;
        actvLevel = _maxActLevel;
        return _maxEndStopPin >= 0;
    }
    else
    {
        pin = _minEndStopPin;
        actvLevel = _minActLevel;
        return _minEndStopPin >= 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// @brief Get debug JSON
String EndStops::getDebugJSON(bool includeBraces, bool detailed) const
{
    String retStr;
    if (includeBraces)
        retStr = "{";
    if (_maxEndStopPin >= 0)
    {
        retStr += "\"max\":{";
        retStr += "\"n\":\"" + _maxName + "\",";
        retStr += "\"p\":" + String(_maxEndStopPin) + ",";
        retStr += "\"lev\":" + String(_maxActLevel) + ",";
        retStr += "\"type\":" + String(_maxInputType) + "},";
    }
    if (_minEndStopPin >= 0)
    {
        retStr += "\"min\":{";
        retStr += "\"n\":\"" + _minName + "\",";
        retStr += "\"p\":" + String(_minEndStopPin) + ",";
        retStr += "\"lev\":" + String(_minActLevel) + ",";
        retStr += "\"type\":" + String(_minInputType) + "},";
    }
    if (includeBraces)
        retStr += "}";
    return retStr;
}
