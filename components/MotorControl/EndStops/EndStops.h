/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// EndStops
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "RaftCore.h"

class EndStops
{
public:
    EndStops();
    virtual ~EndStops();

    // Clear
    void clear();

    // Add endstop
    void add(bool isMax, const char* name, int endStopPin, bool actvLevel, int inputType);

    // Loop - called frequently
    virtual void loop();

    // Check if at end stop
    bool isAtEndStop(bool max);

    // Check if valid
    bool isValid(bool max);

    // Get pin and level
    bool getPinAndLevel(bool max, int& pin, bool& actvLevel);

    // Get debug JSON
    String getDebugJSON(bool includeBraces, bool detailed) const;
    
private:

    // Debug
    static constexpr const char* MODULE_PREFIX = "EndStops";

    String _maxName;
    int _maxEndStopPin;
    bool _maxActLevel;
    int _maxInputType;

    String _minName;
    int _minEndStopPin;
    int _minActLevel;
    int _minInputType;
};
