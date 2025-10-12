/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPatternBase
// Base class for motion patterns (analogous to LEDPatternBase)
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "RaftArduino.h"
#include "NamedValueProvider.h"

class MotionControlIF;
class MotionPatternBase;

// Create function for motion pattern factory
typedef MotionPatternBase* (*MotionPatternCreateFn)(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl);

// Base class for motion patterns
class MotionPatternBase
{
public:
    MotionPatternBase(NamedValueProvider* pNamedValueProvider, MotionControlIF& motionControl) :
        _motionControl(motionControl)
    {
        if (pNamedValueProvider)
            _pNamedValueProvider = pNamedValueProvider;
        else
            _pNamedValueProvider = NamedValueProvider::getNullProvider();
    }

    virtual ~MotionPatternBase()
    {
    }

    /// @brief Setup pattern with optional parameters
    /// @param pParamsJson JSON parameters for pattern configuration
    virtual void setup(const char* pParamsJson = nullptr) = 0;

    /// @brief Service loop - called frequently to advance pattern state
    virtual void loop() = 0;

    /// @brief Motion pattern list item for factory
    struct MotionPatternListItem
    {
        String name;
        MotionPatternCreateFn createFn;
    };

protected:
    /// @brief Refresh rate for pattern service loop (ms)
    uint32_t _refreshRateMs = 10;

    /// @brief Named value provider for accessing system state
    NamedValueProvider* _pNamedValueProvider = nullptr;

    /// @brief Motion control interface
    MotionControlIF& _motionControl;
};
