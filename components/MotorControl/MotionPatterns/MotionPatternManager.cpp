/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPatternManager
// Manages motion patterns - analogous to LED segment pattern management
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotionPatternManager.h"
#include "MotionControlIF.h"
#include "RaftJson.h"
#include "Logger.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MotionPatternManager::MotionPatternManager()
{
}

MotionPatternManager::~MotionPatternManager()
{
    stopPattern(false);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add a pattern to the registry
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionPatternManager::addPattern(const String& patternName, MotionPatternCreateFn createFn)
{
    // Check for existing pattern with same name and remove if so
    for (auto it = _patterns.begin(); it != _patterns.end(); ++it)
    {
        if ((*it).name.equalsIgnoreCase(patternName))
        {
            _patterns.erase(it);
            break;
        }
    }

    // Add pattern
    _patterns.push_back({patternName, createFn});

#ifdef DEBUG_MOTION_PATTERN_START_STOP
    LOG_I(MODULE_PREFIX, "addPattern %s (total patterns: %d)", patternName.c_str(), _patterns.size());
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get pattern names
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionPatternManager::getPatternNames(std::vector<String>& patternNames) const
{
    for (const auto& pattern : _patterns)
    {
        patternNames.push_back(pattern.name);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set named value provider
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionPatternManager::setNamedValueProvider(NamedValueProvider* pNamedValueProvider)
{
    _pNamedValueProvider = pNamedValueProvider;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start a pattern
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionPatternManager::setPattern(MotionControlIF& motionControl, const String& patternName,
                                      uint32_t patternRunTimeDefaultMs, const char* pParamsJson)
{
    // Save current pattern name for logging
    String prevPatternName = _currentPatternName;

    // Stop existing pattern
    stopPattern(true);

    // Handle empty pattern name (just stop)
    if (patternName.length() == 0)
    {
#ifdef DEBUG_MOTION_PATTERN_START_STOP
        LOG_I(MODULE_PREFIX, "setPattern cleared %s", prevPatternName.c_str());
#endif
        return;
    }

    // Find pattern in registry
    for (const auto& pattern : _patterns)
    {
        if (pattern.name.equalsIgnoreCase(patternName))
        {
            // Create pattern instance
            MotionPatternBase* pPattern = pattern.createFn(_pNamedValueProvider, motionControl);
            if (pPattern)
            {
                // Set current pattern
                _pCurrentPattern = pPattern;
                _currentPatternName = patternName;

                // Setup pattern
                _pCurrentPattern->setup(pParamsJson);

                // Check if pattern duration is specified in params
                _patternDurationMs = patternRunTimeDefaultMs;
                if (pParamsJson)
                {
                    RaftJson paramsJson(pParamsJson);
                    _patternDurationMs = paramsJson.getInt("forMs", patternRunTimeDefaultMs);
                }
                _patternStartMs = millis();

#ifdef DEBUG_MOTION_PATTERN_START_STOP
                LOG_I(MODULE_PREFIX, "setPattern %s OK paramsJson %s duration %s",
                        patternName.c_str(), pParamsJson ? pParamsJson : "NONE",
                        _patternDurationMs == 0 ? "FOREVER" : (String(_patternDurationMs) + "ms").c_str());
#endif
            }
            else
            {
                LOG_E(MODULE_PREFIX, "setPattern %s failed to create instance", patternName.c_str());
            }
            return;
        }
    }

    // Pattern not found
    LOG_W(MODULE_PREFIX, "setPattern %s NOT FOUND", patternName.c_str());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Stop current pattern
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void MotionPatternManager::stopPattern(bool stopMotion)
{
    // Stop no longer requested
    _stopRequested = false;

    // Delete pattern instance
    if (_pCurrentPattern)
    {
#ifdef DEBUG_MOTION_PATTERN_START_STOP
        LOG_I(MODULE_PREFIX, "stopPattern %s", _currentPatternName.c_str());
#endif
        delete _pCurrentPattern;
        _pCurrentPattern = nullptr;
        _currentPatternName = "";
    }

    // Stop motion if requested
    if (stopMotion)
    {
        // Note: Motion stopping would be handled by caller if needed
        // We don't stop motion directly here to avoid coupling
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Service loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool MotionPatternManager::loop(MotionControlIF& motionControl)
{
    // Check if pattern active
    if (!_pCurrentPattern)
        return false;

    // Check for timeout
    if ((_patternDurationMs > 0) && Raft::isTimeout(millis(), _patternStartMs, _patternDurationMs))
    {
#ifdef DEBUG_MOTION_PATTERN_DURATION
        LOG_I(MODULE_PREFIX, "loop pattern %s duration expired", _currentPatternName.c_str());
#endif
        stopPattern(true);
        return false;
    }

    // Service pattern
    _pCurrentPattern->loop();

    // Check if stop requested
    if (_stopRequested)
    {
#ifdef DEBUG_MOTION_PATTERN_START_STOP
        LOG_I(MODULE_PREFIX, "loop pattern %s stop requested", _currentPatternName.c_str());
#endif
        stopPattern(true);
        return false;
    }

    return true;
}
