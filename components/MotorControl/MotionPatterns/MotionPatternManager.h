/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotionPatternManager
// Manages motion patterns - analogous to LED segment pattern management
//
// Rob Dobson 2025
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include "MotionPatternBase.h"
#include "RaftArduino.h"

// #define DEBUG_MOTION_PATTERN_START_STOP
// #define DEBUG_MOTION_PATTERN_DURATION

class MotionControlIF;
class NamedValueProvider;

class MotionPatternManager
{
public:
    MotionPatternManager();
    ~MotionPatternManager();

    /// @brief Add a pattern to the registry
    /// @param patternName Name of the pattern
    /// @param createFn Factory function to create pattern instance
    void addPattern(const String& patternName, MotionPatternCreateFn createFn);

    /// @brief Get list of registered pattern names
    /// @param patternNames (out) Vector to receive pattern names
    void getPatternNames(std::vector<String>& patternNames) const;

    /// @brief Set named value provider for pattern parameterization
    /// @param pNamedValueProvider Pointer to named value provider
    void setNamedValueProvider(NamedValueProvider* pNamedValueProvider);

    /// @brief Start a pattern
    /// @param motionControl Motion control interface
    /// @param patternName Name of pattern to start
    /// @param patternRunTimeDefaultMs Default runtime in milliseconds (0 = run forever)
    /// @param pParamsJson Optional JSON parameters for pattern
    void setPattern(MotionControlIF& motionControl, const String& patternName,
                    uint32_t patternRunTimeDefaultMs = 0, const char* pParamsJson = nullptr);

    /// @brief Stop current pattern
    /// @param stopMotion If true, also stop any ongoing motion
    void stopPattern(bool stopMotion);

    /// @brief Service loop - call frequently
    /// @param motionControl Motion control interface
    /// @return true if pattern is active
    bool loop(MotionControlIF& motionControl);

    /// @brief Check if pattern is currently active
    /// @return true if pattern is running
    bool isPatternActive() const
    {
        return _pCurrentPattern != nullptr;
    }

    /// @brief Check if stop has been requested
    /// @return true if stop requested
    bool isStopRequested() const
    {
        return _stopRequested;
    }

    /// @brief Get name of current pattern
    /// @return Current pattern name (empty if none)
    const String& getCurrentPatternName() const
    {
        return _currentPatternName;
    }

private:
    // Pattern registry
    std::vector<MotionPatternBase::MotionPatternListItem> _patterns;

    // Current pattern state
    MotionPatternBase* _pCurrentPattern = nullptr;
    String _currentPatternName;
    uint32_t _patternStartMs = 0;
    uint32_t _patternDurationMs = 0;

    // Control flags
    bool _stopRequested = false;

    // Named value provider
    NamedValueProvider* _pNamedValueProvider = nullptr;

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotionPatternMgr";
};
