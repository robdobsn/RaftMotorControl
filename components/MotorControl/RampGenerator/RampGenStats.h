/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenStats
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "RaftArduino.h"

#define RAMP_GEN_DETAILED_STATS

// Stats
class RampGenStats
{
public:
    RampGenStats();
    void clear();
    void startMotionProcessing();
    void endMotionProcessing();
    void update(uint32_t curAccumulatorStep, 
            uint32_t curStepRatePerTTicks,
            uint32_t curAccumulatorNS,
            int axisIdxWithMaxSteps,
            uint32_t accStepsPerTTicksPerMS,
            uint32_t curStepCountMajorAxis,
            uint32_t stepsBeforeDecel,
            uint32_t maxStepRatePerTTicks);
    void stepDirn(uint32_t axisIdx, bool dirnPositive);
    void stepStart(uint32_t axisIdx);
    String getStatsStr();

private:
    // Stats
    uint64_t _isrStartUs = 0;
    uint64_t _isrAccUs = 0;
    uint32_t _isrCount = 0;
    float _isrAvgUs = 0;
    bool _isrAvgValid = false;
    uint32_t _isrMaxUs = 0;
#ifdef RAMP_GEN_DETAILED_STATS
    uint32_t _curAccumulatorStep = 0;
    uint32_t _curStepRatePerTTicks = 0;
    uint32_t _curAccumulatorNS = 0;
    int _axisIdxWithMaxSteps = -1;
    uint32_t _accStepsPerTTicksPerMS = 0;
    uint32_t _curStepCountMajorAxis = 0;
    uint32_t _stepsBeforeDecel = 0;
    uint32_t _maxStepRatePerTTicks = 0;
#endif
};
