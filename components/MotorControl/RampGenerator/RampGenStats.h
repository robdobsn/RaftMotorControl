/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenStats
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <RaftArduino.h>

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
    uint64_t _isrStartUs;
    uint64_t _isrAccUs;
    uint32_t _isrCount;
    float _isrAvgUs;
    bool _isrAvgValid;
    uint32_t _isrMaxUs;
#ifdef RAMP_GEN_DETAILED_STATS
    uint32_t _curAccumulatorStep;
    uint32_t _curStepRatePerTTicks;
    uint32_t _curAccumulatorNS;
    int _axisIdxWithMaxSteps;
    uint32_t _accStepsPerTTicksPerMS;
    uint32_t _curStepCountMajorAxis;
    uint32_t _stepsBeforeDecel;
    uint32_t _maxStepRatePerTTicks;
#endif
};
