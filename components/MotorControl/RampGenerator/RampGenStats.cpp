/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenStats
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "esp_attr.h"
#include "RampGenStats.h"

String RampGenStats::getStatsStr() const
{
#ifndef RAMP_GEN_DETAILED_STATS
    char dbg[100];
    sprintf(dbg, "ISR Avg %0.2fus Max %ldus", _isrAvgUs, (unsigned long)_isrMaxUs);
    return dbg;
#else
    char dbg[500];
    sprintf(dbg, "AvgISRUs %0.2f MaxISRUs %d curAccumStep %lu curStepRtPerTTicks %lu curAccumNS %lu axisIdxMaxStp %d accStpPerTTicksPerMS %lu curStepCtMajAx %u stepsBeforeDecel %u maxStepRatePerTTicks %lu",
        _isrAvgUs, 
        (int)_isrMaxUs,
        (long int)_curAccumulatorStep, 
        (long int)_curStepRatePerTTicks, 
        (long int)_curAccumulatorNS, 
        _axisIdxWithMaxSteps, 
        (long int)_accStepsPerTTicksPerMS, 
        (int)_curStepCountMajorAxis, 
        (int)_stepsBeforeDecel, 
        (long int)_maxStepRatePerTTicks);
    return dbg;
#endif
}

RampGenStats::RampGenStats()
{
    clear();
}

void RampGenStats::clear()
{
    _isrStartUs = 0;
    _isrAccUs = 0;
    _isrCount = 0;
    _isrAvgUs = 0;
    _isrAvgValid = false;
    _isrMaxUs = 0;
#ifdef RAMP_GEN_DETAILED_STATS
    _curAccumulatorStep = 0;
    _curStepRatePerTTicks = 0;
    _curAccumulatorNS = 0;
    _axisIdxWithMaxSteps = -1;
    _accStepsPerTTicksPerMS = 0;
    _curStepCountMajorAxis = 0;
    _stepsBeforeDecel = 0;
    _maxStepRatePerTTicks = 0;
#endif
}

void IRAM_ATTR RampGenStats::startMotionProcessing()
{
    _isrStartUs = micros();
}

void IRAM_ATTR RampGenStats::endMotionProcessing()
{
    uint32_t elapsedUs = micros() - _isrStartUs;
    // LOG_I(MODULE_PREFIX, "Elapsed uS %d", elapsedUs);
    _isrAccUs += elapsedUs;
    _isrCount++;
    if (_isrCount > 1000)
    {
        _isrAvgUs = _isrAccUs * 1.0 / _isrCount;
        _isrAvgValid = true;
        _isrCount = 0;
        _isrAccUs = 0;
    }
    if (_isrMaxUs < elapsedUs)
        _isrMaxUs = elapsedUs;
}

void IRAM_ATTR RampGenStats::update(uint32_t curAccumulatorStep, 
        uint32_t curStepRatePerTTicks,
        uint32_t curAccumulatorNS,
        int axisIdxWithMaxSteps,
        uint32_t accStepsPerTTicksPerMS,
        uint32_t curStepCountMajorAxis,
        uint32_t stepsBeforeDecel,
        uint32_t maxStepRatePerTTicks)
{
#ifdef RAMP_GEN_DETAILED_STATS
    _curAccumulatorNS = curAccumulatorNS;
    _curStepRatePerTTicks = curStepRatePerTTicks;
    _axisIdxWithMaxSteps = axisIdxWithMaxSteps;
    _accStepsPerTTicksPerMS = _accStepsPerTTicksPerMS;
    _curStepCountMajorAxis = _curStepCountMajorAxis;
    _stepsBeforeDecel = stepsBeforeDecel;
    _maxStepRatePerTTicks = maxStepRatePerTTicks;
#endif
}

void IRAM_ATTR RampGenStats::stepDirn(uint32_t axisIdx, bool dirnPositive)
{
#ifdef RAMP_GEN_DETAILED_STATS
#endif
}

void IRAM_ATTR RampGenStats::stepStart(uint32_t axisIdx)
{
#ifdef RAMP_GEN_DETAILED_STATS
#endif
}
