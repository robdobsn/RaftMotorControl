/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenStats
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RampGenStats.h"
#include "MotorControlConsts.h"

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

String RampGenStats::getJSON(bool includeBraces, bool detailed) const
{
    String jsonStr = "{";
    jsonStr += "\"isrAvUs\":" + String(_isrAvgUs, 2) + ",";
    jsonStr += "\"isrMxUs\":" + String(_isrMaxUs) + ",";
    jsonStr += "\"isrAvOk\":" + String(_isrAvgValid ? "1" : "0");
#ifdef RAMP_GEN_DETAILED_STATS
    if (detailed)
    {
        jsonStr += ",";
        jsonStr += "\"accStp\":" + String(_curAccumulatorStep) + ",";
        jsonStr += "\"stpsPTTk\":" + String(_curStepRatePerTTicks) + ",";
        jsonStr += "\"accNs\":" + String(_curAccumulatorNS) + ",";
        jsonStr += "\"axMax\":" + String(_axisIdxWithMaxSteps) + ",";
        jsonStr += "\"accStpPTTkPms\":" + String(_accStepsPerTTicksPerMS) + ",";
        jsonStr += "\"stpCtMaj\":" + String(_curStepCountMajorAxis) + ",";
        jsonStr += "\"stpPreDec\":" + String(_stepsBeforeDecel) + ",";
        jsonStr += "\"maxStpPTTk\":" + String(_maxStepRatePerTTicks);
    }
#endif
    jsonStr += "}";
    return jsonStr;
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

void MOTOR_TICK_FN_DECORATOR RampGenStats::startMotionProcessing()
{
    _isrStartUs = micros();
}

void MOTOR_TICK_FN_DECORATOR RampGenStats::endMotionProcessing()
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

void MOTOR_TICK_FN_DECORATOR RampGenStats::update(uint32_t curAccumulatorStep, 
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

void MOTOR_TICK_FN_DECORATOR RampGenStats::stepDirn(uint32_t axisIdx, bool dirnPositive)
{
#ifdef RAMP_GEN_DETAILED_STATS
#endif
}

void MOTOR_TICK_FN_DECORATOR RampGenStats::stepStart(uint32_t axisIdx)
{
#ifdef RAMP_GEN_DETAILED_STATS
#endif
}
