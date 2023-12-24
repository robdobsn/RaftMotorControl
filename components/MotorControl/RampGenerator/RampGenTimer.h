/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenTimer
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <vector>
#include <RaftArduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// #define RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS

// General purpose timer used for ramp generation
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#define RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
#include "driver/gptimer.h"
#else
#include "driver/timer.h"
#endif


// This is intended to be used statically in the RampGenerator class
// setup() must be called to initialise and the timer will be started by the
// first RampGenerator object that calls start()

typedef void (*RampGenTimerCB)(void* pObject);

class RampGenTimer
{
public:
    RampGenTimer();
    virtual ~RampGenTimer();
    bool setup(uint32_t timerPeriodUs);
    void shutdown();
    void enable(bool en);
    uint32_t getPeriodUs() const
    {
        return _timerPeriodUs;
    }
    bool hookTimer(RampGenTimerCB timerCB, void* pObject);
    void unhookTimer(void* pObject);

    // Debug
    uint32_t getDebugISRCount();
    uint64_t getDebugRawCount();
    String getDebugStr() const;
    
    // Default ramp generation timer period us
    static constexpr uint32_t RAMP_GEN_PERIOD_US_DEFAULT = 20;

private:
    bool _timerIsSetup = false;
    bool _timerIsEnabled = false;

    // Configured timer period
    uint32_t _timerPeriodUs = RampGenTimer::RAMP_GEN_PERIOD_US_DEFAULT;

#ifndef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS

    // ESP32 timer
    timer_isr_handle_t _rampTimerHandle = nullptr;
    static constexpr uint32_t RAMP_TIMER_DIVIDER = 80;
    timer_group_t _timerGroup = TIMER_GROUP_0;
    timer_idx_t _timerIdx = TIMER_0;

#else
    // Timer handle
    gptimer_handle_t _timerHandle = nullptr;

#endif

    // Debug timer count
    volatile uint32_t _timerISRCount = 0;

    // Hook info for timer callback
    struct TimerCBHook
    {
        RampGenTimerCB timerCB = nullptr;
        void* pObject = nullptr;
    };
    // Vector of callback hooks
    static const uint32_t MAX_TIMER_CB_HOOKS = 20;
    std::vector<TimerCBHook> _timerCBHooks;

#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
    // Mutex for callback hooks vector
    static SemaphoreHandle_t _hookListMutex;
#endif

    // ISR
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
    static bool _staticGPTimerCB(gptimer_t* timer, const gptimer_alarm_event_data_t* eventData, void* arg);
    void _nonStaticGPTimerCB();
#else
    static void _staticLegacyISR(void* arg);
    void _nonStaticLegacyISR();
#endif
    void disableTimerInterrupts();
    void reenableTimerInterrupts();
    void timerReset();
};
