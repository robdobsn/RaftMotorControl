/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenTimer
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#ifdef ESP_PLATFORM

#include "RaftCore.h"
#include "RampGenConsts.h"
#include "driver/gptimer.h"
#include "MotorControlConsts.h"

// #define RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS

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
    String getDebugJSON(bool includeBraces) const;
    
private:
    bool _timerIsSetup = false;
    bool _timerIsEnabled = false;

    // Configured timer period
    uint32_t _timerPeriodUs = RAMP_GEN_PERIOD_US_DEFAULT;

    // Timer handle
    gptimer_handle_t _timerHandle = nullptr;

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
    static MOTOR_TICK_FN_DECORATOR bool _staticGPTimerCB(gptimer_t* timer, const gptimer_alarm_event_data_t* eventData, void* arg)
    {
        // Check the arg is valid
        if (!arg)
            return false;
        RampGenTimer* pRampGenTimer = (RampGenTimer*)arg;
        pRampGenTimer->_nonStaticGPTimerCB();
        return false;
    }
    void MOTOR_TICK_FN_DECORATOR _nonStaticGPTimerCB()
    {
        // Bump count
        _timerISRCount = _timerISRCount + 1;

#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
        // Get semaphore on hooks vector
        BaseType_t xTaskWokenBySemphoreTake = pdFALSE;
        BaseType_t xTaskWokenBySemphoreGive = pdFALSE;
        if (xSemaphoreTakeFromISR(_hookListMutex, &xTaskWokenBySemphoreTake) != pdTRUE)
            return;
#endif

        // Call each function
        for (auto& hook : _timerCBHooks)
        {
            if (hook.timerCB)
            {
                hook.timerCB(hook.pObject);
            }
        }

#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
        // Release semaphore
        xSemaphoreGiveFromISR(_hookListMutex, &xTaskWokenBySemphoreGive);

        // Check if a task was woken
        // See notes on this here: https://www.freertos.org/xSemaphoreTakeFromISR.html
        if ((xTaskWokenBySemphoreTake != pdFALSE) || ((xTaskWokenBySemphoreGive != pdFALSE)))
        {
            taskYIELD ();
        }
#endif    

    }

    // Timer control
    void disableTimerInterrupts();
    void reenableTimerInterrupts();
    void timerReset();

    // Debug
    static constexpr const char* MODULE_PREFIX = "RampGenTimer";    
};

#endif // ESP_PLATFORM