/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RampGenTimer
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RampGenTimer.h"
#include "Logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Statics
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
// Mutex for callback hooks vector
SemaphoreHandle_t RampGenTimer::_hookListMutex = NULL;
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RampGenTimer::RampGenTimer()
{
}

RampGenTimer::~RampGenTimer()
{
    // Shutdown
    shutdown();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RampGenTimer::setup(uint32_t timerPeriodUs)
{
    // Check already setup
    if (_timerIsSetup)
        return true;

    _timerPeriodUs = timerPeriodUs;
    _timerIsEnabled = false;
    
#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
    // Mutex controlling hook vector access
    _hookListMutex = xSemaphoreCreateMutex();
#endif

    // Config gptimer
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    gptimer_config_t gptimerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0,
        .flags = 0,
    };
#pragma GCC diagnostic pop

    // Create gptimer
    if (gptimer_new_timer(&gptimerConfig, &_timerHandle) != ESP_OK)
    {
        LOG_E(MODULE_PREFIX, "Failed to create gptimer");
        return false;
    }

    // Setup alarm
    gptimer_alarm_config_t alarmConfig = {
        .alarm_count = timerPeriodUs,
        .reload_count = 0,                  // counter will reload with 0 on alarm event
        .flags = 
            {
                .auto_reload_on_alarm = true, // enable auto-reload
            }
    };
    if (gptimer_set_alarm_action(_timerHandle, &alarmConfig) != ESP_OK)
    {
        LOG_E(MODULE_PREFIX, "Failed to set gptimer alarm action");
        return false;
    }

    // Register event callbacks
    gptimer_event_callbacks_t alarmEventCBs = {
        .on_alarm = _staticGPTimerCB,
    };
    if (gptimer_register_event_callbacks(_timerHandle, &alarmEventCBs, (void*)this) != ESP_OK)
    {
        LOG_E(MODULE_PREFIX, "Failed to register event callbacks");
        return false;
    }

    // Timer is setup
    _timerIsSetup = true;

    // Debug
    LOG_I(MODULE_PREFIX, "Configured timer ok");
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Shutdown
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenTimer::shutdown()
{
    if (!_timerIsSetup)
        return;
    enable(false);
    gptimer_event_callbacks_t alarmEventCBs = {
        .on_alarm = nullptr,
    };
    gptimer_register_event_callbacks(_timerHandle, &alarmEventCBs, (void*)this);
    gptimer_alarm_config_t alarmConfig = {
        .alarm_count = 1000000,
        .reload_count = 0,                  // counter will reload with 0 on alarm event
        .flags = 
            {
                .auto_reload_on_alarm = false, // disable auto-reload
            }
    };
    gptimer_set_alarm_action(_timerHandle, &alarmConfig);
    gptimer_del_timer(_timerHandle);
#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
    if (_hookListMutex)
		vSemaphoreDelete(_hookListMutex);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Enable
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenTimer::enable(bool en)
{
    // Check valid
    if (!_timerIsSetup)
        return;

    if (en && !_timerIsEnabled)
    {
        gptimer_enable(_timerHandle);
        gptimer_start(_timerHandle);
    }
    else if (!en && _timerIsEnabled)
    {
        gptimer_stop(_timerHandle);
        gptimer_disable(_timerHandle);
    }
    _timerIsEnabled = en;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Hook / unhook
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RampGenTimer::hookTimer(RampGenTimerCB timerCB, void* pObject)
{
    // Check valid
    if (!_timerIsSetup)
        return false;

    // Check for max hooks
    if (_timerCBHooks.size() > MAX_TIMER_CB_HOOKS)
        return false;

#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
    // Get semaphore on hooks vector
	if (xSemaphoreTake(_hookListMutex, pdMS_TO_TICKS(1)) != pdTRUE)
		return false;
#else
    // Disable ISR
    disableTimerInterrupts();
    timerReset();
    delayMicroseconds(20);    
#endif

    LOG_I(MODULE_PREFIX, "Hooking timer callback %p arg %p", timerCB, pObject);

    // Add new hook
    TimerCBHook newHook = { timerCB, pObject };
    _timerCBHooks.push_back(newHook);
    
#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
	// Release semaphore
	xSemaphoreGive(_hookListMutex);
#else
    // Enable ISR
    delayMicroseconds(20);
    reenableTimerInterrupts();
#endif
    return true;
}

void RampGenTimer::unhookTimer(void* pObject)
{
    // Check valid
    if (!_timerIsSetup)
        return;

#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
    // Get semaphore on hooks vector
	if (xSemaphoreTake(_hookListMutex, pdMS_TO_TICKS(1)) != pdTRUE)
		return;
#else
    // Disable ISR
    disableTimerInterrupts();
    timerReset();
    delayMicroseconds(100);
#endif

    // Check for hook to remove
    for (auto it = _timerCBHooks.begin(); it != _timerCBHooks.end(); it++)
    {
        if (it->pObject == pObject)
        {
            _timerCBHooks.erase(it);
        }
        break;
    }
    
#ifdef RAMP_GEN_USE_SEMAPHORE_FOR_LIST_ACCESS
	// Release semaphore
	xSemaphoreGive(_hookListMutex);
    return true;
#else
    // Enable ISR
    delayMicroseconds(100);
    reenableTimerInterrupts();
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debug
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String RampGenTimer::getDebugStr() const
{
    uint32_t timerCount = _timerISRCount;
    char tmpStr[50];
    snprintf(tmpStr, sizeof(tmpStr), "ISRCount %lu", (unsigned long)timerCount);
    return tmpStr;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer interrupts
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenTimer::disableTimerInterrupts()
{
    // Check valid
    if (!_timerIsSetup)
        return;
    // Stop timer ISR
    if (_timerIsEnabled)
    {
        gptimer_stop(_timerHandle);
        gptimer_disable(_timerHandle);
    }
}

void RampGenTimer::reenableTimerInterrupts()
{
    // Check valid
    if (!_timerIsSetup)
        return;

    // Re-enable ISR
    if (_timerIsEnabled)
    {
        gptimer_enable(_timerHandle);
        gptimer_start(_timerHandle);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset timer (not ISR safe)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenTimer::timerReset()
{
    // Check valid
    if (!_timerIsSetup)
        return;

    // Reset timer
    gptimer_set_raw_count(_timerHandle, 0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get debug counts
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t RampGenTimer::getDebugISRCount()
{
    return _timerISRCount;
}

uint64_t RampGenTimer::getDebugRawCount()
{
    // Check valid
    if (!_timerIsSetup)
        return 0;

    uint64_t timerCount = 0;
    gptimer_get_raw_count(_timerHandle, &timerCount);
    return timerCount;
}
