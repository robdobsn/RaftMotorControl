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

static const char* MODULE_PREFIX = "RampGenTimer";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Statics
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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
#ifndef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
    if (_rampTimerHandle)
        esp_intr_free(_rampTimerHandle);
#endif
#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
    if (_hookListMutex)
		vSemaphoreDelete(_hookListMutex);
#endif
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
    
#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
    // Mutex controlling hook vector access
    _hookListMutex = xSemaphoreCreateMutex();
#endif

#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS

    // Config gptimer
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    gptimer_config_t gptimerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
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
        .alarm_count = timerPeriodUs,       // period = 1s @resolution 1MHz
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

    // Enable timer
    if (gptimer_enable(_timerHandle) != ESP_OK)
    {
        LOG_E(MODULE_PREFIX, "Failed to enable gptimer");
        return false;
    }

    // Timer is setup
    _timerIsSetup = true;

    // Debug
    LOG_I(MODULE_PREFIX, "Started timer ok");

#else
    // Setup timer
    timer_config_t timerConfig = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .clk_src = TIMER_SRC_CLK_APB,
#endif
        .divider = 80,   /* 1 us per tick */
    };
    if (timer_init(_timerGroup, _timerIdx, &timerConfig) == ESP_OK)
    {
        timer_set_counter_value(_timerGroup, _timerIdx, 0);
        timer_set_alarm_value(_timerGroup, _timerIdx, timerPeriodUs);
        timer_enable_intr(_timerGroup, _timerIdx);
        timer_isr_register(_timerGroup, _timerIdx, &_staticLegacyISR, 
                    (void*)this, 0, &_rampTimerHandle);
        _timerIsSetup = true;
        _timerIsEnabled = false;
        LOG_I(MODULE_PREFIX, "Started ISR timer for direct stepping");
        return true;
    }
    else
    {
        LOG_E(MODULE_PREFIX, "Failed to start ISR timer for direct stepping");            
    }

#endif

    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Enable
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RampGenTimer::enable(bool en)
{
    // Check valid
    if (!_timerIsSetup)
        return;

    if (en)
    {
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
        gptimer_start(_timerHandle);
#else
        timer_start(_timerGroup, _timerIdx);
#endif
    }
    else
    {
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
        gptimer_stop(_timerHandle);
#else
        timer_pause(_timerGroup, _timerIdx);
#endif
    }
    _timerIsEnabled = en;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get timer count
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint64_t RampGenTimer::getTimerCount()
{
    // Check valid
    if (!_timerIsSetup)
        return 0;

#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
    uint64_t timerCount = 0;
    gptimer_get_raw_count(_timerHandle, &timerCount);
    return timerCount;
#else
    return _timerCount;
#endif
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

#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
    // Get semaphore on hooks vector
	if (xSemaphoreTake(_hookListMutex, pdMS_TO_TICKS(1)) != pdTRUE)
		return false;
#else
    // Disable ISR
    disableTimerInterrupts();
    timerReset();
    delayMicroseconds(20);    
#endif

    // Add new hook
    TimerCBHook newHook = { timerCB, pObject };
    _timerCBHooks.push_back(newHook);
    
#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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

#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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
    
#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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
// Legacy ISR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS

void IRAM_ATTR RampGenTimer::_staticLegacyISR(void* arg)
{
    // Check the arg is valid
    if (!arg)
        return;
    RampGenTimer* pTimer = (RampGenTimer*)arg;
    pTimer->_nonStaticLegacyISR();
}

void IRAM_ATTR RampGenTimer::_nonStaticLegacyISR()
{

    // Handle ISR by calling each hooked callback
    _timerCount = _timerCount + 1;
    
#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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

#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
    // Release semaphore
	xSemaphoreGiveFromISR(_hookListMutex, &xTaskWokenBySemphoreGive);

    // Check if a task was woken
    // See notes on this here: https://www.freertos.org/xSemaphoreTakeFromISR.html
    if ((xTaskWokenBySemphoreTake != pdFALSE) || ((xTaskWokenBySemphoreGive != pdFALSE)))
    {
        taskYIELD ();
    }
#endif

#ifndef USE_ESP_IDF_NEW_TIMER_FUNCTIONS
    if (_timerGroup == 0)
    {
        if (_timerIdx == 0)
        {
            TIMERG0.int_clr_timers.t0 = 1;
            TIMERG0.hw_timer[0].config.alarm_en = 1;
        }
        else
        {
            TIMERG0.int_clr_timers.t1 = 1;
            TIMERG0.hw_timer[1].config.alarm_en = 1;
        }
    }
    else
    {
        if (_timerIdx == 0)
        {
            TIMERG1.int_clr_timers.t0 = 1;
            TIMERG1.hw_timer[0].config.alarm_en = 1;
        }
        else
        {
            TIMERG1.int_clr_timers.t1 = 1;
            TIMERG1.hw_timer[1].config.alarm_en = 1;
        }
    }    
#else
    timer_spinlock_take(_timerGroup);
    timer_group_clr_intr_status_in_isr(_timerGroup, _timerIdx);    
    timer_group_enable_alarm_in_isr(_timerGroup, _timerIdx);
    timer_spinlock_give(_timerGroup);
#endif
}

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPTimer Alarm Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS

bool IRAM_ATTR RampGenTimer::_staticGPTimerCB(gptimer_t* timer, const gptimer_alarm_event_data_t* eventData, void* arg)
{
    // Check the arg is valid
    if (!arg)
        return false;
    RampGenTimer* pTimer = (RampGenTimer*)arg;
    pTimer->_nonStaticGPTimerCB();
    return false;
}

void IRAM_ATTR RampGenTimer::_nonStaticGPTimerCB()
{
#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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

#ifdef USE_SEMAPHORE_FOR_LIST_ACCESS
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

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Debug
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String RampGenTimer::getDebugStr()
{
    uint64_t timerCount = 0;
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
        gptimer_get_raw_count(_timerHandle, &timerCount);
#else
        timerCount = _timerCount;
#endif            
    char tmpStr[50];
    snprintf(tmpStr, sizeof(tmpStr), "%llu", timerCount);
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
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
    // Stop timer ISR
    gptimer_stop(_timerHandle);
#else
    // Stop timer ISR
    timer_disable_intr(_timerGroup, _timerIdx);
#endif
}

void RampGenTimer::reenableTimerInterrupts()
{
    // Check valid
    if (!_timerIsSetup)
        return;

    // Re-enable ISR
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
    if (_timerIsEnabled)
        gptimer_start(_timerHandle);
#else
    timer_enable_intr(_timerGroup, _timerIdx);
#endif
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
#ifdef RAMP_GEN_USE_ESP_IDF_GPTIMER_FUNCTIONS
    gptimer_set_raw_count(_timerHandle, 0);
#else
    _timerCount = 0;
#endif
}
