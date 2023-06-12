/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Timer test
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Logger.h>
#include <RampGenTimer.h>
#include <ArduinoOrAlt.h>
#include <limits.h>
#include "unity.h"

static const char* MODULE_PREFIX = "RampGenTimerTest";

static volatile int localTimerCount = 0;
void IRAM_ATTR rampGenTimerCallback(void* pObject)
{
    localTimerCount = localTimerCount + 1;
}

TEST_CASE("test_RampGenTimer", "[RampGenTimer]")
{
    // Debug
    LOG_I(MODULE_PREFIX, "RampGenTimer Test");

    RampGenTimer rampGenTimer;
    rampGenTimer.setup(20000);
    rampGenTimer.enable(true);

    // Test timer counts up
    int64_t lastTimerCount = -1;
    for (int i = 0; i < 5; i++)
    {
        delay(1);
        if (lastTimerCount == -1)
            lastTimerCount = rampGenTimer.getTimerCount();
        else
        {
            int64_t timerCount = rampGenTimer.getTimerCount();
            TEST_ASSERT_NOT_EQUAL(lastTimerCount, timerCount);
            lastTimerCount = timerCount;
        }
    }

    // Attach hook
    rampGenTimer.hookTimer(rampGenTimerCallback, (void*)&rampGenTimerCallback);

    // Test timer counts up
    int lastTimerCount2 = -1;
    for (int i = 0; i < 5; i++)
    {
        delay(100);
        if (lastTimerCount2 == -1)
            lastTimerCount2 = localTimerCount;
        else
        {
            TEST_ASSERT_NOT_EQUAL(lastTimerCount2, localTimerCount);
            lastTimerCount2 = localTimerCount;
        }
        // LOG_I(MODULE_PREFIX, "localTimerCount %d", localTimerCount);
    }

    // Unattach hook
    rampGenTimer.unhookTimer((void*)&rampGenTimerCallback);
}
