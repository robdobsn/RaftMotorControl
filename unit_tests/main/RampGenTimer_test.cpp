/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Timer test
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <limits.h>
#include "RaftCore.h"
#include "RampGenTimer.h"
#include "unity.h"

static const char* MODULE_PREFIX = "RampGenTimerTest";

static volatile int localTimerCount = 0;
void MOTOR_TICK_FN_DECORATOR rampGenTimerCallback(void* pObject)
{
    localTimerCount = localTimerCount + 1;
}

TEST_CASE("test_RampGenTimer", "[RampGenTimer]")
{
    // Debug
    LOG_I(MODULE_PREFIX, "RampGenTimer Test");

    RampGenTimer rampGenTimer;
    const int timerPeriodUs = 100;
    rampGenTimer.setup(timerPeriodUs);
    rampGenTimer.enable(true);

    // Test raw count changes - note that the raw count may reset to 0 around every
    // timerPeriodUs microseconds
    uint64_t rawCount = rampGenTimer.getDebugRawCount();
    delayMicroseconds((int)(timerPeriodUs*1.23));
    uint64_t rawCount2 = rampGenTimer.getDebugRawCount();
    delayMicroseconds((int)(timerPeriodUs*1.63));
    uint64_t rawCount3 = rampGenTimer.getDebugRawCount();
    TEST_ASSERT((rawCount2 != rawCount) || (rawCount3 != rawCount2));

    // Attach hook
    rampGenTimer.hookTimer(rampGenTimerCallback, (void*)&rampGenTimerCallback);

    // Test timer counts up
    int lastCallbackCount = -1;
    const int timePeriodForTestLoopMs = 100;
    const int expectedIncrementPerLoop = (timePeriodForTestLoopMs*1000)/timerPeriodUs;
    const int errorMargin = expectedIncrementPerLoop / 10;
    for (int i = 0; i < 5; i++)
    {
        delay(timePeriodForTestLoopMs);
        if (lastCallbackCount == -1)
            lastCallbackCount = localTimerCount;
        else
        {
            int expectedApproxCount = lastCallbackCount + expectedIncrementPerLoop;
            TEST_ASSERT_DOUBLE_WITHIN(localTimerCount, expectedApproxCount - errorMargin, expectedApproxCount + errorMargin);
            lastCallbackCount = localTimerCount;
        }
        LOG_I(MODULE_PREFIX, "localTimerCount %d expected %d errorMargin %d", localTimerCount, expectedIncrementPerLoop, errorMargin);
    }

    // Unattach hook
    rampGenTimer.unhookTimer((void*)&rampGenTimerCallback);
}
