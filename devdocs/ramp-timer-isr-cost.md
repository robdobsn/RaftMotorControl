# Ramp timer ISR: cost, the mutex taken from the ISR, and core placement

Proposed 2026-11-22. Not started. Not urgent: SandBot moves smoothly without
these changes.

## Background

While chasing erratic ball speed on SandBot (ESP32-S3, 240 MHz, 32 KB
I-cache), the main loop on core 0 measured 18–27 % slower while the motors
were stepping. The full investigation is in the SandBot repo:
`SandBotFirmware/devdocs/MAIN_LOOP_PERFORMANCE.md`.

The ramp generator timer (`RampGenTimer`, gptimer, 20 µs period = 50 kHz)
costs about 3–4 µs per tick while stepping. Putting the tick functions in
IRAM (`MOTOR_TICK_FN_DECORATOR`, `CONFIG_GPTIMER_ISR_CACHE_SAFE`) made no
measurable difference. So the cost is the ISR's own work, not flash-cache
misses.

## 1. A FreeRTOS mutex is taken on every tick

`RampGenTimer::_nonStaticGPTimerCB()` (RampGenTimer.h) calls
`xSemaphoreTakeFromISR(_hookListMutex, ...)` / `xSemaphoreGiveFromISR`
around the hook loop, 50,000 times a second. `_hookListMutex` is created with
`xSemaphoreCreateMutex()`. The FreeRTOS documentation says mutexes must not
be used from an ISR: priority inheritance needs a task context. It works in
practice, but it is undefined and costs time on every tick.

The list only changes when hooks are added or removed (`hookTimer` /
`unhookTimer`), which is rare and happens at setup.

**Proposals, simplest first:**

- Replace the mutex with a `portMUX_TYPE` spinlock: `portENTER_CRITICAL_ISR`
  in the ISR, `portENTER_CRITICAL` in hook/unhook.
- Or make the ISR lock-free. Keep a fixed array of hooks plus a count, and
  have hook/unhook disable the timer (or the alarm) while editing. An even
  simpler option: publish a new list pointer atomically.

Measure the per-tick time before and after (a GPIO toggle on a logic
analyser, or `esp_cpu_get_cycle_count()` sampled into a ring buffer).

## 2. Run the timer interrupt on core 1

A gptimer interrupt is allocated on the core that calls
`gptimer_register_event_callbacks` / `gptimer_enable`, which today is the
main loop's core (core 0). On SandBot, WiFi and lwIP now run on core 1, and
moving them there was a large win for the main loop. Putting the ramp
interrupt on core 1 as well would give core 0 back the remaining 18–27 %.

**Proposal:** an option (e.g. `"timerCore": 1` in the motion config) to
create and enable the timer from a short task pinned to that core.

**Needs testing:** step timing jitter with WiFi traffic on the same core, and
that anything the ISR shares with core 0 code is safely locked across cores
(the spinlock in item 1 already provides that).

## 3. Done in PR #4 (open, not yet merged)

`MotorControl::getNamedValue` read JSON config and copied `AxesParams` on
every call, and computed forward kinematics separately for `posX` and `posY`.
The S3's FPU is single precision only, so a double cos+sin pair costs about
25 µs at 240 MHz. PR #4 reads config only for the values that need it, caches the step
scale in `setup()`, matches names with `strcasecmp`, and computes FK once for
both values (cached for 1 ms).
