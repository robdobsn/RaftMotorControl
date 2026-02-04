/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Unit tests of RampGenerator
//
// Rob Dobson 2017-2021
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "unity.h"
#include "StepDriverTMC2209.h"
#include "RaftBusSystem.h"
#include "BusSerial.h"
// #include "RampGenerator.h"
// #include "RampGenTimer.h"
// #include "MotionPipeline.h"
// #include "MotionBlock.h"
// #include "EndStops.h"

static const char* MODULE_PREFIX = "TMC2209DriverTest";

// #define TEST_USING_TIMER_ISR
// #define TEST_PERFORMANCE_ONLY

// static const char* TMC2209_TEST_CONFIG_JSON =
// {
// #ifdef TEST_USING_TIMER_ISR    
//     R"("timerIntr":1,)"
// #else
//     R"("timerIntr":0,)"
// #endif
//     R"("axes":[)"
//         R"({)"
//             R"("name":"X",)"
//             R"("params":)"
//             R"({)"
//                 R"("maxSpeed":75,)"
//                 R"("maxAcc":"100",)"
//                 R"("maxRPM":"600",)"
//                 R"("stepsPerRot":"1000",)"
//                 R"("maxVal":"100")"
//             R"(},)"
//             R"("driver":)"
//             R"({)"            
//                 R"("driver":"TMC2209",)"
//                 R"("hw":"local",)"
//                 R"("stepPin":"15",)"
//                 R"("dirnPin":"12",)"
//                 R"("invDirn":0)"
//             R"(})"
//         R"(})"
//     R"(])"
// };

void busStatusCB(RaftBus& bus, const std::vector<BusElemAddrAndStatus>& statusChanges)
{
    LOG_I(MODULE_PREFIX, "busStatusCB");
}

void busOperationStatusCB(RaftBus& bus, BusOperationStatus busOperationStatus)
{
    LOG_I(MODULE_PREFIX, "busOperationStatusCB");
}

TEST_CASE("test_TMC2209", "[TMC2209Test]")
{
    // Debug
    LOG_I(MODULE_PREFIX, "TMC2209 Test");

    // Serial bus
    raftBusSystem.registerBus("serial", BusSerial::createFn);
    raftBusSystem.setup("buses", 
            RaftJson(R"({"buses":{"buslist":[{"type":"serial","name":"SERA","uartNum":2,"rxPin":5,"txPin":47,"baudRate":115200}]}})"),
            busStatusCB, busOperationStatusCB);
    RaftBus* pMotorSerialBus = raftBusSystem.getBusByName("SERA");
    if (pMotorSerialBus == NULL)
    {
        LOG_E(MODULE_PREFIX, "Failed to get serial bus");
        return;
    }
    // RaftJson busConfig(R"({"name":"SERA","uartNum":2,"rxPin":5,"txPin":47,"baudRate":115200})");
    // pMotorSerialBus->setup(busConfig);

    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    // Steppers
    StepDriverBase* pStepper1 = new StepDriverTMC2209();
    StepDriverBase* pStepper2 = new StepDriverTMC2209();

    // Bus setup
    pStepper1->setupSerialBus(pMotorSerialBus, false);
    pStepper2->setupSerialBus(pMotorSerialBus, false);

    // Params
    StepDriverParams stepperParams;
    stepperParams.stepPin = 15;
    stepperParams.dirnPin = 16;
    stepperParams.rmsAmps = 0.1;
    stepperParams.microsteps = 16;
    pStepper1->setup("Ax1", stepperParams, false);
    stepperParams.stepPin = 17;
    stepperParams.dirnPin = 18;
    stepperParams.address = 1;
    pStepper2->setup("Ax2", stepperParams, false);

    // Loop to allow comms
    for (int i = 0; i < 500; i++)
    {
        pStepper1->loop();
        pStepper2->loop();
        delay(1);
    }

//     // End stops
//     std::vector<EndStops*> endStops;

//     // Axes
//     AxesParams axesParams;
//     axesParams.setupAxes(RAMP_GEN_TEST_STEPPER_JSON);
//     MotionPipeline motionPipeline;
//     motionPipeline.setup(10);
//     RampGenTimer rampGenTimer;

//     RampGenerator rampGen(motionPipeline, rampGenTimer, steppers, endStops);
//     rampGen.setup(RAMP_GEN_TEST_STEPPER_JSON);
//     rampGen.enable(true);
//     rampGen.pause(false);

//     LOG_I(MODULE_PREFIX, "TESTING ..........................")
// //     MotionBlock motionBlock;
// //     // Set tick time
// //     motionBlock.setTimerPeriodNs(rampGenTimer.getPeriodUs() * 1000);
// //     // Requested speed in mm/s
// //     motionBlock._requestedSpeed = 100;
// //     // Move distance
// //     motionBlock._moveDistPrimaryAxesMM = 10;
// //     // Primary axis vector
// //     motionBlock._unitVecAxisWithMaxDist = 1;
// //     // Max extry speed mm/s
// //     motionBlock._maxEntrySpeedMMps = 100;
// //     // Actual entry speed mm/s
// //     motionBlock._entrySpeedMMps = 0;
// //     // Actual exit speed mm/s
// //     motionBlock._exitSpeedMMps = 0;
// //     // No end stops for now
// //     motionBlock._endStopsToCheck.clear();
// //     // Keeps track of block
// //     motionBlock._motionTrackingIndex = 1234;
// //     // Block is stand-alone
// //     motionBlock._blockIsFollowed = false;
// //     // Set can execute flag
// //     motionBlock._canExecute = true;
// //     // Set the motion steps
// //     motionBlock.setStepsToTarget(0, 10000);
// //     motionBlock.prepareForStepping(axesParams, false);
// //     motionBlock.debugShowTimingConsts();
// //     motionBlock.debugShowBlkHead();
// //     motionBlock.debugShowBlock(0, axesParams);

// //     motionPipeline.add(motionBlock);

// // #ifdef TEST_USING_TIMER_ISR
// //     rampGenTimer.setup(20, TIMER_GROUP_0, TIMER_0);
// //     rampGenTimer.enable(true);
// //     delayMicroseconds(2000000);
// //     rampGenTimer.enable(false);
// // #else
// //     for (uint64_t i = 0; i < 1000000; i++)
// //     {
// //         rampGen.service();
// //     }
// // #endif

// //     // uint32_t minTicksPerStep = 0;
// //     // uint32_t earlyTicksPerStep = 0;
// //     // uint32_t lateTicksPerStep = 0;
// //     // uint32_t ticksToCompletion = 0;
// //     // std::vector<StepInfo>& stepInfo = rampGenIOTest.getTestResults(minTicksPerStep, 
// //     //                     earlyTicksPerStep, lateTicksPerStep, ticksToCompletion);
// //     // LOG_I(MODULE_PREFIX, "Steps found %d, maxSpeed (ticksPerStep) %d earlySpeed %d lateSpeed %d ticksToCompletion %d",
// //     //                     stepInfo.size(), minTicksPerStep, earlyTicksPerStep, lateTicksPerStep, ticksToCompletion);
// //     // for (auto& step : stepInfo)
// //     // {
// //     //     LOG_I(MODULE_PREFIX, "ticks %lld ticksPerStep %d", 
// //     //         step.timerCount, step.ticksPerStep);
// //     // }

// //     rampGen.debugShowStats();

// //     // TEST_ASSERT_MESSAGE (std::abs(int(minTicksPerStep - 5)) < 2, "Invalid max speed (min ticks per step)");
// //     // TEST_ASSERT_MESSAGE (std::abs(int(earlyTicksPerStep - 90)) < 10, "Invalid step rate early in the move");
// //     // TEST_ASSERT_MESSAGE (std::abs(int(lateTicksPerStep - 36)) < 10, "Invalid step rate towards end of move");
}
