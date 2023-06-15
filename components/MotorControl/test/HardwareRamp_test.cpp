/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// HardwareRamp test
//
// Rob Dobson 2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Logger.h>
#include <MotorControl.h>
#include <BusSerial.h>
#include <ArduinoOrAlt.h>
#include <limits.h>
#include "unity.h"

static const char* MODULE_PREFIX = "MotorControlTest";

ConfigBase testConfig = R"(
    "bus":
    {
        "name": "SERA",
        "type": "serial",
        "uartNum": 2,
        "rxPin": 5,
        "txPin": 35,
        "baudRate": 115200,
        "minAfterSendMs": 5
    },
    "stepper":
    {
        "name": "TestMotor",
        "type": "Steppers",
        "bus": "SERA",
        "motion": {
            "geom": "XYZ",
            "blockDist": 1,
            "allowOutOfBounds": 0,
            "homeBeforeMove": 0,
            "junctionDeviation": 0.05
        },
        "ramp": {
            "rampTimerEn": 1,
            "rampTimerUs": 100,
            "pipelineLen": 100
        },
        "motorEn": {
            "stepEnablePin": 4,
            "stepEnLev": 0,
            "stepDisableSecs": 1
        },
        "axes": [
            {
                "name": "X",
                "params": {
                    "unitsPerRot": 1,
                    "stepsPerRot": 200,
                    "maxSpeed": 50,
                    "maxAcc": 50
                },
                "driver": {
                    "driver": "TMC2209",
                    "hw": "local",
                    "addr": 0,
                    "stepPin": "15",
                    "dirnPin": "16",
                    "invDirn": 0,
                    "writeOnly": 0,
                    "extSenseOhms": 0.15,
                    "extVRef": 0,
                    "extMStep": 0,
                    "intpol": 1,
                    "microsteps": 256,
                    "rmsAmps": 0.01,
                    "holdModeOrFactor": 0.5,
                    "holdDelay": 1,
                    "pwmFreqKHz": 35
                }
            }
        ]
    }
)";

TEST_CASE("test_MotorControl", "[MotorControl]")
{
    // Debug
    LOG_I(MODULE_PREFIX, "MotorControl Test");

    // // Setup serial bus for config of motor
    BusSerial busSerial(nullptr, nullptr);
    busSerial.setup(testConfig, "bus");

    // // Set up motor control
    MotorControl motorControl;
    motorControl.setup(testConfig, nullptr, "stepper");
    motorControl.setBusNameIfValid(busSerial.getBusName().c_str());
    motorControl.connectToBus(&busSerial);
    motorControl.postSetup();

    // Get motor position
    String motorInfo = motorControl.getDebugStr();
    LOG_I(MODULE_PREFIX, "MotorInfo: %s", motorInfo.c_str());

    // Wait
    const int MAX_COUNT = 3000;
    for (int progressCount = 0; progressCount < MAX_COUNT; progressCount++)
    {
        delay(1);
        busSerial.service();
        motorControl.service();
        if (progressCount == 100)
        {
            // Motion command
            String moveCmd = R"({"cmd":"motion","rel":1,"nosplit":1,"speed":10,"speedOk":1,"pos":[{"a":0,"p":10}]})";
            LOG_I(MODULE_PREFIX, "Start move X to 1000");
            motorControl.sendCmdJSON(moveCmd.c_str());
        }
        if ((progressCount % 100 == 0) || (progressCount == MAX_COUNT - 1))
        {
            String motorInfo = motorControl.getDebugStr();
            LOG_I(MODULE_PREFIX, "MotorInfo: %s", motorInfo.c_str());
        }
    }

    // Check
    String motorInfo2 = motorControl.getDebugStr();
    LOG_I(MODULE_PREFIX, "MotorInfo: %s", motorInfo2.c_str());

}
