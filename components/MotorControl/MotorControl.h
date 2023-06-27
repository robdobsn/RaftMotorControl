/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motor Control
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <RaftRetCode.h>
#include <HWElemBase.h>
#include <Controller/MotionController.h>
// #include "RampGenerator/RampGenTimer.h"
// #include "EvaluatorGCode.h"

class ConfigBase;

class MotorControl : public HWElemBase
{
public:
    // Constructor/Destructor
    MotorControl();
    virtual ~MotorControl();

    // Setup
    virtual void setup(ConfigBase& config, ConfigBase* pDefaults, const char* pConfigPrefix) override final;

    // Post-Setup - called after any buses have been connected
    virtual void postSetup() override final;

    // Service
    virtual void service() override final;
    
    // Get values binary = format specific to hardware
    virtual uint32_t getValsBinary(uint32_t formatCode, uint8_t* pBuf, uint32_t bufMaxLen) override final;
    virtual double getNamedValue(const char* param, bool& isFresh) override final;

    // Send binary command
    virtual RaftRetCode sendCmdBinary(uint32_t formatCode, const uint8_t* pData, uint32_t dataLen) override final;

    // Send JSON command
    virtual RaftRetCode sendCmdJSON(const char* jsonCmd) override final;

    // Has capability
    virtual bool hasCapability(const char* pCapabilityStr) override final;
 
     // Creator fn
    static HWElemBase* createFn()
    {
        return new MotorControl();
    }

    // Get data
    virtual String getDataJSON(HWElemStatusLevel_t level = ELEM_STATUS_LEVEL_MIN) override final;

    // Set motor on time after move
    void setMotorOnTimeAfterMoveSecs(float motorOnTimeAfterMoveSecs)
    {
        _motionController.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
    }

    // Set max motor current (amps)
    void setMaxMotorCurrentAmps(uint32_t axisIdx, float maxMotorCurrent)
    {
        _motionController.setMaxMotorCurrentAmps(axisIdx, maxMotorCurrent);
    }

    // Get debug str
    String getDebugStr();

private:
    // Motion controller
    MotionController _motionController;

    // Command handlers
    void handleCmdBinary_MoveTo(const uint8_t* pData, uint32_t dataLen);

};
