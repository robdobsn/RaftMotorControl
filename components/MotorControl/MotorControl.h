/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motor Control
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftRetCode.h"
#include "RaftDevice.h"
#include "MotionController.h"
#include "RaftBus.h"

class MotorControl : public RaftDevice
{
public:
    /// @brief Constructor
    /// @param pClassName device class name
    /// @param pDevConfigJson device configuration JSON
    MotorControl(const char* pClassName, const char *pDevConfigJson);

    /// @brief Destructor
    virtual ~MotorControl();

    // @brief Setup the device
    virtual void setup() override final;

    // @brief Main loop for the device (called frequently)
    virtual void loop() override final;
    
    /// @brief Get binary data from the device
    /// @param formatCode format code for the command
    /// @param buf (out) buffer to receive the binary values
    /// @param bufMaxLen maximum length of data to return
    /// @return RaftRetCode
    virtual RaftRetCode getDataBinary(uint32_t formatCode, std::vector<uint8_t>& buf, uint32_t bufMaxLen) const override final;

    /// @brief Get named value from the device
    /// @param pParam Parameter name
    /// @param isFresh (out) true if the value is fresh
    /// @return double value
    virtual double getNamedValue(const char* pParam, bool& isFresh) const override final;

    // Send binary command
    virtual RaftRetCode sendCmdBinary(uint32_t formatCode, const uint8_t* pData, uint32_t dataLen) override final;

    // Send JSON command
    virtual RaftRetCode sendCmdJSON(const char* jsonCmd) override final;

    // Has capability
    virtual bool hasCapability(const char* pCapabilityStr) const override final;
 
    /// @brief Create function for device factory
    /// @param pClassName device class name
    /// @param pDevConfigJson device configuration JSON
    /// @return RaftDevice* pointer to the created device
    static RaftDevice* create(const char* pClassName, const char* pDevConfigJson)
    {
        return new MotorControl(pClassName, pDevConfigJson);
    }

    /// @brief Get JSON data from the device
    /// @param level Level of data to return
    /// @return JSON string
    virtual String getDataJSON(RaftDeviceJSONLevel level = DEVICE_JSON_LEVEL_MIN) const override final;

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

    // Motor serial bus
    RaftBus* _pMotorSerialBus = nullptr;

    // Command handlers
    void handleCmdBinary_MoveTo(const uint8_t* pData, uint32_t dataLen);
};
