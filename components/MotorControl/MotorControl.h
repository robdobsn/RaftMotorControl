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

    /// @brief Get named value from the device
    /// @param pParam Parameter name
    /// @param isFresh (out) true if the value is fresh
    /// @return double value
    virtual double getNamedValue(const char* pParam, bool& isFresh) const override final;

    // Send JSON command
    virtual RaftRetCode sendCmdJSON(const char* jsonCmd) override final;

    // Send JSON command with error message
    virtual RaftRetCode sendCmdJSON(const char* jsonCmd, String* respMsg) override final;

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

    /// @brief Get the device status as JSON (for DeviceManager aggregation)
    /// @return JSON string with motor status
    virtual String getStatusJSON() const override final;

    /// @brief Get a hash value representing the current device state for change detection
    /// @return Hash value based on step counts and status flags
    virtual uint32_t getDeviceStateHash() const override final;

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

    // @brief Get device debug info JSON
    // @return JSON string
    virtual String getDebugJSON(bool includeBraces) const override final;

private:
    // Motion controller
    MotionController _motionController;

    // Motor serial bus
    RaftBus* _pMotorSerialBus = nullptr;

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotorControl";    
};
