////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.h
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftArduino.h"
#include "RaftSysMod.h"
#include "RestAPIEndpointManager.h"

class MainSysMod : public RaftSysMod
{
public:
    MainSysMod(const char *pModuleName, RaftJsonIF& sysConfig);
    virtual ~MainSysMod();

    // Create function (for use by SysManager factory)
    static RaftSysMod* create(const char* pModuleName, RaftJsonIF& sysConfig)
    {
        return new MainSysMod(pModuleName, sysConfig);
    }

protected:

    // Setup
    virtual void setup() override final;

    // Loop (called frequently)
    virtual void loop() override final;

    // Get sensor angles (for access by other modules)
    float getMT6701Angle() const { return _mt6701Angle; }
    float getAS5600Angle() const { return _as5600Angle; }
    uint32_t getMT6701LastUpdateMs() const { return _mt6701LastUpdateMs; }
    uint32_t getAS5600LastUpdateMs() const { return _as5600LastUpdateMs; }

private:
    // Add REST API endpoints
    virtual void addRestAPIEndpoints(RestAPIEndpointManager &endpointManager) override final;

    // Control via API
    RaftRetCode apiControl(const String &reqStr, String &respStr, const APISourceInfo& sourceInfo);

    // Send command to MotorControl device
    bool sendToMotorControl(const String& cmdJSON, String& respStr);

    // Get motor device
    RaftDevice* getMotorDevice() const;

    // Register for sensor data
    void registerForSensorData();
    
    // Debug
    static constexpr const char *MODULE_PREFIX = "MainSysMod";

    // Example of how to control loop rate
    uint32_t _lastLoopMs = 0;

    // Sensor data from magnetic encoders
    float _mt6701Angle = 0.0f;
    float _as5600Angle = 0.0f;
    uint32_t _mt6701LastUpdateMs = 0;
    uint32_t _as5600LastUpdateMs = 0;

    // Decode states for sensor data
    RaftBusDeviceDecodeState _decodeStateMT6701;
    RaftBusDeviceDecodeState _decodeStateAS5600;
};
