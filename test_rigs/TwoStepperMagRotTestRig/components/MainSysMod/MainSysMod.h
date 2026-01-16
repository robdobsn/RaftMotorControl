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

private:
    // Add REST API endpoints
    virtual void addRestAPIEndpoints(RestAPIEndpointManager &endpointManager) override final;

    // Control via API
    RaftRetCode apiControl(const String &reqStr, String &respStr, const APISourceInfo& sourceInfo);

    // Send command to MotorControl device
    bool sendToMotorControl(const String& cmdJSON, String& respStr);

    // Get motor device
    RaftDevice* getMotorDevice() const;
    
    // Debug
    static constexpr const char *MODULE_PREFIX = "MainSysMod";

    // Example of how to control loop rate
    uint32_t _lastLoopMs = 0;
};
