////////////////////////////////////////////////////////////////////////////////
//
// DriverStatus.h
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftCore.h"
#include "MotorControl.h"

class APISourceInfo;

class DriverStatus : public RaftSysMod
{
public:
    DriverStatus(const char *pModuleName, RaftJsonIF& sysConfig);
    virtual ~DriverStatus();

    // Create function (for use by SysManager factory)
    static RaftSysMod* create(const char* pModuleName, RaftJsonIF& sysConfig)
    {
        return new DriverStatus(pModuleName, sysConfig);
    }

protected:

    // Setup
    virtual void setup() override final;

    // Loop (called frequently)
    virtual void loop() override final;

    // Add REST API endpoints
    virtual void addRestAPIEndpoints(RestAPIEndpointManager &endpointManager) override final;

private:

    // API
    RaftRetCode apiControl(const String &reqStr, String &respStr, const APISourceInfo& sourceInfo);    

    // Example of how to control loop rate
    uint32_t _lastLoopMs = 0;

    // Stepper motors device
    RaftDevice* _pMotorControl = nullptr;

    // Positions
    int32_t _lastPosIdx = 0;
    std::vector<std::tuple<float, float>> _testPositions;
};
