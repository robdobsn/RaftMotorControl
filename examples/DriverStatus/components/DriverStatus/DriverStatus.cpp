////////////////////////////////////////////////////////////////////////////////
//
// DriverStatus.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "DeviceManager.h"
#include "DriverStatus.h"
#include "RaftUtils.h"
#include "SysManager.h"

static const char *MODULE_PREFIX = "DriverStatus";

DriverStatus::DriverStatus(const char *pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
    _testPositions.push_back({100,100});
    _testPositions.push_back({-100,-100});
    _testPositions.push_back({0,0});
}

DriverStatus::~DriverStatus()
{
}

void DriverStatus::addRestAPIEndpoints(RestAPIEndpointManager &endpointManager)
{
    // Move
    endpointManager.addEndpoint("m", RestAPIEndpoint::ENDPOINT_CALLBACK, RestAPIEndpoint::ENDPOINT_GET,
                std::bind(&DriverStatus::apiControl, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                "m", "Move stepper motors");
    LOG_I(MODULE_PREFIX, "addRestAPIEndpoints move");
}

RaftRetCode DriverStatus::apiControl(const String &reqStr, String &respStr, const APISourceInfo& sourceInfo)
{
    bool isFresh = false;
    if (_pMotorControl->getNamedValue("b", isFresh) || !isFresh)
    {
        LOG_I(MODULE_PREFIX, "Stepper motors not ready");
        return Raft::setJsonErrorResult(reqStr.c_str(), respStr, "failNotReady");
    }

    float speed = 10;
    String moveCmd = R"({"cmd":"motion","stop":1,"clearQ":1,"rel":0,"nosplit":0,"speed":__SPEED__,"speedOk":1,"pos":[{"a":0,"p":__POS0__},{"a":1,"p":__POS1__}]})";
    moveCmd.replace("__POS0__", String(std::get<0>(_testPositions[_lastPosIdx])));
    moveCmd.replace("__POS1__", String(std::get<1>(_testPositions[_lastPosIdx])));
    moveCmd.replace("__SPEED__", String(speed));

    // Send command
    _pMotorControl->sendCmdJSON(moveCmd.c_str());

    LOG_I(MODULE_PREFIX, "moveCmd %s", moveCmd.c_str());
    _lastPosIdx = (_lastPosIdx + 1) % _testPositions.size();

    // Return success
    return Raft::setJsonBoolResult(reqStr.c_str(), respStr, true);
}

void DriverStatus::setup()
{
    // Get the stepper motors device
    DeviceManager* pDeviceManager = getSysManager()->getDeviceManager();
    if (!pDeviceManager)
    {
        LOG_E(MODULE_PREFIX, "No device manager");
        return;
    }
    _pMotorControl = pDeviceManager->getDevice("Motors");

    LOG_I(MODULE_PREFIX, "setup, device manager %p, motors %p", pDeviceManager, _pMotorControl);

}

void DriverStatus::loop()
{
    // Check for loop rate
    if (Raft::isTimeout(millis(), _lastLoopMs, 1000))
    {
        // Update last loop time
    _lastLoopMs = millis();

        // Check status of driver
        if (!_pMotorControl)
        {
            LOG_E(MODULE_PREFIX, "No motor control device");
            return;
        }
        LOG_I(MODULE_PREFIX, "Driver status: %s", _pMotorControl->getDataJSON(RaftDeviceJSONLevel::DEVICE_JSON_LEVEL_FULL).c_str());
    }
}
