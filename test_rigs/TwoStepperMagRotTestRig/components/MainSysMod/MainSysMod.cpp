////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "MainSysMod.h"

MainSysMod::MainSysMod(const char *pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
}

MainSysMod::~MainSysMod()
{
}

void MainSysMod::setup()
{
    // The following code is an example of how to use the config object to
    // get a parameter from SysType (JSON) file for this system module
    // Replace this with your own setup code
    String configValue = config.getString("exampleGroup/exampleKey", "This Should Not Happen!");
    LOG_I(MODULE_PREFIX, "%s", configValue.c_str());
}

void MainSysMod::loop()
{
    // Check for loop rate
    if (Raft::isTimeout(millis(), _lastLoopMs, 1000))
    {
        // Update last loop time
        _lastLoopMs = millis();

        // Put some code here that will be executed once per second
        // ...
    }
}

void MainSysMod::addRestAPIEndpoints(RestAPIEndpointManager &endpointManager)
{
    // Control shade
    endpointManager.addEndpoint("motors", RestAPIEndpoint::ENDPOINT_CALLBACK, RestAPIEndpoint::ENDPOINT_GET,
                            std::bind(&MainSysMod::apiControl, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                            "Control Motors - e.g. motors/setPos?a0=1000&a1=-1000");
}

RaftRetCode MainSysMod::apiControl(const String &reqStr, String &respStr, const APISourceInfo& sourceInfo)
{
    // Extract params
    RaftJson requestAsJSON = RestAPIEndpointManager::getJSONFromRESTRequest(reqStr.c_str());

    // Handle commands
    bool rslt = false;
    String rsltStr = "Failed";
    LOG_I(MODULE_PREFIX, "apiControl: requestAsJSON %s", requestAsJSON.toString().c_str());
    String motorCmdJSON = "";
    String command = requestAsJSON.getString("pathSegments[1]", "");
    if (!command.isEmpty())
    {
        if (command.equalsIgnoreCase("setPos"))
        {
            // Get steps for stepper motors
            double axis0Pos = requestAsJSON.getDouble("params/a0", 0);
            double axis1Pos = requestAsJSON.getDouble("params/a1", 0);
            bool unitsAreSteps = requestAsJSON.getBool("params/unitsAreSteps", false);
            LOG_I(MODULE_PREFIX, "apiControl: setPos axis0Pos %f axis1Pos %f unitsAreSteps %d", axis0Pos, axis1Pos, unitsAreSteps);

            // Form the JSON command to send to MotorControl
            motorCmdJSON = R"({"cmd":"motion","stop":1,"clearQ":1,"rel":0,"nosplit":1,"steps":__STEPS__,"feedrate":__SPEED__,"pos":[{"a":0,"p":__POS0__},{"a":1,"p":__POS1__}]})";
            motorCmdJSON.replace("__STEPS__", unitsAreSteps ? "1" : "0");
            motorCmdJSON.replace("__SPEED__", "1");
            motorCmdJSON.replace("__POS0__", String(axis0Pos));
            motorCmdJSON.replace("__POS1__", String(axis1Pos));
            // Debug
            LOG_I(MODULE_PREFIX, "apiControl: setPos motorCmd %s (from params %s)", motorCmdJSON.c_str(), requestAsJSON.toString().c_str());
        }
        else if (command.equalsIgnoreCase("stop"))
        {
            // Form the JSON command to send to MotorControl
            motorCmdJSON = R"({"cmd":"motion","stop":1,"clearQ":1})";

            // Debug
            LOG_I(MODULE_PREFIX, "apiControl: stop motorCmd %s (from params %s)", motorCmdJSON.c_str(), requestAsJSON.toString().c_str());
        }
        else if (command.equalsIgnoreCase("disable"))
        {
            // Form the JSON command to send to MotorControl
            motorCmdJSON = R"({"cmd":"motion", "en":0})";

            // Debug
            LOG_I(MODULE_PREFIX, "apiControl: disableMotors motorCmd %s (from params %s)", motorCmdJSON.c_str(), requestAsJSON.toString().c_str());
        }
        else if (command.equalsIgnoreCase("setOrigin"))
        {
            // Form the JSON command to send to MotorControl
            motorCmdJSON = R"({"cmd":"setOrigin"})";

            // Debug
            LOG_I(MODULE_PREFIX, "apiControl: setOrigin motorCmd %s (from params %s)", motorCmdJSON.c_str(), requestAsJSON.toString().c_str());
        }
        else
        {
            rsltStr = "Unknown command";
            rslt = false;
            LOG_E(MODULE_PREFIX, "apiControl: UNKNOWN command %s", command.c_str());
            return Raft::setJsonErrorResult(reqStr.c_str(), respStr, rsltStr.c_str());
        }
    }

    // Send to MotorControl device
    rslt = sendToMotorControl(motorCmdJSON, respStr);

    // Result
    if (rslt)
    {
        LOG_I(MODULE_PREFIX, "apiControl: reqStr %s rslt %s", reqStr.c_str(), rsltStr.c_str());
        return Raft::setJsonBoolResult(reqStr.c_str(), respStr, true);
    }
    LOG_E(MODULE_PREFIX, "apiControl: FAILED reqStr %s rslt %s", reqStr.c_str(), rsltStr.c_str());
    return Raft::setJsonErrorResult(reqStr.c_str(), respStr, rsltStr.c_str());
}

bool MainSysMod::sendToMotorControl(const String& cmdJSON, String& respStr)
{
    // Get MotorControl device
    RaftDevice* pMotorControl = getMotorDevice();
    if (!pMotorControl)
    {
        respStr = "{\"rslt\":\"failed\",\"error\":\"No MotorControl device found\"}";
        return false;
    }

    // Send command to MotorControl device
    String errorMsg;
    RaftRetCode rc = pMotorControl->sendCmdJSON(cmdJSON.c_str(), &errorMsg);
    if (rc == RaftRetCode::RAFT_OK)
    {
        respStr = "{\"rslt\":\"ok\"}";
        return true;
    }
    else
    {
        // Use getRetCodeStr for consistent, short error messages
        String errorMsg = Raft::getRetCodeStr(rc);
        respStr = "{\"rslt\":\"fail\",\"error\":\"" + errorMsg + "\"}";
        return false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get motor device
RaftDevice* MainSysMod::getMotorDevice() const
{
    // Get motor device
    auto pDevMan = getSysManager()->getDeviceManager();
    if (!pDevMan)
        return nullptr;
    RaftDevice* pMotor = pDevMan->getDevice("MotorControl");
    if (!pMotor)
        return nullptr;
    return pMotor;
}
