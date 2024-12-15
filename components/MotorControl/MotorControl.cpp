/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motor Control
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "RaftJsonPrefixed.h"
#include "RaftBusSystem.h"
#include "MotorControl.h"

// #define DEBUG_MOTOR_CMD_JSON

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
/// @param pClassName device class name
/// @param pDevConfigJson device configuration JSON
MotorControl::MotorControl(const char* pClassName, const char *pDevConfigJson)
        : RaftDevice(pClassName, pDevConfigJson)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
MotorControl::~MotorControl()
{
    // Tell motion controller to stop
    _motionController.deinit();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup the device
void MotorControl::setup()
{
    // Setup motion controller
    _motionController.setup(deviceConfig);

    // Setup serial bus
    String serialBusName = deviceConfig.getString("bus", "");
    _pMotorSerialBus = raftBusSystem.getBusByName(serialBusName);
    _motionController.setupSerialBus(_pMotorSerialBus, false); 

    // Debug
    LOG_I(MODULE_PREFIX, "setup type %s serialBusName %s%s", 
            deviceClassName.c_str(), serialBusName.c_str(),
            _pMotorSerialBus ? "" : " (BUS INVALID)");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Main loop for the device (called frequently)
void MotorControl::loop()
{
    // Loop motion controller
    _motionController.loop();

    // Check if ready to record status
    if (!Raft::isTimeout(millis(), _readLastMs, _recordStatusMs))
        return;
    _readLastMs = millis();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Check if device has capability
/// @param pCapabilityStr capability string
/// @return true if the device has the capability
bool MotorControl::hasCapability(const char* pCapabilityStr) const
{
    switch(pCapabilityStr[0])
    {
        // Streaming outbound
        case 's': return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get JSON data from the device
/// @param level Level of data to return
/// @return JSON string
String MotorControl::getDataJSON(RaftDeviceJSONLevel level) const
{
    // Get data
    return _motionController.getDataJSON(level);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get named value from the device
/// @param pParam Parameter name
/// @param isFresh (out) true if the value is fresh
/// @return double value
double MotorControl::getNamedValue(const char* param, bool& isFresh) const
{
    switch(tolower(param[0]))
    {
        case 'x':
        case 'y':
        case 'z':
        {
            // Get axis position
            isFresh = true;
            AxesValues<AxisPosDataType> pos;
            AxesValues<AxisStepsDataType> actuatorPos;
            _motionController.getLastMonitoredPos(actuatorPos, pos);
            switch(tolower(param[0]))
            {
                case 'x': return pos.getVal(0);
                case 'y': return pos.getVal(1);
                case 'z': return pos.getVal(2);
            }
            isFresh = false;
            return 0;
        }
        case 'b':
        {
            // Check for busy
            isFresh = true;
            return _motionController.isBusy();
            break;
        }
        default: { isFresh = false; return 0; }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Send a JSON command to the device
/// @param jsonCmd JSON command
/// @return RaftRetCode
/// - RAFT_OK if the motion was successfully added to the pipeline
/// - RAFT_BUSY if the pipeline is full
/// - RAFT_INVALID_DATA if geometry not set
/// - RAFT_INVALID_OPERATION if homing is needed
/// - RAFT_CANNOT_START if no movement
/// - RAFT_NOT_IMPLEMENTED if the command is not implemented
RaftRetCode MotorControl::sendCmdJSON(const char* cmdJSON)
{
    RaftRetCode retCode = RAFT_NOT_IMPLEMENTED;
    // Extract command from JSON
    RaftJson jsonInfo(cmdJSON);
    String cmd = jsonInfo.getString("cmd", "");
    if (cmd.equalsIgnoreCase("motion"))
    {
        MotionArgs motionArgs;
        motionArgs.fromJSON(cmdJSON);
#ifdef DEBUG_MOTOR_CMD_JSON
        String cmdStr = motionArgs.toJSON();
        LOG_I(MODULE_PREFIX, "sendCmdJSON %s", cmdStr.c_str());
#endif
        retCode = _motionController.moveTo(motionArgs);
    }
    else if (cmd.equalsIgnoreCase("maxCurrent"))
    {
        float maxCurrentA = jsonInfo.getDouble("maxCurrentA", 0);
        uint32_t axisIdx = jsonInfo.getInt("axisIdx", 0);
        retCode = _motionController.setMaxMotorCurrentAmps(axisIdx, maxCurrentA);
    }
    else if (cmd.equalsIgnoreCase("offAfter"))
    {
        float motorOnTimeAfterMoveSecs = jsonInfo.getDouble("offAfterS", 0);
        retCode = _motionController.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
    }
    return retCode;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get debug string
/// @return Debug string
String MotorControl::getDebugJSON(bool includeBraces) const
{
    return _motionController.getDebugJSON(includeBraces);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Get time of last device status update
/// @param includeElemOnlineStatusChanges Include element online status changes in the status update time
/// @param includePollDataUpdates Include poll data updates in the status update time
/// @return Time of last device status update in milliseconds
uint32_t MotorControl::getDeviceInfoTimestampMs(bool includeElemOnlineStatusChanges, bool includePollDataUpdates) const
{
    if (includePollDataUpdates)
        return _readLastMs;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Get the device status as JSON
/// @return JSON string
String MotorControl::getStatusJSON() const
{
    // Buffer
    std::vector<uint8_t> data;
    formDeviceDataResponse(data);
 
    // Return JSON
    return "{\"0\":{\"x\":\"" + Raft::getHexStr(data.data(), data.size()) + "\",\"_t\":\"" + getPublishDeviceType() + "\"}}";
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Form device data response
/// @param data (out) Data buffer
void MotorControl::formDeviceDataResponse(std::vector<uint8_t>& data) const
{
    // Get average and store in format expected by conversion function
    uint16_t timeVal = (uint16_t)(_readLastMs & 0xFFFF);
    data.push_back((timeVal >> 8) & 0xFF);
    data.push_back(timeVal & 0xFF);

    // Get position
    AxesValues<AxisStepsDataType> actuatorPos;
    AxesValues<AxisPosDataType> realWorldPos;
    _motionController.getLastMonitoredPos(actuatorPos, realWorldPos);
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        int32_t posVal = actuatorPos.getVal(axisIdx);
        data.push_back((posVal >> 24) & 0xFF);
        data.push_back((posVal >> 16) & 0xFF);
        data.push_back((posVal >> 8) & 0xFF);
        data.push_back(posVal & 0xFF);
    }
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        int32_t posVal = realWorldPos.getVal(axisIdx);
        data.push_back((posVal >> 24) & 0xFF);
        data.push_back((posVal >> 16) & 0xFF);
        data.push_back((posVal >> 8) & 0xFF);
        data.push_back(posVal & 0xFF);
    }

}

