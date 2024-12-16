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
    // Check if test time set
    uint32_t timeNowMs = _testTimeMs > 0 ? _testTimeMs : millis();

    // Setup motion controller
    _motionController.setup(deviceConfig, timeNowMs);

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
    // Check if test time set
    uint32_t timeNowMs = _testTimeMs > 0 ? _testTimeMs : millis();

    // Loop motion controller
    _motionController.loop(timeNowMs, _testNonTimerIntervalMs);

    // Check if ready to record status
    if (!Raft::isTimeout(timeNowMs, _readLastMs, _recordStatusMs))
        return;
    _readLastMs = timeNowMs;
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
    // Check if test time set
    uint32_t timeNowMs = _testTimeMs > 0 ? _testTimeMs : millis();
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
        retCode = _motionController.moveTo(motionArgs, timeNowMs);
    }
    else if (cmd.equalsIgnoreCase("maxCurrent"))
    {
        float maxCurrentA = jsonInfo.getDouble("maxCurrentA", 0);
        uint32_t axisIdx = jsonInfo.getInt("axisIdx", 0);
        retCode = _motionController.setMaxMotorCurrentAmps(axisIdx, maxCurrentA, timeNowMs);
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
    // Resize to sufficient for new data
    uint32_t curPos = data.size();
    data.resize(curPos + 2 + AXIS_VALUES_MAX_AXES * 8);

    // Get average and store in format expected by conversion functions
    curPos = Raft::setBEUInt16(data.data(), curPos, _readLastMs & 0xFFFF);

    // Get position
    AxesValues<AxisStepsDataType> actuatorPos;
    AxesValues<AxisPosDataType> realWorldPos;
    _motionController.getLastMonitoredPos(actuatorPos, realWorldPos);
    for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
    {
        curPos = Raft::setBEInt32(data.data(), curPos, actuatorPos.getVal(axisIdx));
        curPos = Raft::setBEFloat32(data.data(), curPos, realWorldPos.getVal(axisIdx));
    }
}

