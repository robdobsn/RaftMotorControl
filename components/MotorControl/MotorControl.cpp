/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motor Control
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "MotorControl.h"
#include "RaftJsonPrefixed.h"
#include "RaftBusSystem.h"
#include "Logger.h"
#include "HomingPattern.h"
#include "DeviceManager.h"
#include "DeviceTypeRecordDynamic.h"

// #define DEBUG_MOTOR_CMD_JSON
// #define DEBUG_SEND_CMD_TIMINGS

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

    // Register motion patterns
    _motionController.addMotionPattern("homing", HomingPattern::create);

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
    // Check for new-style axis queries: e.g. "0pos", "1min", "2max"
    if (isdigit(param[0])) {
        // Parse axis index
        int axisIdx = param[0] - '0';
        const char* prop = param + 1;
        // Check for "pos"
        if (strcmp(prop, "pos") == 0) {
            isFresh = true;
            AxesValues<AxisPosDataType> pos = _motionController.getLastMonitoredPos();
            return pos.getVal(axisIdx);
        }
        // Check for "min" or "max"
        if (strcmp(prop, "min") == 0) {
            bool fresh = false;
            bool triggered = _motionController.getEndStopState(axisIdx, false, fresh);
            isFresh = fresh;
            return triggered ? 1.0 : 0.0;
        }
        if (strcmp(prop, "max") == 0) {
            bool fresh = false;
            bool triggered = _motionController.getEndStopState(axisIdx, true, fresh);
            isFresh = fresh;
            return triggered ? 1.0 : 0.0;
        }
        if (strcmp(prop, "steps") == 0) {
            AxesValues<AxisStepsDataType> steps = _motionController.getAxisTotalSteps();
            isFresh = true;
            return steps.getVal(axisIdx);
        }
        // Unknown property
        isFresh = false;
        return 0.0;
    }
    switch(tolower(param[0]))
    {
        case 'x':
        case 'y':
        case 'z':
        {
            // Get axis position
            isFresh = true;
            AxesValues<AxisPosDataType> pos = _motionController.getLastMonitoredPos();
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
    return sendCmdJSON(cmdJSON, nullptr);
}

RaftRetCode MotorControl::sendCmdJSON(const char* cmdJSON, String* respMsg)
{
    RaftRetCode retCode = RAFT_OK;
#ifdef DEBUG_SEND_CMD_TIMINGS
    uint64_t startTimeUs = micros();
    uint64_t parseStartUs, motionArgsStartUs, moveToStartUs;
#endif

    // Extract command from JSON
#ifdef DEBUG_SEND_CMD_TIMINGS
    parseStartUs = micros();
#endif
    RaftJson jsonInfo(cmdJSON);
    String cmd = jsonInfo.getString("cmd", "");
#ifdef DEBUG_SEND_CMD_TIMINGS
    uint64_t parseTimeUs = micros() - parseStartUs;
#endif

#ifdef DEBUG_MOTOR_CMD_JSON
        LOG_I(MODULE_PREFIX, "sendCmdJSON cmd %s", cmd.c_str());
#endif

    if (cmd.equalsIgnoreCase("motion"))
    {
#ifdef DEBUG_SEND_CMD_TIMINGS
        motionArgsStartUs = micros();
#endif
        MotionArgs motionArgs;
        motionArgs.fromJSON(cmdJSON);
#ifdef DEBUG_SEND_CMD_TIMINGS
        uint64_t motionArgsTimeUs = micros() - motionArgsStartUs;
        moveToStartUs = micros();
#endif
#ifdef DEBUG_MOTOR_CMD_JSON
        LOG_I(MODULE_PREFIX, "sendCmdJSON motion %s", motionArgs.toJSON().c_str());
#endif
        retCode = _motionController.moveTo(motionArgs, respMsg);
#ifdef DEBUG_SEND_CMD_TIMINGS
        uint64_t moveToTimeUs = micros() - moveToStartUs;
        uint64_t totalTimeUs = micros() - startTimeUs;
        LOG_I(MODULE_PREFIX, "sendCmdJSON TIMING: total=%lluus parse=%lluus motionArgs=%lluus moveTo=%lluus cmd=%s",
              totalTimeUs, parseTimeUs, motionArgsTimeUs, moveToTimeUs, cmd.c_str());
#endif
    }
    else if (cmd.equalsIgnoreCase("stop"))
    {
        // Stop motion: halt ramp generator and clear motion queue
        // Optional: disable motors if "disableMotors" is true
        bool disableMotors = jsonInfo.getBool("disableMotors", false);
        _motionController.stopAll(disableMotors);
    }
    else if (cmd.equalsIgnoreCase("setOrigin"))
    {
        _motionController.setCurPositionAsOrigin();
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
    else if (cmd.equalsIgnoreCase("startPattern"))
    {
        String patternName = jsonInfo.getString("pattern", "");
        uint32_t runTimeMs = jsonInfo.getInt("forMs", 0);
        _motionController.setMotionPattern(patternName, runTimeMs, cmdJSON);
    }
    else if (cmd.equalsIgnoreCase("stopPattern"))
    {
        _motionController.stopPattern();
    }
#ifdef WARN_ON_SEND_CMD_JSON_FAILED
if (retCode != RAFT_OK)
{
    LOG_W(MODULE_PREFIX, "sendCmdJSON failed: req %s resp %s", 
            cmdJSON, respMsg && respMsg->length() > 0 ? respMsg->c_str() : Raft::getRetCodeStr(retCode));
}
#endif

#ifdef DEBUG_SEND_CMD_TIMINGS
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "sendCmdJSON TIMING: total=%lluus parse=%lluus cmd=%s",
          totalTimeUs, parseTimeUs, cmd.c_str());
#endif
    return retCode;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get debug string
/// @return Debug string
String MotorControl::getDebugJSON(bool includeBraces) const
{
    return _motionController.getDebugJSON(includeBraces);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get the device status as JSON (for DeviceManager aggregation)
/// @return JSON string with motor status
String MotorControl::getStatusJSON() const
{
    // Delegate to getDataJSON with PUBLISH level
    return getDataJSON(DEVICE_JSON_LEVEL_PUBLISH);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get a hash value representing the current device state for change detection
/// @return Hash value based on step counts and status flags
uint32_t MotorControl::getDeviceStateHash() const
{
    // Get current state
    AxesValues<AxisStepsDataType> steps = _motionController.getAxisTotalSteps();
    bool busy = _motionController.isBusy();
    bool paused = _motionController.isPaused();
    
    // Build hash from step counts and status flags
    // Note: Position is not included because it is derived from step counts
    // (through kinematics transformation), so if steps haven't changed, position can't either
    uint32_t hash = 0;
    for (uint32_t i = 0; i < steps.numAxes(); i++)
    {
        AxisStepsDataType stepVal = steps.getVal(i);
        // XOR in both halves of the step value
        hash ^= (uint32_t)(stepVal & 0xFFFFFFFF);
        if constexpr (sizeof(AxisStepsDataType) > 4)
        {
            hash ^= (uint32_t)(stepVal >> 32);
        }
    }
    
    // Add status flags to the hash
    hash ^= (busy ? 0x01 : 0x00);
    hash ^= (paused ? 0x02 : 0x00);
    
    return hash;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get the device status as binary (for DeviceManager aggregation)
/// @return Binary data vector with motor status
std::vector<uint8_t> MotorControl::getStatusBinary() const
{
    // Get device data from motion controller
    std::vector<uint8_t> data;
    _motionController.formBinaryDataResponse(data);
    
    // Wrap with standard header
    std::vector<uint8_t> binBuf;
    RaftDevice::genBinaryDataMsg(binBuf, DeviceManager::DEVICE_CONN_MODE_DIRECT, 
                                  0, getDeviceTypeIndex(), true, data);
    return binBuf;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get the device type record for this device
/// @param devTypeRec (out) Device type record with schema
/// @return true if the device has a device type record
bool MotorControl::getDeviceTypeRecord(DeviceTypeRecordDynamic& devTypeRec) const
{
    // Device info JSON with binary schema
    // Format: 2-byte timestamp, 3x4-byte floats (pos), 3x4-byte int32s (steps), 1-byte flags, 4-byte pattern
    // Total: 31 bytes
    // Flags byte at offset 26: bit 0 = busy, bit 1 = paused
    static const char* devInfoJson = R"~({"name":"Motor Controller","desc":"Multi-axis Motor Controller","manu":"Robotical","type":"MotorControl")~"
        R"~(,"resp":{"b":31,"a":[)~"
        R"~({"n":"pos0","t":">f","u":"mm","r":[-1000,1000],"d":1,"f":".2f","o":"float"},)~"
        R"~({"n":"pos1","t":">f","u":"mm","r":[-1000,1000],"d":1,"f":".2f","o":"float"},)~"
        R"~({"n":"pos2","t":">f","u":"mm","r":[-1000,1000],"d":1,"f":".2f","o":"float"},)~"
        R"~({"n":"steps0","t":">i","u":"steps","r":[-2147483648,2147483647],"d":1,"f":"d","o":"int"},)~"
        R"~({"n":"steps1","t":">i","u":"steps","r":[-2147483648,2147483647],"d":1,"f":"d","o":"int"},)~"
        R"~({"n":"steps2","t":">i","u":"steps","r":[-2147483648,2147483647],"d":1,"f":"d","o":"int"},)~"
        R"~({"n":"busy","at":26,"t":"B","r":[0,1],"m":"0x01","f":"b","o":"bool"},)~"
        R"~({"n":"paused","at":26,"t":"B","r":[0,1],"m":"0x02","s":1,"f":"b","o":"bool"})~"
        R"~(]}})~";
    
    // Set the device type record
    devTypeRec = DeviceTypeRecordDynamic(
        getPublishDeviceType().c_str(),
        "",     // addresses
        "",     // detectionValues
        "",     // initValues
        "",     // pollInfo
        31,     // pollDataSizeBytes
        devInfoJson,
        nullptr // pollResultDecodeFn
    );
    
    return true;
}
