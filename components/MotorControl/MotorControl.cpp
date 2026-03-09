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
#include "HomingSeekCenter.h"
#include "DeviceManager.h"
#include "DeviceTypeRecordDynamic.h"
#include "DeviceTypeRecords.h"

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

    // Extract homing pattern
    _homingPattern = deviceConfig.getString("homingPattern", "homing-seek-center");

    // Setup serial bus
    String serialBusName = deviceConfig.getString("bus", "");
    _pMotorSerialBus = raftBusSystem.getBusByName(serialBusName);
    _motionController.setupSerialBus(_pMotorSerialBus, false);

    // Register motion patterns
    _motionController.addMotionPattern("homing-seek-center", HomingSeekCenter::create);
    LOG_I(MODULE_PREFIX, "setup registered homing-seek-center pattern");

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
/// @brief Get named value from the device
/// @param pParam Parameter name
/// @param isFresh (out) true if the value is fresh
/// @return double value
double MotorControl::getNamedValue(const char* param, bool& isFresh) const
{
    if (!param)
    {
        isFresh = false;
        return 0.0;
    }

    // Named values for motion/geometry metadata
    if (!isdigit(param[0]))
    {
        String paramStr(param);
        String geom = deviceConfig.getString("motion/geom", "");
        String geomLower = geom;
        geomLower.toLowerCase();
        bool isScara = geomLower.indexOf("scara") >= 0;

        const AxesParams axesParams = _motionController.getAxesParams();
        double arm1Len = deviceConfig.getDouble("motion/arm1LenMM", 0.0);
        double arm2Len = deviceConfig.getDouble("motion/arm2LenMM", 0.0);
        double maxRadius = deviceConfig.getDouble("motion/maxRadiusMM", 0.0);
        if (maxRadius <= 0.0 && arm1Len > 0.0 && arm2Len > 0.0)
            maxRadius = arm1Len + arm2Len;

        if (paramStr.equalsIgnoreCase("axesCount"))
        {
            isFresh = true;
            return axesParams.getNumAxes();
        }
        if (paramStr.equalsIgnoreCase("arm1LenMM"))
        {
            isFresh = arm1Len > 0.0;
            return arm1Len;
        }
        if (paramStr.equalsIgnoreCase("arm2LenMM"))
        {
            isFresh = arm2Len > 0.0;
            return arm2Len;
        }
        if (paramStr.equalsIgnoreCase("maxRadiusMM"))
        {
            isFresh = maxRadius > 0.0;
            return maxRadius;
        }
        if (paramStr.equalsIgnoreCase("originTheta2OffsetDegrees"))
        {
            isFresh = true;
            return deviceConfig.getDouble("motion/originTheta2OffsetDegrees", 180.0);
        }
        if (paramStr.equalsIgnoreCase("homeBeforeMove"))
        {
            isFresh = true;
            return deviceConfig.getBool("motion/homeBeforeMove", true) ? 1.0 : 0.0;
        }
        if (paramStr.equalsIgnoreCase("homed"))
        {
            isFresh = true;
            return _motionController.isAllAxesHomed() ? 1.0 : 0.0;
        }
        if (paramStr.length() > 5 && paramStr.startsWith("homed") && isdigit(paramStr[5]))
        {
            uint32_t axisIdx = paramStr[5] - '0';
            isFresh = true;
            return _motionController.isAxisHomed(axisIdx) ? 1.0 : 0.0;
        }
        if (paramStr.equalsIgnoreCase("outOfBoundsDefault"))
        {
            isFresh = true;
            return static_cast<double>(axesParams.getOutOfBoundsDefault());
        }
        if (paramStr.equalsIgnoreCase("workspaceMinX") || paramStr.equalsIgnoreCase("workspaceMaxX") ||
            paramStr.equalsIgnoreCase("workspaceMinY") || paramStr.equalsIgnoreCase("workspaceMaxY"))
        {
            double minX = 0.0;
            double maxX = 0.0;
            double minY = 0.0;
            double maxY = 0.0;
            if (isScara)
            {
                if (maxRadius <= 0.0)
                {
                    isFresh = false;
                    return 0.0;
                }
                minX = -maxRadius;
                maxX = maxRadius;
                minY = -maxRadius;
                maxY = maxRadius;
                isFresh = true;
            }
            else
            {
                minX = axesParams.getMinUnits(0);
                maxX = axesParams.getMaxUnits(0);
                minY = axesParams.getMinUnits(1);
                maxY = axesParams.getMaxUnits(1);
                isFresh = true;
            }

            if (paramStr.equalsIgnoreCase("workspaceMinX")) return minX;
            if (paramStr.equalsIgnoreCase("workspaceMaxX")) return maxX;
            if (paramStr.equalsIgnoreCase("workspaceMinY")) return minY;
            if (paramStr.equalsIgnoreCase("workspaceMaxY")) return maxY;
        }
    }

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
/// @brief Get named string from the device
/// @param param Parameter name
/// @param isFresh (out) true if the value is fresh
/// @return String value
String MotorControl::getNamedString(const char* param, bool& isFresh) const
{
    if (!param)
    {
        isFresh = false;
        return "";
    }

    String paramStr(param);
    if (paramStr.equalsIgnoreCase("geom"))
    {
        isFresh = true;
        return deviceConfig.getString("motion/geom", "");
    }
    if (paramStr.equalsIgnoreCase("homingPattern"))
    {
        isFresh = true;
        return _homingPattern;
    }
    if (paramStr.equalsIgnoreCase("outOfBoundsDefault"))
    {
        const AxesParams axesParams = _motionController.getAxesParams();
        OutOfBoundsAction action = axesParams.getOutOfBoundsDefault();
        isFresh = true;
        switch (action)
        {
            case OutOfBoundsAction::ALLOW: return "allow";
            case OutOfBoundsAction::CLAMP: return "clamp";
            case OutOfBoundsAction::DISCARD: return "discard";
            default: return "discard";
        }
    }

    isFresh = false;
    return "";
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
    else if (cmd.equalsIgnoreCase("home"))
    {
        // Convenience command to start the homing-seek-center pattern without needing to send the full pattern JSON
        // (which would be {"cmd":"startPattern","pattern":"homing-seek-center"})
        uint32_t runTimeMs = jsonInfo.getInt("forMs", 0);
        _motionController.setMotionPattern(_homingPattern, runTimeMs, cmdJSON);
    }
    else
    {
        retCode = RAFT_NOT_IMPLEMENTED;
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
    std::vector<uint8_t> data;
    _motionController.formBinaryDataResponse(data);
    const String payload = DeviceTypeRecords::deviceStatusToJson(
            0,
            DeviceOnlineState::ONLINE,
            getDeviceTypeIndex(),
            data);
    return String("{") + payload + String("}");
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
    RaftDevice::genBinaryDataMsg(binBuf, RaftDeviceID::BUS_NUM_DIRECT_CONN, 
                                  0, getDeviceTypeIndex(), DeviceOnlineState::ONLINE, data);
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
        getConfiguredDeviceType().c_str(),
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
