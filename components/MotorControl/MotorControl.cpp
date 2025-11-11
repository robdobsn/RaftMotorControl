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
// @brief Setup the device
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
// @brief Main loop for the device (called frequently)
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
/// @brief Get binary data from the device
/// @param formatCode format code for the command
/// @param buf (out) buffer to receive the binary data
/// @param bufMaxLen maximum length of data to return
/// @return RaftRetCode
RaftRetCode MotorControl::getDataBinary(uint32_t formatCode, std::vector<uint8_t>& buf, uint32_t bufMaxLen) const
{
    return RAFT_NOT_IMPLEMENTED;
}

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /// @brief Send a binary command to the device
// /// @param formatCode Format code for the command
// /// @param pData Pointer to the data
// /// @param dataLen Length of the data
// /// @return RaftRetCode
// RaftRetCode MotorControl::sendCmdBinary(uint32_t formatCode, const uint8_t* pData, uint32_t dataLen)
// {
//     // Check format code
//     if (formatCode == MULTISTEPPER_CMD_BINARY_FORMAT_1)
//     {
//         // Check length ok
//         if (dataLen < MULTISTEPPER_OPCODE_POS + 1)
//             return RAFT_INVALID_DATA;

//         // Check op-code
//         switch(pData[MULTISTEPPER_OPCODE_POS])
//         {
//             case MULTISTEPPER_MOVETO_OPCODE:
//             {
//                 handleCmdBinary_MoveTo(pData + MULTISTEPPER_OPCODE_POS + 1, dataLen - MULTISTEPPER_OPCODE_POS - 1);
//                 break;
//             }
//         }
//     }
//     return RAFT_OK;
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Send a JSON command to the device
/// @param jsonCmd JSON command
/// @return RaftRetCode
RaftRetCode MotorControl::sendCmdJSON(const char* cmdJSON)
{
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
        _motionController.moveTo(motionArgs);
    }
    else if (cmd.equalsIgnoreCase("maxCurrent"))
    {
        float maxCurrentA = jsonInfo.getDouble("maxCurrentA", 0);
        uint32_t axisIdx = jsonInfo.getInt("axisIdx", 0);
        _motionController.setMaxMotorCurrentAmps(axisIdx, maxCurrentA);
    }
    else if (cmd.equalsIgnoreCase("offAfter"))
    {
        float motorOnTimeAfterMoveSecs = jsonInfo.getDouble("offAfterS", 0);
        _motionController.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
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
    return RAFT_OK;
}

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /// @brief Handle binary move-to command
// /// @param pData Pointer to the data
// /// @param dataLen Length of the data
// void MotorControl::handleCmdBinary_MoveTo(const uint8_t* pData, uint32_t dataLen)
// {
//     // Check length ok
//     if (dataLen < MULTISTEPPER_MOVETO_BINARY_FORMAT_POS + 1)
//         return;
    
//     // Check version of args
//     if (pData[MULTISTEPPER_MOVETO_BINARY_FORMAT_POS] != MULTISTEPPER_MOTION_ARGS_BINARY_FORMAT_1)
//         return;

//     // Check args length
//     if (dataLen < sizeof(MotionArgs))
//         return;

//     // Send the request for interpretation
//     _motionController.moveTo((const MotionArgs&)*pData);
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Get debug string
/// @return Debug string
String MotorControl::getDebugJSON(bool includeBraces) const
{
    return _motionController.getDebugJSON(includeBraces);
}
