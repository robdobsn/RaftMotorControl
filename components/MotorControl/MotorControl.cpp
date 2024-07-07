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

static const char *MODULE_PREFIX = "MotorControl";

#define DEBUG_MOTOR_CMD_JSON

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
    _motionController.setupSerialBus(_pMotorSerialBus, true); 

    // Debug
    LOG_I(MODULE_PREFIX, "setup type %s", deviceClassName.c_str());
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
String MotorControl::getDebugStr()
{
    return _motionController.getDebugStr();
}
