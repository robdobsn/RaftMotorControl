/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Motor Control
//
// Rob Dobson 2021-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftRetCode.h"
#include "RaftDevice.h"
#include "MotionController.h"
#include "RaftBus.h"

class MotorControl : public RaftDevice
{
public:
    /// @brief Create function for device factory
    /// @param pClassName device class name
    /// @param pDevConfigJson device configuration JSON
    /// @return RaftDevice* pointer to the created device
    static RaftDevice* create(const char* pClassName, const char* pDevConfigJson)
    {
        return new MotorControl(pClassName, pDevConfigJson);
    }

    /// @brief Constructor
    /// @param pClassName device class name
    /// @param pDevConfigJson device configuration JSON
    MotorControl(const char* pClassName, const char *pDevConfigJson);

    /// @brief Destructor
    virtual ~MotorControl();

    /// @brief Setup the device
    virtual void setup() override final;

    /// @brief Main loop for the device (called frequently)
    virtual void loop() override final;
    
    /// @brief Get named value from the device
    /// @param pParam Parameter name
    /// @param isFresh (out) true if the value is fresh
    /// @return double value
    virtual double getNamedValue(const char* pParam, bool& isFresh) const override final;

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Send a JSON command to the device 
    /// @param jsonCmd JSON string containing the command and parameters
    /// @return RaftRetCode 
    /// - `RAFT_OK`: Command executed successfully.
    /// - `RAFT_INVALID_DATA`: Invalid or missing data in the JSON command.
    /// - `RAFT_INVALID_OPERATION`: The command type is unrecognized or unsupported.
    ///
    /// ### Supported Commands
    ///
    /// #### Motion Command
    /// Executes a motion request with the specified parameters.
    /// @code
    /// {
    //    "cmd": "motion",
    //    "pos":[{"a":0,"p":100.0},{"a":1,"p":100.0}],     // Target positions for the axes in units
    //    "rel": false,                                    // Move to absolute positions (false) or relative (true)
    //    "feedrate": 100.0,                               // Feedrate percentage
    //    "ramped": true                                   // Whether the motion should be ramped or constant velocity
    /// }
    /// @endcode
    ///
    /// #### Max Current Command
    /// Sets the maximum motor current for a specific axis.
    /// @code
    /// {
    ///   "cmd": "maxCurrent",
    ///   "maxCurrentA": 2.5,
    ///   "axisIdx": 1
    /// }
    /// @endcode
    ///
    /// #### Motor Off After Move Command
    /// Configures the duration for which the motor remains on after a move.
    /// @code
    /// {
    ///   "cmd": "offAfter",
    ///   "offAfterS": 5.0
    /// }
    /// @endcode
    ///
    /// ### Notes
    /// - If the JSON structure is invalid or unsupported, the method returns `RAFT_INVALID_OPERATION`
    ///   or `RAFT_INVALID_DATA`.
    /// - Debugging can be enabled using the `DEBUG_MOTOR_CMD_JSON` macro for detailed logs.
    virtual RaftRetCode sendCmdJSON(const char* jsonCmd) override final;

    /// @brief Send JSON command with response message
    /// @param jsonCmd JSON command string
    /// @param respMsg Pointer to string for response message (can be nullptr)
    /// @return RaftRetCode
    virtual RaftRetCode sendCmdJSON(const char* jsonCmd, String* respMsg) override final;

    // Has capability
    virtual bool hasCapability(const char* pCapabilityStr) const override final;
 
    /// @brief Get JSON data from the device
    /// @param level Level of data to return
    /// @return JSON string
    virtual String getDataJSON(RaftDeviceJSONLevel level = DEVICE_JSON_LEVEL_MIN) const override final;

    /// @brief Get the device status as JSON (for DeviceManager aggregation)
    /// @return JSON string with motor status
    virtual String getStatusJSON() const override final;

    /// @brief Get a hash value representing the current device state for change detection
    /// @return Hash value based on step counts and status flags
    virtual uint32_t getDeviceStateHash() const override final;

    /// @brief Get the device status as binary (for DeviceManager aggregation)
    /// @return Binary data vector with motor status
    virtual std::vector<uint8_t> getStatusBinary() const override final;

    /// @brief Get the device type record for this device
    /// @param devTypeRec (out) Device type record with schema
    /// @return true if the device has a device type record
    virtual bool getDeviceTypeRecord(DeviceTypeRecordDynamic& devTypeRec) const override final;

    // Set motor on time after move
    RaftRetCode setMotorOnTimeAfterMoveSecs(float motorOnTimeAfterMoveSecs)
    {
        return _motionController.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
    }

    /// @brief Set max motor current (amps)
    /// @param axisIdx Axis index
    /// @param maxMotorCurrent Max motor current (amps)
    /// @return RaftRetCode
    RaftRetCode setMaxMotorCurrentAmps(uint32_t axisIdx, float maxMotorCurrent)
    {
        return _motionController.setMaxMotorCurrentAmps(axisIdx, maxMotorCurrent);
    }


    /// @brief Get device debug info JSON
    /// @return JSON string
    virtual String getDebugJSON(bool includeBraces) const override final;

private:
    // Motion controller
    MotionController _motionController;

    // Motor serial bus
    RaftBus* _pMotorSerialBus = nullptr;

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotorControl";    
};
