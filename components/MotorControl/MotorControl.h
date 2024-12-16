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

    // Has capability
    virtual bool hasCapability(const char* pCapabilityStr) const override final;
 
    /// @brief Get JSON data from the device
    /// @param level Level of data to return
    /// @return JSON string
    virtual String getDataJSON(RaftDeviceJSONLevel level = DEVICE_JSON_LEVEL_MIN) const override final;

    /// @brief Set motor on time after move
    /// @param motorOnTimeAfterMoveSecs Motor on time after move in seconds
    /// @return RaftRetCode
    RaftRetCode setMotorOnTimeAfterMoveSecs(float motorOnTimeAfterMoveSecs)
    {
        return _motionController.setMotorOnTimeAfterMoveSecs(motorOnTimeAfterMoveSecs);
    }

    /// @brief Set max motor current (amps)
    /// @param axisIdx Axis index
    /// @param maxMotorCurrent Max motor current (amps)
    /// @param timeNowMs Current time in milliseconds
    /// @return RaftRetCode
    RaftRetCode setMaxMotorCurrentAmps(uint32_t axisIdx, float maxMotorCurrent, uint32_t timeNowMs)
    {
        return _motionController.setMaxMotorCurrentAmps(axisIdx, maxMotorCurrent, timeNowMs);
    }

    /// @brief Get time of last device status update
    /// @param includeElemOnlineStatusChanges Include element online status changes in the status update time
    /// @param includePollDataUpdates Include poll data updates in the status update time
    /// @return Time of last device status update in milliseconds
    virtual uint32_t getDeviceInfoTimestampMs(bool includeElemOnlineStatusChanges, bool includePollDataUpdates) const override final;

    /// @brief Get the device status as JSON
    /// @return JSON string
    virtual String getStatusJSON() const override final;

    /// @brief Get device debug info JSON
    /// @return JSON string
    virtual String getDebugJSON(bool includeBraces) const override final;

    /// @brief Set test time ms
    void setTestTimeMs(uint32_t testTimeMs, uint32_t nonTimerIntervalMs)
    {
        _testTimeMs = testTimeMs;
        _testNonTimerIntervalMs = nonTimerIntervalMs;
    }

private:
    // Motion controller
    MotionController _motionController;

    // Motor serial bus
    RaftBus* _pMotorSerialBus = nullptr;

    // Record status
    static const uint32_t RECORD_STATUS_MS_DEFAULT = 200;
    uint32_t _recordStatusMs = RECORD_STATUS_MS_DEFAULT;
    uint32_t _readLastMs = 0;

    // Test time
    uint32_t _testTimeMs = 0;
    uint32_t _testNonTimerIntervalMs = 0;
    
    // Form device data response
    void formDeviceDataResponse(std::vector<uint8_t>& data) const;

    // Debug
    static constexpr const char* MODULE_PREFIX = "MotorControl";    
};
