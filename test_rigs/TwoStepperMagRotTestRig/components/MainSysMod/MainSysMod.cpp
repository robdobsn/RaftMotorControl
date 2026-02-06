////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "MainSysMod.h"
#include "ExecTimer.h"
#include "DeviceTypeRecords.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "DeviceTypeRecords_generated.h"
#include "DevicePollRecords_generated.h"
#pragma GCC diagnostic pop

// #define INVERT_MT6701_ANGLE
// #define INVERT_AS5600_ANGLE

#define WARN_ON_API_CONTROL_FAIL
// #define DEBUG_API_CONTROL
// #define DEBUG_INFO_API_CONTROL_TIMINGS
// #define DEBUG_SENSOR_DEVICE_CALLBACK
// #define DEBUG_SENSOR_ANGLES

MainSysMod::MainSysMod(const char *pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
}

MainSysMod::~MainSysMod()
{
}

void MainSysMod::setup()
{
    // Register for sensor data
    registerForSensorData();
}

void MainSysMod::loop()
{
    // Periodic sensor angle logging
    if (Raft::isTimeout(millis(), _lastSensorAngleLogMs, SENSOR_ANGLE_LOG_INTERVAL_MS))
    {
        _lastSensorAngleLogMs = millis();
#ifdef DEBUG_SENSOR_ANGLES
        LOG_I(MODULE_PREFIX, "Sensor Angles: MT6701=%.2f° (%.2f°) AS5600=%.2f° (%.2f°)", 
              _mt6701AccumulatedAngle, _mt6701Angle, _as5600AccumulatedAngle, _as5600Angle);
#endif
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
#ifdef DEBUG_INFO_API_CONTROL_TIMINGS
    // Timing measurement start
    uint64_t startTimeUs = micros();
    uint64_t parseStartUs, sendStartUs;
    parseStartUs = micros();
#endif

    // Extract params
    RaftJson requestAsJSON = RestAPIEndpointManager::getJSONFromRESTRequest(reqStr.c_str(), RestAPIEndpointManager::PARAMS_ONLY);

    // Timing measurement - command build time
#ifdef DEBUG_INFO_API_CONTROL_TIMINGS
    uint64_t parseTimeUs = micros() - parseStartUs;
    sendStartUs = micros();
#endif

    // Send to MotorControl device
    String rsltStr;
    bool rslt = sendToMotorControl(requestAsJSON.c_str(), rsltStr);

#ifdef DEBUG_INFO_API_CONTROL_TIMINGS
    uint64_t sendTimeUs = micros() - sendStartUs;
    // Log timing information
    uint64_t totalTimeUs = micros() - startTimeUs;
    LOG_I(MODULE_PREFIX, "apiControl TIMING: total=%lluus parse=%lluus send=%lluus params %s requestAsJSON %s",
          totalTimeUs, parseTimeUs, sendTimeUs, reqStr.c_str(), requestAsJSON.c_str());
#endif

    // Result
    if (rslt)
    {
        rsltStr = "ok";
#ifdef DEBUG_API_CONTROL
        LOG_I(MODULE_PREFIX, "apiControl: reqStr %s rslt %s", reqStr.c_str(), rsltStr.c_str());
#endif
        return Raft::setJsonBoolResult(reqStr.c_str(), respStr, true);
    }
#ifdef DEBUG_API_CONTROL
    LOG_E(MODULE_PREFIX, "apiControl: FAILED reqStr %s rslt %s", reqStr.c_str(), rsltStr.c_str());
#endif
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
        // Use detailed error message if available, otherwise use return code string
        respStr = (errorMsg.length() > 0) ? errorMsg : Raft::getRetCodeStr(rc);
#ifdef WARN_ON_API_CONTROL_FAIL
        LOG_W(MODULE_PREFIX, "sendToMotorControl failed: rc=%s msg=%s", 
              Raft::getRetCodeStr(rc), respStr.c_str());
#endif
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
    RaftDevice* pMotor = pDevMan->getDeviceByStringLookup("MotorControl");
    if (!pMotor)
        return nullptr;
    return pMotor;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Register for data from magnetic encoder devices
void MainSysMod::registerForSensorData()
{
    // Get device manager
    DeviceManager* pDevMan = getSysManager()->getDeviceManager();
    if (!pDevMan)
        return;

    // Register for device data notifications from MT6701 (Joint 1)
    RaftDeviceID deviceID_MT6701 = RaftDeviceID(1, 0x06);  // I2C bus 1, address 0x06
    pDevMan->registerForDeviceData(deviceID_MT6701, 
        [this](uint32_t deviceTypeIdx, std::vector<uint8_t> data, const void* pCallbackInfo) {

            // Decode device data
            poll_MT6701 deviceData;
            DeviceTypeRecordDecodeFn pDecodeFn = deviceTypeRecords.getPollDecodeFn(deviceTypeIdx);
            if (pDecodeFn)
            {
                pDecodeFn(data.data(), data.size(), &deviceData, sizeof(deviceData), 1, _decodeStateMT6701);
                double mt6701Angle = deviceData.angle;
#ifdef INVERT_MT6701_ANGLE
                mt6701Angle = 360.0f - mt6701Angle;
#endif
                _mt6701Angle = mt6701Angle;
                _mt6701LastUpdateMs = millis();

                // Handle wrap-around detection and accumulation
                if (_mt6701FirstReading)
                {
                    _mt6701AccumulatedAngle = mt6701Angle;
                    _mt6701PrevRawAngle = mt6701Angle;
                    _mt6701FirstReading = false;
                }
                else
                {
                    float delta = mt6701Angle - _mt6701PrevRawAngle;
                    // Detect wrap from ~360 to ~0 (forward)
                    if (delta < -180.0f)
                    {
                        _mt6701AccumulatedAngle += 360.0f;
                    }
                    // Detect wrap from ~0 to ~360 (backward)
                    else if (delta > 180.0f)
                    {
                        _mt6701AccumulatedAngle -= 360.0f;
                    }
                    _mt6701AccumulatedAngle += delta;
                    _mt6701PrevRawAngle = mt6701Angle;
                }

#ifdef DEBUG_SENSOR_DEVICE_CALLBACK
                LOG_I(MODULE_PREFIX, "MT6701 callback: angle=%.2f accumulated=%.2f", mt6701Angle, _mt6701AccumulatedAngle);
#endif
            }
        },
        50  // Update rate ms
    );

    // Register for device data notifications from AS5600 (Joint 2)
    RaftDeviceID deviceID_AS5600 = RaftDeviceID(1, 0x36);  // I2C bus 1, address 0x36
    pDevMan->registerForDeviceData(deviceID_AS5600, 
        [this](uint32_t deviceTypeIdx, std::vector<uint8_t> data, const void* pCallbackInfo) {

            // Decode device data
            poll_AS5600 deviceData;
            DeviceTypeRecordDecodeFn pDecodeFn = deviceTypeRecords.getPollDecodeFn(deviceTypeIdx);
            if (pDecodeFn)
            {
                pDecodeFn(data.data(), data.size(), &deviceData, sizeof(deviceData), 1, _decodeStateAS5600);
                double as5600Angle = deviceData.angle;
#ifdef INVERT_AS5600_ANGLE
                as5600Angle = 360.0f - as5600Angle;
#endif
                _as5600Angle = as5600Angle;
                _as5600LastUpdateMs = millis();

                // Handle wrap-around detection and accumulation
                if (_as5600FirstReading)
                {
                    _as5600AccumulatedAngle = as5600Angle;
                    _as5600PrevRawAngle = as5600Angle;
                    _as5600FirstReading = false;
                }
                else
                {
                    float delta = as5600Angle - _as5600PrevRawAngle;
                    // Detect wrap from ~360 to ~0 (forward)
                    if (delta < -180.0f)
                    {
                        _as5600AccumulatedAngle += 360.0f;
                    }
                    // Detect wrap from ~0 to ~360 (backward)
                    else if (delta > 180.0f)
                    {
                        _as5600AccumulatedAngle -= 360.0f;
                    }
                    _as5600AccumulatedAngle += delta;
                    _as5600PrevRawAngle = as5600Angle;
                }

#ifdef DEBUG_SENSOR_DEVICE_CALLBACK
                LOG_I(MODULE_PREFIX, "AS5600 callback: angle=%.2f accumulated=%.2f", as5600Angle, _as5600AccumulatedAngle);
#endif
            }
        },
        50  // Update rate ms
    );

    LOG_I(MODULE_PREFIX, "Registered for magnetic encoder data (MT6701 @ 0x06, AS5600 @ 0x36)");
}
