/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverBase
//
// Rob Dobson 2016-2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCore.h"
#include "RaftBus.h"
#include "StepDriverBase.h"

// Warning on CRC error
#define WARN_ON_CRC_ERROR
#ifdef ESP_PLATFORM
#define WARN_ON_DRIVER_BUSY
#endif

// Debug
// #define DEBUG_REGISTER_READ_PROCESS
// #define DEBUG_REGISTER_READ_VALUE
// #define DEBUG_REGISTER_WRITE
// #define DEBUG_READ_TIMEOUT
// #define DEBUG_READ_DETAIL

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Constructor
StepDriverBase::StepDriverBase()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Destructor
StepDriverBase::~StepDriverBase()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Setup
/// @param stepperName - name of stepper
/// @param stepperParams - parameters for the stepper
/// @param usingISR - true if using ISR
/// @param timeNowMs - current time in milliseconds
/// @return true if successful
bool StepDriverBase::setup(const String& stepperName, const StepDriverParams& stepperParams, bool usingISR, uint32_t timeNowMs)
{
    // Store config
    _name = stepperName;
    _requestedParams = stepperParams;
    _usingISR = usingISR;
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @briefSetup bus to use for serial comms with driver
/// @param pBus - pointer to bus
/// @param useBusForDirectionReversal - true if bus should be used for direction reversal (otherwise a pin must be used)
void StepDriverBase::setupSerialBus(RaftBus* pBus, bool useBusForDirectionReversal)
{
    _pSerialBus = pBus;
    _useBusForDirectionReversal = useBusForDirectionReversal;
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Loop - called frequently
/// @param timeNowMs - current time in milliseconds
void StepDriverBase::loop(uint32_t timeNowMs)
{
    // Check if we are reading
    if (isReadInProgress())
    {
#ifdef DEBUG_REGISTER_READ_PROCESS
            LOG_I(MODULE_PREFIX, "loop read %s rxAvail %d rdBytesToIgnore %d rdBytesReqd %d", 
                        _name.c_str(), _pSerialBus ? _pSerialBus->rxDataBytesAvailable() : 0, _readBytesToIgnore, _readBytesRequired);
#endif
        // Check for enough data to fulfill read
        if (busValid() && _pSerialBus->rxDataBytesAvailable() >= _readBytesToIgnore + _readBytesRequired)
        {
            // Read the data
            uint32_t reqLen = _readBytesToIgnore + _readBytesRequired;
            uint8_t readData[reqLen];
            if (_pSerialBus->rxDataGet(readData, reqLen) == reqLen)
            {
                // Clear read in progress
                _readInProgress = false;
  
#ifdef DEBUG_REGISTER_READ_PROCESS
                String debugStr;
                Raft::getHexStrFromBytes(readData, reqLen, debugStr);
                LOG_I(MODULE_PREFIX, "loop read %s regIdx %d rawread 0x%s", 
                                _name.c_str(), _readRegisterIdx, debugStr.c_str());
#endif

                // Check register index is valid
                if (_readRegisterIdx < _driverRegisters.size())
                {
                    // Check the CRC
                    uint8_t replyCRC = readData[_readBytesToIgnore + TMC_REPLY_CRC_POS];
                    uint8_t calculatedCRC = calcTrinamicsCRC(readData + _readBytesToIgnore, TMC_REPLY_DATAGRAM_LEN-1);
                    if (replyCRC != calculatedCRC)
                    {
#ifdef WARN_ON_CRC_ERROR
                        LOG_W(MODULE_PREFIX, "loop read CRC error 0x%02x 0x%02x %s stepperAddr 0x%02x regIdx %d regAddr 0x%02x", 
                                    replyCRC, 
                                    calculatedCRC,
                                    _name.c_str(),
                                    _requestedParams.address,
                                    _readRegisterIdx, 
                                    _driverRegisters[_readRegisterIdx].regAddr);
#endif
                        _lastReadResult = READ_RESULT_CRC_ERROR;
                        _driverRegisters[_readRegisterIdx].readValid = false;
                    }
                    else
                    {
                        // Data pointer
                        const uint8_t* pData = readData + _readBytesToIgnore + TMC_REPLY_DATA_POS;
                        _driverRegisters[_readRegisterIdx].regValCur = Raft::getBEUInt32AndInc(pData);

#ifdef DEBUG_REGISTER_READ_VALUE
                        LOG_I(MODULE_PREFIX, "loop read %s reg %s(0x%02x) data 0x%08x", 
                                    _name.c_str(),
                                    _driverRegisters[_readRegisterIdx].regName.c_str(),
                                    _driverRegisters[_readRegisterIdx].regAddr,
                                    _driverRegisters[_readRegisterIdx].regValCur);
#endif
                        _lastReadResult = READ_RESULT_OK;
                        _driverRegisters[_readRegisterIdx].readValid = true;
                    }
                }
            }
        }
    }

    // Check timeout
    if (isReadInProgress() && Raft::isTimeout(timeNowMs, _readStartTimeMs, READ_TIMEOUT_MS))
    {
#ifdef DEBUG_READ_TIMEOUT
        LOG_I(MODULE_PREFIX, "loop name %s read timed out", _name.c_str());
#endif
        _lastReadResult = READ_RESULT_TIMEOUT;
        _readInProgress = false;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Check if the driver is busy
/// @return true if busy
bool StepDriverBase::isBusy() const
{
    if (isReadInProgress())
        return true;
    if (busValid() && !_pSerialBus->isReady())
        return true;
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Write register in Trinamics driver
/// @param pRegName - name of register
/// @param regAddr - address of register
/// @param data - data to write
void StepDriverBase::writeTrinamicsRegister(const char* pRegName, uint8_t regAddr, uint32_t data)
{
    // Check valid
    if (!busValid() || isBusy())
        return;

    // Form datagram
    uint8_t datagram[] = {
        _tmcSyncByte,
        _requestedParams.address,
        uint8_t(regAddr | 0x80u),
        uint8_t((data >> 24) & 0xff),
        uint8_t((data >> 16) & 0xff),
        uint8_t((data >> 8) & 0xff),
        uint8_t(data & 0xff),
        0   // This is where the CRC will go
    };
    datagram[sizeof(datagram)-1] = calcTrinamicsCRC(datagram, sizeof(datagram)-1);

    // Form the command
    BusRequestInfo reqInfo(_name, _requestedParams.address, datagram, sizeof(datagram));

    // Send request
    bool resultOk = _pSerialBus->addRequest(reqInfo);
    _lastWriteResultOk = resultOk;

#ifdef DEBUG_REGISTER_WRITE
    String datagramStr;
    Raft::getHexStrFromBytes(datagram, sizeof(datagram), datagramStr);
    LOG_I(MODULE_PREFIX, "writeTrinamicsRegister %s axis %s reg %s(0x%02x) value 0x%08x datagram %s", 
                    resultOk ? "OK" : "FAILED", _name.c_str(), 
                    pRegName, regAddr, data, datagramStr.c_str());
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Start a read from Trinamics register
/// @param readRegisterIdx - index of register to read
void StepDriverBase::startReadTrinamicsRegister(uint32_t readRegisterIdx, uint32_t timeNowMs)
{
    // Check valid
    if (!busValid() || isBusy())
    {
#ifdef WARN_ON_DRIVER_BUSY
        LOG_W(MODULE_PREFIX, "startReadTrinamicsRegister name %s readRegisterIdx %d failed busValid %d busy %d",
                    _name.c_str(), readRegisterIdx, busValid(), isBusy());
#endif
        return;
    }
    if (readRegisterIdx >= _driverRegisters.size())
    {
        LOG_W(MODULE_PREFIX, "startReadTrinamicsRegister name %s readRegisterIdx %d failed out of range",
                    _name.c_str(), readRegisterIdx);
        return;
    }

    // Form datagram
    uint8_t datagram[] = {
        _tmcSyncByte,
        _requestedParams.address,
        _driverRegisters[readRegisterIdx].regAddr,
        0   // This is where the CRC will go
    };
    datagram[sizeof(datagram)-1] = calcTrinamicsCRC(datagram, sizeof(datagram)-1);

    // Form the command
    BusRequestInfo reqInfo(_name, _requestedParams.address, datagram, sizeof(datagram));

    // Clear any currently read data
    _pSerialBus->rxDataClear();

    // Send request
    _pSerialBus->addRequest(reqInfo);

    // Bytes required to be read
    _readBytesToIgnore = _singleWireReadWrite ? sizeof(datagram) : 0;
    _readBytesRequired = TMC_REPLY_DATAGRAM_LEN;
    _readRegisterIdx = readRegisterIdx;
    _readStartTimeMs = timeNowMs;
    _readInProgress = true;

#ifdef DEBUG_READ_DETAIL
        LOG_I(MODULE_PREFIX, "startReadTrinamicsRegister name %s regIdx %d regAddr %d", 
                    _name.c_str(),
                    _readRegisterIdx, 
                    _driverRegisters[readRegisterIdx].regAddr);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Calculate trinamics CRC
/// @param pData - pointer to data
/// @param len - length of data
/// @return CRC
uint8_t StepDriverBase::calcTrinamicsCRC(const uint8_t* pData, uint32_t len) const
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        uint8_t currentByte = pData[i];
        for (uint32_t j = 0; j < 8; j++)
        {
            // update CRC based result of XOR operation
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            crc &= 0xff;
            currentByte = currentByte >> 1;
        }
    }
    return crc;
}
