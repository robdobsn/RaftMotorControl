/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// StepDriverBase
//
// Rob Dobson 2023
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include "RaftUtils.h"
#include "StepDriverParams.h"

class RaftBus;

class StepDriverBase
{
public:
    StepDriverBase();
    virtual ~StepDriverBase();

    // Setup
    virtual bool setup(const String& stepperName, const StepDriverParams& stepperParams, bool usingISR);

    // Called after bus has been connected
    virtual void setupSerialBus(RaftBus* pBus, bool useBusForDirectionReversal);

    // Loop - called frequently
    virtual void loop();

    // Microsteps
    virtual void setMicrosteps(uint32_t microsteps)
    {
    }

    // Direction
    virtual void setDirection(bool dirn, bool forceSet = false) = 0;

    // Start and end a single step
    virtual void stepStart() = 0;
    virtual bool stepEnd() = 0;

    virtual uint32_t getSerialAddress() const
    {
        return _serialBusAddress;
    }

    virtual String getDriverType() const
    {
        return "None";
    }

    virtual void setMaxMotorCurrentAmps(float maxMotorCurrentAmps)
    {
    }

    virtual String getDebugJSON(bool includeBraces, bool detailed) const
    {
        return includeBraces ? "{}" : "";
    }

    virtual String getStatusJSON(bool includeBraces, bool detailed) const
    {
        return includeBraces ? "{}" : "";
    }

    virtual bool isOperatingOk() const
    {
        return true;
    }

protected:
    class DriverRegisterMap
    {
    public:
        DriverRegisterMap(const char* pRegName, uint8_t addr, uint32_t initVal, uint32_t writeMask, bool isConfig)
        {
            regName = pRegName;
            regAddr = addr;
            regWriteVal = initVal;
            writeBitMask = writeMask;
            isConfigReg = isConfig;
        }
        String regName;
        uint8_t regAddr = 0;
        uint32_t regValCur = 0;
        uint32_t regWriteVal = 0;
        uint32_t writeBitMask = 0xffffffff;
        bool isConfigReg : 1 = false;
        bool writePending : 1 = false;
        bool readPending : 1 = false;
        bool readValid : 1 = false;
    };

    // Write register in Trinamics driver
    void writeTrinamicsRegister(const char* pRegName, uint8_t regAddr, uint32_t data);

    // Start a read from Trinamics register
    void startReadTrinamicsRegister(uint32_t readRegisterIdx);

    // Calculate Trinamics CRC
    uint8_t calcTrinamicsCRC(const uint8_t* pData, uint32_t len) const;

    // Bus valid
    bool busValid() const
    {
        return _pSerialBus != nullptr;
    }

    // Driver busy
    bool isBusy() const;

    // Is read in progress
    bool isReadInProgress() const
    {
        if (!busValid())
            return false;
        return _readInProgress;
    }

    // Write pending register index
    uint32_t _lastPendWriteRegIdx = 0;
    int writePendingRegIdx()
    {
        for (uint32_t i = 0; i < _driverRegisters.size(); i++)
        {
            uint32_t idx = (i + _lastPendWriteRegIdx) % _driverRegisters.size();
            if (_driverRegisters[idx].writePending)
            {
                _lastPendWriteRegIdx = idx;
                return idx;
            }
        }
        return -1;
    }

    // Read pending register index
    uint32_t _lastPendReadRegIdx = 0;
    int readPendingRegIdx()
    {
        for (uint32_t i = 0; i < _driverRegisters.size(); i++)
        {
            uint32_t idx = (i + _lastPendReadRegIdx) % _driverRegisters.size();
            if (_driverRegisters[idx].readPending)
            {
                _lastPendReadRegIdx = idx;
                return idx;
            }
        }
        return -1;
    }

    // Get register value in hex
    String getRegValHex(uint32_t regIdx) const
    {
        if (regIdx >= _driverRegisters.size())
            return "";
        return Raft::getHexStr(_driverRegisters[regIdx].readValid ? 
                    _driverRegisters[regIdx].regValCur :
                    _driverRegisters[regIdx].regWriteVal, true);
    }

    // Bus used for communication with driver
    RaftBus* _pSerialBus = nullptr;
    uint8_t _serialBusAddress = 0;
    String _name;
    bool _useBusForDirectionReversal = false;

    // Stepping parameters
    StepDriverParams _requestedParams;

    // Hardware has been initialised
    bool _hwIsSetup = false;

    // Single wire used for read and write (e.g. on TMC2209 UART)
    bool _singleWireReadWrite = false;

    // Driver registers
    std::vector<DriverRegisterMap> _driverRegisters;

    // In the case of single-wire read/write some data must be ignored
    uint32_t _readBytesToIgnore = 0;

    // Read operation info
    uint32_t _readBytesRequired = 0;
    uint32_t _readStartTimeMs = 0;
    bool _readInProgress = false;
    uint32_t _readRegisterIdx = 0;

    // Sync byte for specific chip
    uint8_t _tmcSyncByte = 0;

    // Using ISR - so avoid logging, etc
    bool _usingISR = false;
    
    // Consts
    static const uint32_t READ_TIMEOUT_MS = 4;
    static const uint32_t TMC_REPLY_DATAGRAM_LEN = 8;
    static const uint32_t TMC_REPLY_DATA_POS = 3;
    static const uint32_t TMC_REPLY_DATA_LEN = 4;
    static const uint32_t TMC_REPLY_CRC_POS = 7;

    // Read and write results
    bool _lastWriteResultOk = false;
    String getWriteResultStr(bool writeResult) const
    {
        return writeResult ? "OK" : "Error";
    }
    enum READ_RESULT_ENUM
    {
        READ_RESULT_NONE,
        READ_RESULT_OK,
        READ_RESULT_CRC_ERROR,
        READ_RESULT_TIMEOUT
    };
    READ_RESULT_ENUM _lastReadResult = READ_RESULT_NONE;
    String getReadResultStr(READ_RESULT_ENUM readResult) const
    {
        switch (readResult)
        {
            case READ_RESULT_NONE:
                return "None";
            case READ_RESULT_OK:
                return "OK";
            case READ_RESULT_CRC_ERROR:
                return "CRC Fail";
            case READ_RESULT_TIMEOUT:
                return "Timeout";
        }
        return "Unknown";
    }

    // Debug
    static constexpr const char* MODULE_PREFIX = "StepDriverBase";    
};
