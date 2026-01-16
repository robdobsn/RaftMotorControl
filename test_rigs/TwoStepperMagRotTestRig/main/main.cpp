/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main entry point
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCoreApp.h"
#include "RegisterSysMods.h"
#include "RegisterWebServer.h"
#include "MainSysMod.h"
#include "BusI2C.h"
#include "MotorControl.h"
#include "DeviceFactory.h"
#include "BusSerial.h"

// Create the app
RaftCoreApp raftCoreApp;

// Entry point
extern "C" void app_main(void)
{
    
    // Register SysMods from RaftSysMods library
    RegisterSysMods::registerSysMods(raftCoreApp.getSysManager());

    // Register WebServer from RaftWebServer library
    RegisterSysMods::registerWebServer(raftCoreApp.getSysManager());

    // Register BusI2C
    raftBusSystem.registerBus("I2C", BusI2C::createFn);

    // Register BusSerial (only on ESP32)
    raftBusSystem.registerBus("Serial", BusSerial::createFn);

    // Register sysmod
    raftCoreApp.registerSysMod("MainSysMod", MainSysMod::create, true);

        // Register MotorControl device
    deviceFactory.registerDevice("MotorControl", MotorControl::create);

    // Loop forever
    while (1)
    {
        // Loop the app
        raftCoreApp.loop();
    }
}
