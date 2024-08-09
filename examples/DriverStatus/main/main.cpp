/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main entry point
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCoreApp.h"
#include "RegisterSysMods.h"
#include "DriverStatus.h"
#include "BusSerial.h"
#include "MotorControl.h"

// Entry point
extern "C" void app_main(void)
{
    RaftCoreApp raftCoreApp;
    
    // Register SysMods from RaftSysMods library
    RegisterSysMods::registerSysMods(raftCoreApp.getSysManager());

    // Register sysmod
    raftCoreApp.registerSysMod("DriverStatus", DriverStatus::create, true);

    // Register BusSerial
    raftBusSystem.registerBus("Serial", BusSerial::createFn);

    // Register device motor control
    deviceFactory.registerDevice("Motors", MotorControl::create);

    // Loop forever
    while (1)
    {
        // Yield for 1 tick
        vTaskDelay(1);

        // Loop the app
        raftCoreApp.loop();
    }
}
