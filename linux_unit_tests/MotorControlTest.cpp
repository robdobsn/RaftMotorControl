

#include "MotorControl.h"

int main()
{
    MotorControl motorControl("MotorControl", "{}");
    motorControl.setup();
    motorControl.loop();
    return 0;
}
