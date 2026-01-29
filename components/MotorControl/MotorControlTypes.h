/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotorControlTypes
// Common types and enumerations for motor control
//
// Rob Dobson 2024
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// Out of bounds action
enum class OutOfBoundsAction
{
    USE_DEFAULT = 0,  // For MotionArgs: use SysType default (not valid for SysType)
    DISCARD,          // Reject move if out of bounds
    CLAMP,            // Constrain/clamp coordinates to bounds
    ALLOW             // Allow out of bounds coordinates
};
