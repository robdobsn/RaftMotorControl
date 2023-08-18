/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisUtils - Utility functions for axes
//
// Rob Dobson 2016-2022
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <stdint.h>
#include <RaftArduino.h>
#include "esp_attr.h"
#include <Logger.h>

// Utils
class AxisUtils
{
public:
    static double cosineRule(double a, double b, double c);
    static double wrapRadians(double angle);
    static double wrapDegrees(double angle);
    static double r2d(double angleRadians);
    static double d2r(double angleDegrees);
    static bool isApprox(double v1, double v2, double withinRng = 0.0001);
    static bool isApproxWrap(double v1, double v2, double wrapSize=360.0, double withinRng = 0.0001);
};
