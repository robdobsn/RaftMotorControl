/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AxisUtils - Utility functions for axes
//
// Rob Dobson 2016-2022
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cmath>

// Utils
namespace AxisUtils
{
    /// @brief Calculate angle C of a triangle using the cosine rule
    /// @param a Length of side a
    /// @param b Length of side b
    /// @param c Length of side c
    /// @return Angle C in radians
    double cosineRule(double a, double b, double c)
    {
        double val = (a*a + b*b - c*c) / (2*a*b);
        if (val > 1) val = 1;
        if (val < -1) val = -1;
        return acos(val);
    }

    /// @brief Wrap an angle in radians to the range [0, 2*PI)
    /// @param angle Angle in radians
    /// @return Wrapped angle in radians
    double wrapRadians(double angleRadians)
    {
        static const double twoPi = 2.0 * M_PI;
        return angleRadians - twoPi * floor(angleRadians / twoPi);
    }

    /// @brief Wrap an angle in degrees to the range [0, 360)
    /// @param angle Angle in degrees
    /// @return Wrapped angle in degrees
    double wrapDegrees(double angleDegrees)
    {
        double wrapped = fmod(angleDegrees, 360.0);
        if (std::fabs(wrapped) < 1e-2)
            return 0.0;            
        if (wrapped < 0.0)
            wrapped += 360.0;
        return wrapped;
    }

    /// @brief Convert angle from radians to degrees
    /// @param angleRadians Angle in radians
    /// @param fix0To360 If true, wrap angle to [0, 360)
    /// @return Angle in degrees
    double r2d(double angleRadians, bool fix0To360 = true)
    {
        if (fix0To360)
            return wrapDegrees(angleRadians * 180.0 / M_PI);
        return angleRadians * 180.0 / M_PI;
    }

    /// @brief Convert angle from degrees to radians
    /// @param angleDegrees Angle in degrees
    /// @return Angle in radians
    double d2r(double angleDegrees, bool fix0To2Pi = true)
    {
        if (fix0To2Pi)
            return wrapRadians(angleDegrees * M_PI / 180.0);
        return angleDegrees * M_PI / 180.0;
    }

    /// @brief Check if two values are approximately equal within a range
    /// @param v1 First value
    /// @param v2 Second value
    /// @param withinRng Tolerance range
    /// @return True if values are approximately equal, false otherwise
    bool isApprox(double v1, double v2, double withinRng = 0.0001)
    {
        return fabs(v1 - v2) < withinRng;
    }

    /// @brief Check if two values are approximately equal within a range, considering wrapping
    /// @param v1 First value
    /// @param v2 Second value
    /// @param wrapSize Wrap size (e.g., 360 for degrees)
    /// @param withinRng Tolerance range
    /// @return True if values are approximately equal considering wrapping, false otherwise
    bool isApproxWrap(double v1, double v2, double wrapSize = 360.0, double withinRng = 0.0001)
    {
        double t1 = v1 - wrapSize * floor(v1 / wrapSize);
        double t2 = v2 - wrapSize * floor(v2 / wrapSize);
        return (fabs(t1 - t2) < withinRng) || (fabs(t1 - wrapSize - t2) < withinRng) || (fabs(t1 + wrapSize - t2) < withinRng);
    }
};
