// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// //
// // AxesPosValues
// //
// // Rob Dobson 2016-2023
// //
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #pragma once

// #include <stdint.h>
// #include <math.h>
// #include "Logger.h"
// #include "RaftArduino.h"
// #include "esp_attr.h"

// class AxesPosValues
// {
// public:
//     typedef float AxisPosStoreType;
//     static const uint32_t STORE_TO_POS_FACTOR = 1;
//     AxisPosStoreType _pt[AXIS_VALUES_MAX_AXES];
//     uint8_t _validityFlags;

// public:
//     AxesPosValues()
//     {
//         clear();
//     }
//     AxesPosValues(const AxesPosValues &other)
//     {
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = other._pt[i];
//         _validityFlags = other._validityFlags;
//     }
//     AxesPosValues(AxisPosDataType ax0, AxisPosDataType ax1)
//     {
//         _pt[0] = ax0 * STORE_TO_POS_FACTOR;
//         _pt[1] = ax1 * STORE_TO_POS_FACTOR;
//         for (uint32_t i = 2; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = 0;
//         _validityFlags = 0x03;
//     }
//     AxesPosValues(AxisPosDataType ax0, AxisPosDataType ax1, AxisPosDataType ax2)
//     {
//         _pt[0] = ax0 * STORE_TO_POS_FACTOR;
//         _pt[1] = ax1 * STORE_TO_POS_FACTOR;
//         _pt[2] = ax2 * STORE_TO_POS_FACTOR;
//         for (uint32_t i = 3; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = 0;
//         _validityFlags = 0x07;
//     }
//     AxesPosValues(AxisPosDataType ax0, AxisPosDataType ax1, AxisPosDataType ax2, bool xValid, bool yValid, bool zValid)
//     {
//         _pt[0] = ax0 * STORE_TO_POS_FACTOR;
//         _pt[1] = ax1 * STORE_TO_POS_FACTOR;
//         _pt[2] = ax2 * STORE_TO_POS_FACTOR;
//         for (uint32_t i = 3; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = 0;
//         _validityFlags = xValid ? 0x01 : 0;
//         _validityFlags |= yValid ? 0x02 : 0;
//         _validityFlags |= zValid ? 0x04 : 0;
//     }
//     uint32_t numAxes() const
//     {
//         return AXIS_VALUES_MAX_AXES;
//     }
//     bool operator==(const AxesPosValues& other) const
//     {
//         if (_validityFlags != other._validityFlags)
//             return false;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             if ((_validityFlags & (0x01 << i)) && (_pt[i] != other._pt[i]))
//                 return false;
//         return true;
//     }
//     bool operator!=(const AxesPosValues& other) const
//     {
//         return !(*this == other);
//     }
//     void clear()
//     {
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = 0;
//         _validityFlags = 0;
//     }
//     inline AxisPosDataType IRAM_ATTR getVal(uint32_t axisIdx) const
//     {
//         if (axisIdx < AXIS_VALUES_MAX_AXES)
//             return _pt[axisIdx] / STORE_TO_POS_FACTOR;
//         return 0;
//     }
//     void setVal(uint32_t axisIdx, AxisPosDataType val)
//     {
//         if (axisIdx < AXIS_VALUES_MAX_AXES)
//         {
//             _pt[axisIdx] = val * STORE_TO_POS_FACTOR;
//             uint32_t axisMask = 0x01 << axisIdx;
//             _validityFlags |= axisMask;
//         }
//     }
//     void set(AxisPosDataType val0, AxisPosDataType val1, AxisPosDataType val2 = 0)
//     {
//         _pt[0] = val0 * STORE_TO_POS_FACTOR;
//         _pt[1] = val1 * STORE_TO_POS_FACTOR;
//         _pt[2] = val2 * STORE_TO_POS_FACTOR;
//         for (uint32_t i = 3; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = 0; 
//         _validityFlags = 0x07;
//     }
//     void setValid(uint32_t axisIdx, bool isValid)
//     {
//         if (axisIdx < AXIS_VALUES_MAX_AXES)
//         {
//             uint32_t axisMask = 0x01 << axisIdx;
//             if (isValid)
//                 _validityFlags |= axisMask;
//             else
//                 _validityFlags &= ~axisMask;
//         }
//     }
//     bool isValid(uint32_t axisIdx) const
//     {
//         if (axisIdx < AXIS_VALUES_MAX_AXES)
//         {
//             uint32_t axisMask = 0x01 << axisIdx;
//             return (_validityFlags & axisMask) != 0;
//         }
//         return false;
//     }
//     bool anyValid() const
//     {
//         return (_validityFlags != 0);
//     }
//     AxesPosValues &operator=(const AxesPosValues &other)
//     {
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             _pt[i] = other._pt[i];
//         _validityFlags = other._validityFlags;
//         return *this;
//     }
//     AxesPosValues operator-(const AxesPosValues &pt) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             result._pt[i] = _pt[i] - (pt.isValid(i) ? pt._pt[i] : 0);
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator-(AxisPosDataType val) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             result._pt[i] = _pt[i] - (val * STORE_TO_POS_FACTOR);
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator+(const AxesPosValues &pt) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             result._pt[i] = _pt[i] + (pt.isValid(i) ? pt._pt[i] : 0);
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator+(AxisPosDataType val) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//             result._pt[i] = _pt[i] + (val * STORE_TO_POS_FACTOR);
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator/(const AxesPosValues &pt) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//         {
//             if (pt._pt[i] != 0)
//                 result._pt[i] = _pt[i] / (pt.isValid(i) ? pt._pt[i] : 1);
//         }
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator/(AxisPosDataType val) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//         {
//             if (val != 0)
//                 result._pt[i] = _pt[i] / (val * STORE_TO_POS_FACTOR);
//         }
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator*(const AxesPosValues &pt) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//         {
//             result._pt[i] = _pt[i] * (pt.isValid(i) ? pt._pt[i] : 1);
//         }
//         result._validityFlags = _validityFlags;
//         return result;
//     }
//     AxesPosValues operator*(AxisPosDataType val) const
//     {
//         AxesPosValues result;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//         {
//             result._pt[i] = _pt[i] * (val * STORE_TO_POS_FACTOR);
//         }
//         result._validityFlags = _validityFlags;
//         return result;
//     }

//     // Calculate distance between points including only axes that are indicated
//     // true in the optional includeDist argument
//     AxisPosDataType distanceTo(const AxesPosValues &pt, bool includeDist[] = NULL) const
//     {
//         double distSum = 0;
//         for (uint32_t i = 0; i < AXIS_VALUES_MAX_AXES; i++)
//         {
//             if (isValid(i) && ((includeDist == NULL) || includeDist[i]))
//             {
//                 double sq = _pt[i] - pt._pt[i];
//                 sq = sq * sq;
//                 distSum += sq;
//             }
//         }
//         return sqrt(distSum) / STORE_TO_POS_FACTOR;
//     }

//     // Debug
//     String getDebugStr()
//     {
//         String debugStr;
//         for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
//         {
//             if (axisIdx != 0)
//                 debugStr += " ";
//             debugStr += String(_pt[axisIdx], 2);
//             if ((_validityFlags & (0x01 << axisIdx)) == 0)
//                 debugStr += "(INV)";
//         }
//         return debugStr;
//     }

//     // String toJSON()
//     // {
//     //     String jsonStr = "[";
//     //     for (uint32_t axisIdx = 0; axisIdx < AXIS_VALUES_MAX_AXES; axisIdx++)
//     //     {
//     //         if (axisIdx != 0)
//     //             jsonStr += ",";
//     //         jsonStr += String((double)(_pt[axisIdx]) / STORE_TO_POS_FACTOR, 2);
//     //     }
//     //     jsonStr += "]";
//     //     return jsonStr;
//     // }
// };