#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include "MotorControl.h"

namespace py = pybind11;

PYBIND11_MODULE(motor_control, m) {
    m.doc() = "Python bindings for the MotorControl C++ class";

    // Bind the RaftRetCode enum
    py::enum_<RaftRetCode>(m, "RaftRetCode")
        .value("OK", RAFT_OK)
        .value("BUSY", RAFT_BUSY)
        .value("POS_MISMATCH", RAFT_POS_MISMATCH)
        .value("NOT_XFERING", RAFT_NOT_XFERING)
        .value("NOT_STREAMING", RAFT_NOT_STREAMING)
        .value("SESSION_NOT_FOUND", RAFT_SESSION_NOT_FOUND)
        .value("CANNOT_START", RAFT_CANNOT_START)
        .value("INVALID_DATA", RAFT_INVALID_DATA)
        .value("INVALID_OBJECT", RAFT_INVALID_OBJECT)
        .value("INVALID_OPERATION", RAFT_INVALID_OPERATION)
        .value("INSUFFICIENT_RESOURCE", RAFT_INSUFFICIENT_RESOURCE)
        .value("OTHER_FAILURE", RAFT_OTHER_FAILURE)
        .value("NOT_IMPLEMENTED", RAFT_NOT_IMPLEMENTED)
        .value("BUS_PENDING", RAFT_BUS_PENDING)
        .value("BUS_HW_TIME_OUT", RAFT_BUS_HW_TIME_OUT)
        .value("BUS_ACK_ERROR", RAFT_BUS_ACK_ERROR)
        .value("BUS_ARB_LOST", RAFT_BUS_ARB_LOST)
        .value("BUS_SW_TIME_OUT", RAFT_BUS_SW_TIME_OUT)
        .value("BUS_INVALID", RAFT_BUS_INVALID)
        .value("BUS_NOT_READY", RAFT_BUS_NOT_READY)
        .value("BUS_INCOMPLETE", RAFT_BUS_INCOMPLETE)
        .value("BUS_BARRED", RAFT_BUS_BARRED)
        .value("BUS_NOT_INIT", RAFT_BUS_NOT_INIT)
        .value("BUS_STUCK", RAFT_BUS_STUCK)
        .value("BUS_SLOT_POWER_UNSTABLE", RAFT_BUS_SLOT_POWER_UNSTABLE)
        .value("UNKNOWN", RAFT_OTHER_FAILURE)  // Default
        .export_values();

    // Bind the RaftDeviceJSONLevel enum
    py::enum_<RaftDeviceJSONLevel>(m, "RaftDeviceJSONLevel")
        .value("MIN", DEVICE_JSON_LEVEL_MIN)
        .value("BASIC", DEVICE_JSON_LEVEL_BASIC)
        .value("FULL", DEVICE_JSON_LEVEL_FULL)
        .export_values();

    // Helper function to get string representation of RaftRetCode
    m.def("get_ret_code_str", &Raft::getRetCodeStr, py::arg("retc"),
        "Get string representation of RaftRetCode");

    // Bind the MotorControl class
    py::class_<MotorControl>(m, "MotorControl")
        .def(py::init<const char*, const char*>(), py::arg("class_name"), py::arg("config_json"))
        .def("setup", &MotorControl::setup)
        .def("loop", &MotorControl::loop)
        .def("get_data_binary", [](const MotorControl& self, uint32_t format_code, uint32_t buf_max_len) {
            std::vector<uint8_t> buf;
            RaftRetCode result = self.getDataBinary(format_code, buf, buf_max_len);
            return std::make_pair(result, buf);
        }, py::arg("format_code"), py::arg("buf_max_len"))
        .def("get_named_value", [](const MotorControl& self, const char* param) {
            bool isFresh;
            double value = self.getNamedValue(param, isFresh);
            return std::make_pair(value, isFresh);  // Return a tuple (value, isFresh)
        }, py::arg("param"), "Get a named value from the device, returning the value and freshness flag")
        .def("send_cmd_json", &MotorControl::sendCmdJSON, py::arg("json_cmd"))
        .def("has_capability", &MotorControl::hasCapability, py::arg("capability"))
        .def("get_data_json", &MotorControl::getDataJSON, py::arg("level") = DEVICE_JSON_LEVEL_MIN)
        .def("set_motor_on_time_after_move_secs", &MotorControl::setMotorOnTimeAfterMoveSecs, py::arg("motor_on_time_secs"))
        .def("set_max_motor_current_amps", &MotorControl::setMaxMotorCurrentAmps, py::arg("axis_idx"), py::arg("max_motor_current"))
        .def("get_debug_json", &MotorControl::getDebugJSON, py::arg("include_braces"))
        .def("get_device_info_timestamp_ms", &MotorControl::getDeviceInfoTimestampMs,
            py::arg("include_elem_online_status_changes"),
            py::arg("include_poll_data_updates"),
            "Get time of the last device status update in milliseconds")
       .def("get_status_json", [](const MotorControl& self) {
                    return std::string(self.getStatusJSON().c_str()); },
            "Get the device status as a JSON string");        
}
