#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "RaftKinematics.h"
#include "RaftKinematicsSystem.h"

namespace py = pybind11;

PYBIND11_MODULE(raft_kinematics, m) {
    m.doc() = "Python bindings for RaftKinematics";

    // Bind RaftKinematicsSystem
    py::class_<RaftKinematicsSystem>(m, "RaftKinematicsSystem")
        .def(py::init<>()) // Bind constructor
        .def("register_kinematics", &RaftKinematicsSystem::registerKinematics,
             "Register a kinematics type")
        .def_static("create_kinematics", &RaftKinematicsSystem::createKinematics,
                    "Create a kinematics instance from configuration");

    // Bind RaftKinematics (abstract class)
    py::class_<RaftKinematics>(m, "RaftKinematics")
        .def("pt_to_actuator", &RaftKinematics::ptToActuator,
             "Convert Cartesian point to actuator steps")
        .def("actuator_to_pt", &RaftKinematics::actuatorToPt,
             "Convert actuator steps to Cartesian point")
        .def("correct_step_overflow", &RaftKinematics::correctStepOverflow,
             "Correct step overflow")
        .def("pre_process_coords", &RaftKinematics::preProcessCoords,
             "Pre-process coordinates for motion planning");
}
