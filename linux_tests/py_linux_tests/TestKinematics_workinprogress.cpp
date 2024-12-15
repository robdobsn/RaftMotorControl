#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "RaftKinematics.h"
#include "AxesValues.h"
#include "AxesState.h"
#include "AxesParams.h"
#include "MotionArgs.h"

namespace py = pybind11;

// Custom wrapper for the abstract RaftKinematics class
class PyRaftKinematics : public RaftKinematics {
public:
    using RaftKinematics::RaftKinematics;

    bool ptToActuator(const AxesValues<AxisPosDataType>& targetPt,
                      AxesValues<AxisStepsDataType>& outActuator,
                      const AxesState& curAxesState,
                      const AxesParams& axesParams,
                      bool constrainToBounds) const override {
        PYBIND11_OVERRIDE_PURE(
            bool, /* Return type */
            RaftKinematics, /* Parent class */
            ptToActuator, /* Name of the function */
            targetPt,
            outActuator,
            curAxesState,
            axesParams,
            constrainToBounds
        );
    }

    bool actuatorToPt(const AxesValues<AxisStepsDataType>& targetActuator,
                      AxesValues<AxisPosDataType>& outPt,
                      const AxesState& curAxesState,
                      const AxesParams& axesParams) const override {
        PYBIND11_OVERRIDE_PURE(
            bool, /* Return type */
            RaftKinematics, /* Parent class */
            actuatorToPt, /* Name of the function */
            targetActuator,
            outPt,
            curAxesState,
            axesParams
        );
    }

    void correctStepOverflow(AxesState& curAxesState,
                             const AxesParams& axesParams) const override {
        PYBIND11_OVERRIDE(
            void, /* Return type */
            RaftKinematics, /* Parent class */
            correctStepOverflow, /* Name of the function */
            curAxesState,
            axesParams
        );
    }

    AxisDistDataType preProcessCoords(MotionArgs& args,
                                      const AxesState& axesState,
                                      const AxesParams& axesParams) const override {
        PYBIND11_OVERRIDE(
            AxisDistDataType, /* Return type */
            RaftKinematics, /* Parent class */
            preProcessCoords, /* Name of the function */
            args,
            axesState,
            axesParams
        );
    }
};

PYBIND11_MODULE(raft_kinematics, m) {
    m.doc() = "Python bindings for RaftKinematics";

    // Expose AxesValues
    py::class_<AxesValues<AxisPosDataType>>(m, "AxesValues")
        .def(py::init<>())
        .def("getVal", &AxesValues<AxisPosDataType>::getVal, "Get value by index")
        .def("setVal", &AxesValues<AxisPosDataType>::setVal, "Set value by index");

    py::class_<AxesValues<AxisStepsDataType>>(m, "AxesValuesSteps")
        .def(py::init<>())
        .def("getVal", &AxesValues<AxisStepsDataType>::getVal, "Get value by index")
        .def("setVal", &AxesValues<AxisStepsDataType>::setVal, "Set value by index");

    // Expose AxesState
    py::class_<AxesState>(m, "AxesState")
        .def(py::init<>())
        .def("getUnitsFromOrigin", &AxesState::getUnitsFromOrigin, "Get units from origin");

    // Expose AxesParams
    py::class_<AxesParams>(m, "AxesParams")
        .def(py::init<>())
        .def("isPrimaryAxis", &AxesParams::isPrimaryAxis, "Check if axis is primary");

    // Expose MotionArgs
    py::class_<MotionArgs>(m, "MotionArgs")
        .def(py::init<>())
        .def("getAxesSpecified", &MotionArgs::getAxesSpecified, "Get specified axes")
        .def("isRelative", &MotionArgs::isRelative, "Check if motion is relative")
        .def("getAxesPosConst", &MotionArgs::getAxesPosConst, "Get constant axes positions")
        .def("setAxesPositions", &MotionArgs::setAxesPositions, "Set axes positions");

    // Expose RaftKinematics
    py::class_<RaftKinematics, PyRaftKinematics>(m, "RaftKinematics")
        .def(py::init<>())
        .def("ptToActuator", &RaftKinematics::ptToActuator, "Convert a point in Cartesian to actuator steps")
        .def("actuatorToPt", &RaftKinematics::actuatorToPt, "Convert actuator steps to a point in Cartesian")
        .def("correctStepOverflow", &RaftKinematics::correctStepOverflow, "Correct step overflow for continuous rotation bots")
        .def("preProcessCoords", &RaftKinematics::preProcessCoords, "Pre-process coordinates and return distance to move");

    // Expose RaftKinematicsSystem
    py::class_<RaftKinematicsSystem>(m, "RaftKinematicsSystem")
        .def_static("createKinematics", [](const std::string& json_config) {
            RaftJsonIF config(json_config);
            RaftKinematics* kinematics = RaftKinematicsSystem::createKinematics(config);
            if (!kinematics) {
                throw std::runtime_error("Failed to create kinematics object");
            }
            return kinematics;
        }, py::return_value_policy::take_ownership, "Create a RaftKinematics object from a JSON string");
}
