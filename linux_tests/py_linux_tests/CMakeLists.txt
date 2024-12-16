cmake_minimum_required(VERSION 3.16)
project(MotorControlWrapper LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include RaftCore
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../RaftCore/build_linux/install")
find_package(RaftCore REQUIRED)

# Include RaftMotorControl
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../build_linux/install")
find_package(RaftMotorControl REQUIRED)

# Find Pybind11
set(pybind11_DIR "/home/rob/.local/lib/python3.10/site-packages/pybind11/share/cmake/pybind11")
find_package(pybind11 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Wrapper source files
set(SOURCES
    motor_control_wrapper.cpp
)

# Add the Python module
pybind11_add_module(motor_control ${SOURCES})

# Set output filename and location
set_target_properties(motor_control PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    PREFIX ""  # Removes the default "lib" prefix
    SUFFIX ".so"  # Ensures the suffix is exactly ".so"
)

# Link RaftCore or other dependencies if necessary
target_link_libraries(motor_control PRIVATE 
    RaftMotorControl::RaftMotorControl
    RaftCore::RaftCore)



# cmake_minimum_required(VERSION 3.16)
# project(raft_kinematics)

# set(CMAKE_VERBOSE_MAKEFILE ON)

# # Include pybind11
# set(pybind11_DIR "/home/rob/.local/lib/python3.10/site-packages/pybind11/share/cmake/pybind11")
# find_package(pybind11 REQUIRED)

# # Set the path to RaftCore
# list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../RaftCore/build_linux/install")
# set(RaftCore_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../RaftCore/build_linux/install/lib/cmake/RaftCore")
# find_package(RaftCore REQUIRED)

# # Include directories
# include_directories(
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl/Kinematics
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl/Axes
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl/Controller
#     ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl
# )

# # Add source files
# set(SOURCES
#     TestKinematics.cpp
# )

# # Create the Python module
# add_library(raft_kinematics MODULE ${SOURCES})

# # Link the module with pybind11 and RaftCore
# target_link_libraries(raft_kinematics PRIVATE pybind11::module RaftCore::RaftCore)

# # Set properties for the Python module
# set_target_properties(raft_kinematics PROPERTIES PREFIX "" SUFFIX ".so")
