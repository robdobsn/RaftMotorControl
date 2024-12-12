cmake_minimum_required(VERSION 3.16)
project(raft_kinematics)

find_package(pybind11 REQUIRED)

add_library(raft_kinematics MODULE bindings.cpp RaftKinematics.cpp KinematicsXYZ.cpp KinematicsSingleArmSCARA.cpp)
target_link_libraries(raft_kinematics PRIVATE pybind11::module)
target_include_directories(raft_kinematics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
set_target_properties(raft_kinematics PROPERTIES PREFIX "" SUFFIX ".so")
