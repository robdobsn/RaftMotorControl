# Raft Project
cmake_minimum_required(VERSION 3.16)

# Bootstrap using RaftBootstrap script
set(BOOTSTRAP_URL "https://github.com/robdobsn/RaftCore/releases/download/v1.24.1/RaftBootstrap.cmake")
file(DOWNLOAD ${BOOTSTRAP_URL} "${CMAKE_BINARY_DIR}/RaftBootstrap.cmake")
include("${CMAKE_BINARY_DIR}/RaftBootstrap.cmake")

 list(APPEND EXTRA_COMPONENT_DIRS "..")
 
# Set the components to include the tests for.
# This can be overriden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx" ..
# - when using idf.py: idf.py -T xxxxx build
set(TEST_COMPONENTS "RaftMotorControl" CACHE STRING "Components to test")

# Define the project dependencies
project(${_systype_name} DEPENDS ${ADDED_PROJECT_DEPENDENCIES})
