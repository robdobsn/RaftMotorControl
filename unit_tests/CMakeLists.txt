# unit test project
cmake_minimum_required(VERSION 3.16)
include(FetchContent)

# Fetch the RaftCore library
FetchContent_Declare(
    raftcore
    SOURCE_DIR RaftCore 
    GIT_REPOSITORY https://github.com/robdobsn/RaftCore.git
    GIT_TAG        main
)
FetchContent_Populate(raftcore)

set(ADDED_PROJECT_DEPENDENCIES ${ADDED_PROJECT_DEPENDENCIES} raftcore)
set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS} ${raftcore_SOURCE_DIR})

# Set the components to include the tests for.
# This can be overriden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx" ..
# - when using idf.py: idf.py -T xxxxx build
set(TEST_COMPONENTS "MotorControl" CACHE STRING "Components to test")

# Include the library root folder which contains the CMakelists.txt file
list(APPEND EXTRA_COMPONENT_DIRS "..")

# Include the Raft CMake
include(${raftcore_SOURCE_DIR}/scripts/RaftProject.cmake)

# Project
project(unittests)
