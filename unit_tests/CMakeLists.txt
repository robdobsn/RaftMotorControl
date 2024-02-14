# This is the project CMakeLists.txt file for the test subproject
cmake_minimum_required(VERSION 3.16)
include(FetchContent)

# Fetch the RaftCore library
FetchContent_Declare(
  raftcore
  GIT_REPOSITORY https://github.com/robdobsn/RaftCore.git
  GIT_TAG        ReWorkConfigBase
)

# Populate first so we have a folder that we can add in EXTRA_COMPONENT_DIRS
message(STATUS "Fetching dependencies...")
FetchContent_Populate(raftcore)

# Fetch
set(EXTRA_COMPONENT_DIRS ${EXTRA_COMPONENT_DIRS} ${raftcore_SOURCE_DIR} "../components")
FetchContent_MakeAvailable(raftcore)

# Set the components to include the tests for.
# This can be overriden from CMake cache:
# - when invoking CMake directly: cmake -D TEST_COMPONENTS="xxxxx" ..
# - when using idf.py: idf.py -T xxxxx build
set(TEST_COMPONENTS "MotorControl" CACHE STRING "Components to test")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(unittests DEPENDS raftcore)

