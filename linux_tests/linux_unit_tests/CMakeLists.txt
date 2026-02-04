cmake_minimum_required(VERSION 3.16)
project(linux_unit_tests LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include RaftCore
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../RaftCore/build_linux/install")
find_package(RaftCore REQUIRED)

# Include RaftMotorControl
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../build_linux/install")
find_package(RaftMotorControl REQUIRED)

# Dependencies
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 REQUIRED)

# Fetch matplotlib-cpp
add_custom_target(matplotlibcpp
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/FetchGitRepo.py
        https://github.com/lava/matplotlib-cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/matplotlib-cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Fetching matplotlib-cpp repository..."
)

# Include paths
include_directories(
    ${Python3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${PROCESSED_INCLUDE_DIRS}
)

# Matplotlib-cpp
set(MATPLOTLIB_CPP_DEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/matplotlib-cpp")
include_directories(${MATPLOTLIB_CPP_DEST_DIR})

# Source files
set(SOURCES
    # raftkinematicstest.cpp
    # MotorControlTest.cpp
    raftkinematicstest.cpp
)

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    RaftCore::RaftCore
    RaftMotorControl::RaftMotorControl
    ${Python3_LIBRARIES}
    pthread
)
