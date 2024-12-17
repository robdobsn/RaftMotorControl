cmake_minimum_required(VERSION 3.16)

# Define the project
project(RaftMotorControl VERSION 1.0.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define RaftCore fetch directory and build variables
set(RAFTCORE_REPO "https://github.com/robdobsn/RaftCore#feature-generic-threading")
set(RAFTCORE_DIR ${CMAKE_BINARY_DIR}/raftcorelib)
set(RAFTCORE_BUILD_DIR ${RAFTCORE_DIR}/build_linux)
set(RAFTCORE_INSTALL_DIR ${RAFTCORE_BUILD_DIR}/install)

# Fetch and build RaftCore
add_custom_target(fetch_raftcore
    COMMAND git clone --depth 1 ${RAFTCORE_REPO} ${RAFTCORE_DIR}
    COMMENT "Cloning RaftCore repository..."
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    BYPRODUCTS ${RAFTCORE_DIR}
)

add_custom_target(build_raftcore ALL
    COMMAND cmake -DCMAKE_INSTALL_PREFIX=${RAFTCORE_INSTALL_DIR} ${RAFTCORE_DIR}/linux_tests/linux_lib
    COMMAND make -j$(nproc)
    COMMAND make install
    COMMENT "Building and installing RaftCore as a Linux library..."
    DEPENDS fetch_raftcore
    WORKING_DIRECTORY ${RAFTCORE_BUILD_DIR}
    BYPRODUCTS ${RAFTCORE_INSTALL_DIR}
)

# Add the RaftCore library path to the CMake prefix path
list(APPEND CMAKE_PREFIX_PATH ${RAFTCORE_INSTALL_DIR})
find_package(RaftCore REQUIRED)

# Set up RaftMotorControl source and include paths
set(RAFT_MOTOR_CONTROL_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl)

file(GLOB_RECURSE RAFT_MOTOR_CONTROL_SOURCE_FILES
    ${RAFT_MOTOR_CONTROL_BASE_DIR}/*.cpp
)

file(GLOB RAFT_MOTOR_CONTROL_INCLUDE_DIRS
    ${RAFT_MOTOR_CONTROL_BASE_DIR}/*
)

# Define the interface target for include paths
add_library(RaftMotorControlIncludes INTERFACE)
target_include_directories(RaftMotorControlIncludes INTERFACE
    $<BUILD_INTERFACE:${RAFT_MOTOR_CONTROL_INCLUDE_DIRS}>
)

# Define RaftMotorControl library
add_library(RaftMotorControl STATIC ${RAFT_MOTOR_CONTROL_SOURCE_FILES})
target_link_libraries(RaftMotorControl
    PUBLIC
    RaftMotorControlIncludes
    RaftCore::RaftCore
)

# Install RaftMotorControl
install(TARGETS RaftMotorControl RaftMotorControlIncludes
    EXPORT RaftMotorControlTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/RaftMotorControl
)

install(DIRECTORY ${RAFT_MOTOR_CONTROL_BASE_DIR}/
    DESTINATION include/RaftMotorControl
    FILES_MATCHING PATTERN "*.h"
)

# Export project configuration for find_package()
install(EXPORT RaftMotorControlTargets
    FILE RaftMotorControlConfig.cmake
    NAMESPACE RaftMotorControl::
    DESTINATION lib/cmake/RaftMotorControl
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
    DESTINATION lib/cmake/RaftMotorControl
)

# Add the target to the build system
export(TARGETS RaftMotorControl RaftMotorControlIncludes
    NAMESPACE RaftMotorControl::
    FILE "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlTargets.cmake"
)

# Ensure RaftMotorControl depends on building RaftCore
add_dependencies(RaftMotorControl build_raftcore)


# cmake_minimum_required(VERSION 3.16)

# # Define the base directory for source and include files
# set(RAFT_MOTOR_CONTROL_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl)

# # Automatically gather all source files
# file(GLOB_RECURSE RAFT_MOTOR_CONTROL_SOURCE_FILES
#   ${RAFT_MOTOR_CONTROL_BASE_DIR}/*.cpp
# )

# # Automatically gather all include directories
# file(GLOB RAFT_MOTOR_CONTROL_INCLUDE_DIRS
#   ${RAFT_MOTOR_CONTROL_BASE_DIR}/*
# )

# # Define the project
# project(RaftMotorControl VERSION 1.0.0 LANGUAGES C CXX)

# # Fetch RaftCore
# add_custom_target(raftcorelib
#     COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/FetchGitRepo.py
#         https://github.com/robdobsn/RaftCore
#         ${CMAKE_CURRENT_SOURCE_DIR}/raftcorelib
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#     COMMENT "Fetching RaftCore repository..."
# )

# # Function to process include directories for INSTALL_INTERFACE
# function(append_install_interface_prefix src_dirs dest_var prefix)
#     set(transformed_dirs "")
#     foreach(dir IN LISTS ${src_dirs})
#         string(REPLACE "./" "" dir "${dir}") # Remove leading "./"
#         list(APPEND transformed_dirs "${prefix}/${dir}")
#     endforeach()
#     set(${dest_var} "${transformed_dirs}" PARENT_SCOPE)
# endfunction()

# # Handle build configurations based on the system
# if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

#   # Set C++ standard
#   set(CMAKE_CXX_STANDARD 20)
#   set(CMAKE_CXX_STANDARD_REQUIRED ON)

#   # Include RaftCore
#   list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../RaftCore/build_linux/install")
#   find_package(RaftCore REQUIRED)

#   # Process include directories for BUILD_INTERFACE
#   append_install_interface_prefix(
#     "RAFT_MOTOR_CONTROL_INCLUDE_DIRS"   # Source list
#     "RAFT_MOTOR_CONTROL_BUILD_INCLUDE_DIRS"   # Output variable
#     "components"
#   )

#   # Define the interface target for include paths
#   add_library(RaftMotorControlIncludes INTERFACE)
#   target_include_directories(RaftMotorControlIncludes INTERFACE
#     $<BUILD_INTERFACE:${RAFT_MOTOR_CONTROL_BUILD_INCLUDE_DIRS}>
#   )

#   foreach(dir IN LISTS RAFT_MOTOR_CONTROL_INCLUDE_DIRS)
#     target_include_directories(RaftMotorControlIncludes INTERFACE
#         $<INSTALL_INTERFACE:include/RaftMotorControl/${dir}>
#     )
#   endforeach()

#   # Add -fPIC for static library compilation
#   set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#   # Define the library
#   add_library(RaftMotorControl STATIC ${RAFT_MOTOR_CONTROL_SOURCE_FILES})
#   target_link_libraries(RaftMotorControl 
#     PUBLIC 
#     RaftMotorControlIncludes 
#     RaftCore::RaftCore)

#   # Set up installation paths
#   install(TARGETS RaftMotorControl RaftMotorControlIncludes
#       EXPORT RaftMotorControlTargets
#       ARCHIVE DESTINATION lib
#       LIBRARY DESTINATION lib
#       RUNTIME DESTINATION bin
#       INCLUDES DESTINATION include/RaftMotorControl
#   )

#   # Install include files
#   install(DIRECTORY ${RAFT_MOTOR_CONTROL_BASE_DIR}/
#       DESTINATION include/RaftMotorControl
#       FILES_MATCHING PATTERN "*.h"
#   )

#   # Export the project configuration for find_package()
#   install(EXPORT RaftMotorControlTargets
#       FILE RaftMotorControlConfig.cmake
#       NAMESPACE RaftMotorControl::
#       DESTINATION lib/cmake/RaftMotorControl
#   )

#   # Create a ConfigVersion file for compatibility
#   include(CMakePackageConfigHelpers)
#   write_basic_package_version_file(
#       "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
#       VERSION ${PROJECT_VERSION}
#       COMPATIBILITY SameMajorVersion
#   )

#   install(FILES
#       "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
#       DESTINATION lib/cmake/RaftMotorControl
#   )

#   # Add the target to the build system
#   export(TARGETS RaftMotorControl RaftMotorControlIncludes
#       NAMESPACE RaftMotorControl::
#       FILE "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlTargets.cmake"
#   )

# endif()


# # cmake_minimum_required(VERSION 3.16)

# # set(RAFT_MOTOR_CONTROL_SOURCE_FILES
# #   components/MotorControl/Axes/AxisEndstopChecks.cpp
# #   components/MotorControl/Controller/MotionArgs.cpp
# #   components/MotorControl/Controller/MotionBlockManager.cpp
# #   components/MotorControl/Controller/MotionController.cpp
# #   components/MotorControl/Controller/MotionPlanner.cpp
# #   components/MotorControl/EndStops/EndStops.cpp
# #   components/MotorControl/MotorControl.cpp
# #   components/MotorControl/RampGenerator/MotionBlock.cpp
# #   components/MotorControl/RampGenerator/MotionPipeline.cpp
# #   components/MotorControl/RampGenerator/RampGenerator.cpp
# #   components/MotorControl/RampGenerator/RampGenStats.cpp
# #   components/MotorControl/Steppers/StepDriverBase.cpp
# #   components/MotorControl/Steppers/StepDriverTMC2209.cpp
# # )

# # set(RAFT_MOTOR_CONTROL_INCLUDE_DIRS
# #   MotorControl/.
# #   MotorControl/Axes
# #   MotorControl/Controller
# #   MotorControl/EndStops
# #   MotorControl/Kinematics
# #   MotorControl/MotorEnabler
# #   MotorControl/RampGenerator
# #   MotorControl/Steppers
# # )

# # # Define the project
# # project(RaftMotorControl VERSION 1.0.0 LANGUAGES C CXX)

# # # Function to process include directories for INSTALL_INTERFACE
# # function(append_install_interface_prefix src_dirs dest_var prefix)
# #     set(transformed_dirs "")
# #     foreach(dir IN LISTS ${src_dirs})
# #         string(REPLACE "./" "" dir "${dir}") # Remove leading "./"
# #         list(APPEND transformed_dirs "${prefix}/${dir}")
# #     endforeach()
# #     set(${dest_var} "${transformed_dirs}" PARENT_SCOPE)
# # endfunction()

# # # Handle build configurations based on the system
# # if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

# #   # Set C++ standard
# #   set(CMAKE_CXX_STANDARD 20)
# #   set(CMAKE_CXX_STANDARD_REQUIRED ON)

# #   # Include RaftCore
# #   list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../RaftCore/build_linux/install")
# #   find_package(RaftCore REQUIRED)

# #   # Process include directories for BUILD_INTERFACE
# #   append_install_interface_prefix(
# #     "RAFT_MOTOR_CONTROL_INCLUDE_DIRS"   # Source list
# #     "RAFT_MOTOR_CONTROL_BUILD_INCLUDE_DIRS"   # Output variable
# #     "components"
# #   )

# #   # Define the interface target for include paths
# #   add_library(RaftMotorControlIncludes INTERFACE)
# #   target_include_directories(RaftMotorControlIncludes INTERFACE
# #     $<BUILD_INTERFACE:${RAFT_MOTOR_CONTROL_BUILD_INCLUDE_DIRS}>
# #   )

# #   foreach(dir IN LISTS RAFT_MOTOR_CONTROL_INCLUDE_DIRS)
# #     target_include_directories(RaftMotorControlIncludes INTERFACE
# #         $<INSTALL_INTERFACE:include/RaftMotorControl/${dir}>
# #     )
# #   endforeach()

# #   # Add -fPIC for static library compilation
# #   set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# #   # Define the library
# #   add_library(RaftMotorControl STATIC ${RAFT_MOTOR_CONTROL_SOURCE_FILES})
# #   target_link_libraries(RaftMotorControl 
# #     PUBLIC 
# #     RaftMotorControlIncludes 
# #     RaftCore::RaftCore)

# #   # Set up installation paths
# #   install(TARGETS RaftMotorControl RaftMotorControlIncludes
# #       EXPORT RaftMotorControlTargets
# #       ARCHIVE DESTINATION lib
# #       LIBRARY DESTINATION lib
# #       RUNTIME DESTINATION bin
# #       INCLUDES DESTINATION include/RaftMotorControl
# #   )

# #   # Install include files
# #   install(DIRECTORY components/
# #       DESTINATION include/RaftMotorControl
# #       FILES_MATCHING PATTERN "*.h"
# #   )

# #   # Export the project configuration for find_package()
# #   install(EXPORT RaftMotorControlTargets
# #       FILE RaftMotorControlConfig.cmake
# #       NAMESPACE RaftMotorControl::
# #       DESTINATION lib/cmake/RaftMotorControl
# #   )

# #   # Create a ConfigVersion file for compatibility
# #   include(CMakePackageConfigHelpers)
# #   write_basic_package_version_file(
# #       "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
# #       VERSION ${PROJECT_VERSION}
# #       COMPATIBILITY SameMajorVersion
# #   )

# #   install(FILES
# #       "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
# #       DESTINATION lib/cmake/RaftMotorControl
# #   )

# #   # Add the target to the build system
# #   export(TARGETS RaftMotorControl RaftMotorControlIncludes
# #       NAMESPACE RaftMotorControl::
# #       FILE "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlTargets.cmake"
# #   )

# # endif()
