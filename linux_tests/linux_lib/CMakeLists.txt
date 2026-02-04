cmake_minimum_required(VERSION 3.16)

# Define the project
project(RaftMotorControl VERSION 1.0.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add -fPIC for static library compilation
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Define RaftCore paths
set(RAFTCORE_DIR ${CMAKE_BINARY_DIR}/raftcorelib/src)
set(RAFTCORE_BUILD_DIR ${RAFTCORE_DIR}/linux_tests/linux_lib/build_linux)
set(RAFTCORE_INSTALL_DIR ${RAFTCORE_BUILD_DIR}/install)

# Set correct library path for RaftCore
set(RAFTCORE_LIB ${RAFTCORE_INSTALL_DIR}/lib/libRaftCore.a)

# Clear the list before populating
set(RAFTCORE_INCLUDE_DIRS "")

# Use GLOB_RECURSE to get all entries (files and directories)
file(GLOB_RECURSE ALL_PATHS
    LIST_DIRECTORIES TRUE
    ${RAFTCORE_INSTALL_DIR}/include/*
)

# Filter only directories
foreach(path ${ALL_PATHS})
    if(IS_DIRECTORY ${path})
        list(APPEND RAFTCORE_INCLUDE_DIRS ${path})
    endif()
endforeach()

# # Filter only directories and make them relative to CMAKE_SOURCE_DIR
# foreach(path ${ALL_PATHS})
#     if(IS_DIRECTORY ${path})
#         file(RELATIVE_PATH RELATIVE_PATH_TO_SRC ${CMAKE_SOURCE_DIR} ${path})
#         list(APPEND RAFTCORE_INCLUDE_DIRS ${RELATIVE_PATH_TO_SRC})
#     endif()
# endforeach()

message(STATUS "============================= RAFTCORE_INCLUDE_DIRS: ${RAFTCORE_INCLUDE_DIRS}")

# Set up RaftMotorControl source and include paths
file(REAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl RAFT_MOTOR_CONTROL_BASE_DIR)

file(GLOB_RECURSE RAFT_MOTOR_CONTROL_SOURCE_FILES
    ${RAFT_MOTOR_CONTROL_BASE_DIR}/*.cpp
)

message(STATUS "============================= RAFT_MOTOR_CONTROL_SOURCE_FILES: ${RAFT_MOTOR_CONTROL_SOURCE_FILES}")

# First, gather all paths (files and directories)
file(GLOB ALL_PATHS
    ${RAFT_MOTOR_CONTROL_BASE_DIR}/*
)

# Clear the list before populating
set(RAFT_MOTOR_CONTROL_INCLUDE_DIRS "")

# Add the base directory first
list(APPEND RAFT_MOTOR_CONTROL_INCLUDE_DIRS ${RAFT_MOTOR_CONTROL_BASE_DIR})

# Filter only directories
foreach(path ${ALL_PATHS})
    if(IS_DIRECTORY ${path})
        list(APPEND RAFT_MOTOR_CONTROL_INCLUDE_DIRS ${path})
    endif()
endforeach()

message(STATUS "============================= RAFT_MOTOR_CONTROL_INCLUDE_DIRS: ${RAFT_MOTOR_CONTROL_INCLUDE_DIRS}")

# Define the interface target for include paths
add_library(RaftMotorControlIncludes INTERFACE)
target_include_directories(RaftMotorControlIncludes INTERFACE
    $<BUILD_INTERFACE:${RAFT_MOTOR_CONTROL_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${${RAFT_MOTOR_CONTROL_BASE_DIR}}>
)

foreach(dir IN LISTS RAFT_MOTOR_CONTROL_INCLUDE_DIRS)
  target_include_directories(RaftMotorControlIncludes INTERFACE
      $<INSTALL_INTERFACE:include/RaftMotorControl/${dir}>
  )
endforeach()

# Define the RaftMotorControl library
add_library(RaftMotorControl STATIC ${RAFT_MOTOR_CONTROL_SOURCE_FILES})

# Add all include directories (RaftCore + MotorControl)
target_include_directories(RaftMotorControl PUBLIC
    ${RAFT_MOTOR_CONTROL_BASE_DIR}
    ${RAFT_MOTOR_CONTROL_INCLUDE_DIRS}
    ${RAFTCORE_INCLUDE_DIRS}
)

# Link RaftCore library
target_link_libraries(RaftMotorControl PUBLIC RaftMotorControlIncludes)

# Install RaftMotorControl library
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


# cmake_minimum_required(VERSION 3.16)

# # Define the project
# project(RaftMotorControl VERSION 1.0.0 LANGUAGES C CXX)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Define RaftCore paths
# set(RAFTCORE_DIR ${CMAKE_BINARY_DIR}/raftcorelib)
# set(RAFTCORE_BUILD_DIR ${RAFTCORE_DIR}/linux_tests/linux_lib/build_linux)
# set(RAFTCORE_INSTALL_DIR ${RAFTCORE_BUILD_DIR}/install)

# # Set correct include directory and library path for RaftCore
# set(RAFTCORE_INCLUDE_DIR ${RAFTCORE_BUILD_DIR}/install/include)
# set(RAFTCORE_LIB ${RAFTCORE_BUILD_DIR}/install/lib/libRaftCore.a)

# # Set up RaftMotorControl source and include paths
# set(RAFT_MOTOR_CONTROL_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../components/MotorControl)

# file(GLOB_RECURSE RAFT_MOTOR_CONTROL_SOURCE_FILES
#     ${RAFT_MOTOR_CONTROL_BASE_DIR}/*.cpp
# )

# file(GLOB RAFT_MOTOR_CONTROL_INCLUDE_DIRS
#     ${RAFT_MOTOR_CONTROL_BASE_DIR}/*
# )

# # Define the RaftMotorControl library
# add_library(RaftMotorControl STATIC ${RAFT_MOTOR_CONTROL_SOURCE_FILES})

# # Link RaftCore
# target_include_directories(RaftMotorControl PUBLIC ${RAFTCORE_INCLUDE_DIR})
# target_link_libraries(RaftMotorControl PUBLIC 
#   ${RAFTCORE_LIB}
#   )

# # Install RaftMotorControl library
# install(TARGETS RaftMotorControl
#     EXPORT RaftMotorControlTargets
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
#     INCLUDES DESTINATION include/RaftMotorControl
# )

# install(DIRECTORY ${RAFT_MOTOR_CONTROL_BASE_DIR}/
#     DESTINATION include/RaftMotorControl
#     FILES_MATCHING PATTERN "*.h"
# )

# # Export project configuration for find_package()
# install(EXPORT RaftMotorControlTargets
#     FILE RaftMotorControlConfig.cmake
#     NAMESPACE RaftMotorControl::
#     DESTINATION lib/cmake/RaftMotorControl
# )

# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#     "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY SameMajorVersion
# )

# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
#     DESTINATION lib/cmake/RaftMotorControl
# )

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

# # Define RaftCore fetch directory and build variables
# set(RAFTCORE_REPO "https://github.com/robdobsn/RaftCore#feature-generic-threading")
# set(RAFTCORE_DIR ${CMAKE_BINARY_DIR}/raftcorelib)
# set(RAFTCORE_BUILD_DIR ${RAFTCORE_DIR}/linux_tests/linux_lib/build_linux)
# set(RAFTCORE_INSTALL_DIR ${RAFTCORE_BUILD_DIR}/install)

# # Fetch RaftCore
# add_custom_target(raftcorelib
#     COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/FetchGitRepo.py
#         ${RAFTCORE_REPO}
#         ${RAFTCORE_DIR}
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

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Include RaftCore
# find_package(RaftCore REQUIRED)

# # Process include directories for BUILD_INTERFACE
# append_install_interface_prefix(
#   "RAFT_MOTOR_CONTROL_INCLUDE_DIRS"   # Source list
#   "RAFT_MOTOR_CONTROL_BUILD_INCLUDE_DIRS"   # Output variable
#   "components"
# )

# # Define the interface target for include paths
# add_library(RaftMotorControlIncludes INTERFACE)
# target_include_directories(RaftMotorControlIncludes INTERFACE
#   $<BUILD_INTERFACE:${RAFT_MOTOR_CONTROL_BUILD_INCLUDE_DIRS}>
# )

# foreach(dir IN LISTS RAFT_MOTOR_CONTROL_INCLUDE_DIRS)
#   target_include_directories(RaftMotorControlIncludes INTERFACE
#       $<INSTALL_INTERFACE:include/RaftMotorControl/${dir}>
#   )
# endforeach()

# # Add -fPIC for static library compilation
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# # Define the library
# add_library(RaftMotorControl STATIC ${RAFT_MOTOR_CONTROL_SOURCE_FILES})
# target_link_libraries(RaftMotorControl 
#   PUBLIC 
#   RaftMotorControlIncludes 
#   RaftCore::RaftCore)

# # Set up installation paths
# install(TARGETS RaftMotorControl RaftMotorControlIncludes
#     EXPORT RaftMotorControlTargets
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
#     INCLUDES DESTINATION include/RaftMotorControl
# )

# # Install include files
# install(DIRECTORY ${RAFT_MOTOR_CONTROL_BASE_DIR}/
#     DESTINATION include/RaftMotorControl
#     FILES_MATCHING PATTERN "*.h"
# )

# # Export the project configuration for find_package()
# install(EXPORT RaftMotorControlTargets
#     FILE RaftMotorControlConfig.cmake
#     NAMESPACE RaftMotorControl::
#     DESTINATION lib/cmake/RaftMotorControl
# )

# # Create a ConfigVersion file for compatibility
# include(CMakePackageConfigHelpers)
# write_basic_package_version_file(
#     "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY SameMajorVersion
# )

# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlConfigVersion.cmake"
#     DESTINATION lib/cmake/RaftMotorControl
# )

# # Add the target to the build system
# export(TARGETS RaftMotorControl RaftMotorControlIncludes
#     NAMESPACE RaftMotorControl::
#     FILE "${CMAKE_CURRENT_BINARY_DIR}/RaftMotorControlTargets.cmake"
# )


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
