# RaftMotorControl Test Suite - Phase 1 Layer Tests
cmake_minimum_required(VERSION 3.16)

# Test configuration
if (NOT DEFINED ENABLE_TESTING)
    set(ENABLE_TESTING OFF)
endif()

# Only build tests if explicitly enabled
if(ENABLE_TESTING)
    
    # Find required packages
    find_package(GTest REQUIRED)
    find_package(GMock REQUIRED)
    
    # Include directories
    set(MOTOR_CONTROL_INCLUDE_DIRS
        "../components/MotorControl/"
        "../components/MotorControl/Axes"
        "../components/MotorControl/Controller"
        "../components/MotorControl/Steppers"
        "../components/MotorControl/RampGenerator"
    )
    
    # Create mock RaftCore headers for testing
    set(MOCK_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/mock_includes")
    file(MAKE_DIRECTORY ${MOCK_INCLUDE_DIR})
    
    # Create minimal mock headers for compilation
    file(WRITE ${MOCK_INCLUDE_DIR}/Logger.h "
        #pragma once
        #define LOG_I(tag, ...) printf(__VA_ARGS__)
        #define LOG_W(tag, ...) printf(__VA_ARGS__)
        #define LOG_E(tag, ...) printf(__VA_ARGS__)
    ")
    
    file(WRITE ${MOCK_INCLUDE_DIR}/RaftUtils.h "
        #pragma once
        #include <string>
        typedef std::string String;
    ")
    
    file(WRITE ${MOCK_INCLUDE_DIR}/RaftJsonIF.h "
        #pragma once
        #include <string>
        class RaftJsonIF {
        public:
            virtual std::string getString(const char*, const char* def = \"\") const = 0;
            virtual double getDouble(const char*, double def = 0.0) const = 0;
            virtual bool getBool(const char*, bool def = false) const = 0;
            virtual long getLong(const char*, long def = 0) const = 0;
        };
    ")
    
    file(WRITE ${MOCK_INCLUDE_DIR}/RaftJson.h "
        #pragma once
        #include \"RaftJsonIF.h\"
        class RaftJson : public RaftJsonIF {
        public:
            RaftJson(const std::string&) {}
            std::string getString(const char*, const char* def = \"\") const override { return def; }
            double getDouble(const char*, double def = 0.0) const override { return def; }
            bool getBool(const char*, bool def = false) const override { return def; }
            long getLong(const char*, long def = 0) const override { return def; }
        };
    ")
    
    # Layer 1 Test: Motor Driver Base Interface
    add_executable(test_motor_driver_base
        unit/test_motor_driver_base.cpp
        ../components/MotorControl/Steppers/StepDriverTMC2209Enhanced.cpp
    )
    
    target_include_directories(test_motor_driver_base PRIVATE
        ${MOTOR_CONTROL_INCLUDE_DIRS}
        ${MOCK_INCLUDE_DIR}
    )
    
    target_link_libraries(test_motor_driver_base
        GTest::GTest
        GTest::Main
        GMock::GMock
    )
    
    # Add compile definitions for test environment
    target_compile_definitions(test_motor_driver_base PRIVATE
        UNIT_TEST_BUILD=1
        ESP32_MOCK_BUILD=1
    )
    
    # Register tests
    add_test(NAME MotorDriverBaseInterface COMMAND test_motor_driver_base)
    
    # Test runner script
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/run_tests.sh "#!/bin/bash
set -e
echo \"Running Layer 1 Tests: Motor Driver Base Interface\"
echo \"====================================================\"
./test_motor_driver_base
echo \"Layer 1 tests completed successfully!\"
    ")
    
    # Make script executable
    execute_process(COMMAND chmod +x ${CMAKE_CURRENT_BINARY_DIR}/run_tests.sh)
    
    # Status message
    message(STATUS "RaftMotorControl Layer 1 tests configured")
    message(STATUS "Run tests with: cd build && make && ./run_tests.sh")
    
endif()