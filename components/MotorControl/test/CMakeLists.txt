idf_component_register(
                SRCS 
                  "./RampGenTimer_test.cpp"
                  "./HardwareRamp_test.cpp"
                  # "./Steppers_RampGen_test.cpp"
                  # "./MotionPlanner_test.cpp"
                  # "./MotionController_test.cpp"
                INCLUDE_DIRS
                  "."
                REQUIRES
                  unity
                  MotorControl
                )

target_compile_options(${COMPONENT_LIB} PRIVATE "-D LOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")
