idf_component_register(SRCS 
            "test_main.cpp"
            "xyz_test.cpp"
        INCLUDE_DIRS 
            "."
        REQUIRES
            unity
            RaftCore
            RaftMotorControl
        WHOLE_ARCHIVE
        )

target_compile_options(${COMPONENT_LIB} PRIVATE "-D LOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")