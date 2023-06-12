# Ensure CMake supports FetchContent
cmake_minimum_required(VERSION 3.16)

idf_component_register(
      SRCS 
        "MotorControl.cpp"
        "Axes/AxisEndstopChecks.cpp"
        "Axes/AxisUtils.cpp"
        "Controller/MotionArgs.cpp"
        "Controller/MotionController.cpp"
        "Controller/MotionBlockManager.cpp"
        "Controller/MotionPlanner.cpp"
        "EndStops/EndStops.cpp"
        "Geometries/AxisGeomXYZ.cpp"
        "RampGenerator/RampGenTimer.cpp"
        "RampGenerator/RampGenerator.cpp"
        "RampGenerator/MotionBlock.cpp"
        "RampGenerator/MotionPipeline.cpp"
        "Steppers/StepDriverBase.cpp"
        "Steppers/StepDriverTMC2209.cpp"
      INCLUDE_DIRS
        "."
        "Axes"
        "Controller"
        "EndStops"
        "Geometries"
        "MotorEnabler"
        "RampGenerator"
        "Steppers"
      REQUIRES 
        raftcore-src
)
