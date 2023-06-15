# Ensure CMake supports FetchContent
cmake_minimum_required(VERSION 3.16)

idf_component_register(
      SRCS 
        "components/MotorControl/MotorControl.cpp"
        "components/MotorControl/Axes/AxisEndstopChecks.cpp"
        "components/MotorControl/Axes/AxisUtils.cpp"
        "components/MotorControl/Controller/MotionArgs.cpp"
        "components/MotorControl/Controller/MotionController.cpp"
        "components/MotorControl/Controller/MotionBlockManager.cpp"
        "components/MotorControl/Controller/MotionPlanner.cpp"
        "components/MotorControl/EndStops/EndStops.cpp"
        "components/MotorControl/Geometries/AxisGeomXYZ.cpp"
        "components/MotorControl/RampGenerator/RampGenerator.cpp"
        "components/MotorControl/RampGenerator/RampGenStats.cpp"
        "components/MotorControl/RampGenerator/RampGenTimer.cpp"
        "components/MotorControl/RampGenerator/MotionBlock.cpp"
        "components/MotorControl/RampGenerator/MotionPipeline.cpp"
        "components/MotorControl/Steppers/StepDriverBase.cpp"
        "components/MotorControl/Steppers/StepDriverTMC2209.cpp"
      INCLUDE_DIRS
        "components/MotorControl"
        "components/MotorControl/Axes"
        "components/MotorControl/Controller"
        "components/MotorControl/EndStops"
        "components/MotorControl/Geometries"
        "components/MotorControl/MotorEnabler"
        "components/MotorControl/RampGenerator"
        "components/MotorControl/Steppers"
      REQUIRES 
        raftcore-src
)
