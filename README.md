# RaftMotorControl

Raft is an opinionated operating environment for the Espressif ESP32 family

Raft currently supports: ESP32, ESP32S3, ESP32C3 with ESP IDF and Arduino frameworks

Raft:

* Is a framework to build both simple and complex applications with relative ease
* Has modules (called System Modules or SysMods) for many common app requirements like:
    * managing WiFi and BLE networking
    * web-server for files and REST APIs
    * web-socket and MQTT communication
* I2C bus management
* Uses JSON to configure SysMods and hardware interfaces
* Provides consistent RPC and publish/subscribe communications over WiFi (websocket and REST API), BLE and serial interfaces
* Builds on the Arduino setup() and loop() programming convention in a more capable and consistent framework
* Re-uses and builds upon well known technologies like JSON, REST, MQTT, WebServer, REST and I2C

This is the Motor Control component of Raft which provides:
* stepper motor driver
* 3D forward and reverse kinematics
* ramp generation for fine positional control with acceleration/deceleration

