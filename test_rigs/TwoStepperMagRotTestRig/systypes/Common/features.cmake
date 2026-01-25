# Set the target Espressif chip
set(IDF_TARGET "esp32s3")

# Raft components
set(RAFT_COMPONENTS
    RaftCore@feature-device-man-concurrency
    RaftSysMods@feature-debug-state-pub-performance
    RaftWebServer@feature-performance-of-websockets
    RaftI2C@feature-set-poll-interval
)

# File system
set(FS_TYPE "littlefs")
set(FS_IMAGE_PATH "../Common/FSImage")

# Web UI - disabled, served separately from dev machine
# The WebUI is now located at ../../WebUI and should be run with npm start for development
# set(UI_SOURCE_PATH "../Common/WebUI")

