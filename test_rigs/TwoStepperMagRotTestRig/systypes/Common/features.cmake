# Set the target Espressif chip
set(IDF_TARGET "esp32s3")

# Raft components
set(RAFT_COMPONENTS
    RaftCore@main
    RaftSysMods@main
    RaftWebServer@main
    RaftI2C@main
)

# File system
set(FS_TYPE "littlefs")
set(FS_IMAGE_PATH "../Common/FSImage")

# Web UI
# This assumes an app is built using npm run build
# The web app is built into a folder called "dist" in the UI_SOURCE_PATH
set(UI_SOURCE_PATH "../Common/WebUI")

# Gzip compression is enabled by default to minimize flash usage
# Uncomment the following line if you do NOT want to gzip the web UI
# set(WEB_UI_GEN_FLAGS ${WEB_UI_GEN_FLAGS} --nogzip)

# Source maps are disabled by default to minimize size
# Uncomment the following line to include a source map for the web UI - this will increase the size
# set(WEB_UI_GEN_FLAGS ${WEB_UI_GEN_FLAGS} --incmap)
