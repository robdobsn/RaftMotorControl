# Set the target Espressif chip
set(IDF_TARGET "esp32s3")

# System version
add_compile_definitions(SYSTEM_VERSION="1.0.0")

# Raft components
set(RAFT_COMPONENTS
    RaftCore@main
)
