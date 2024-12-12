# See README.md for install of Eigen and Python3.10 and matplotlib and numpy 

# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -Wall -std=c++20 -g

# Include paths
INCLUDES = -I. -I./RaftCore/components/core/Utils -I./RaftCore/components/core/ArduinoUtils -I./RaftCore/components/core/RaftJson
INCLUDES += $(shell find ../components/MotorControl -type d -exec echo -I{} \;) 
INCLUDES += $(shell find ../components/MotorControl/Axes -type d -exec echo -I{} \;) 
INCLUDES += $(shell find ../components/MotorControl/Kinematics -type d -exec echo -I{} \;)
INCLUDES += $(shell find ../components/MotorControl/Controller -type d -exec echo -I{} \;)
INCLUDES += -I/usr/include/eigen3 -I/usr/include/python3.10

MATPLOTLIB_CPP_DEST_DIR=./matplotlib-cpp
INCLUDES += -I$(MATPLOTLIB_CPP_DEST_DIR)

# Linker flags
LDFLAGS = -lpython3.10

# Source files
# SOURCES = 2R_2D_Kinematics.cpp
# SOURCES = animdemo.cpp
SOURCES = raftkinematicstest.cpp utils.cpp ./RaftCore/components/core/Utils/RaftUtils.cpp ./RaftCore/components/core/ArduinoUtils/ArduinoWString.cpp 

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
EXECUTABLE = linux_unit_tests

# Default target
all: $(EXECUTABLE)

# RaftCore
RAFT_CORE_REPO_URL=https://github.com/robdobsn/RaftCore
DEST_DIR=./RaftCore
# GIT_TAG=v1.0  # Optional: Specify a tag if needed

raft_core:
	@echo "Fetching RaftCore repository..."
	python3 ./FetchGitRepo.py $(RAFT_CORE_REPO_URL) $(DEST_DIR) $(GIT_TAG)

# Matplotlib-cpp
MATPLOTLIB_CPP_REPO_URL=https://github.com/lava/matplotlib-cpp
MATPLOTLIB_CPP_DEST_DIR=./matplotlib-cpp
# MATPLOTLIB_CPP_GIT_TAG=master  # Optional: Specify a tag if needed
INCLUDES += -I$(MATPLOTLIB_CPP_DEST_DIR)

matplotlibcpp:
	@echo "Fetching matplotlib-cpp repository..."
	python3 ./FetchGitRepo.py $(MATPLOTLIB_CPP_REPO_URL) $(MATPLOTLIB_CPP_DEST_DIR) $(MATPLOTLIB_CPP_GIT_TAG)

# Rule to link the executable
$(EXECUTABLE): raft_core matplotlibcpp $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

# Rule to compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean target
.PHONY: clean
clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

# Dependencies
$(OBJECTS): $(SOURCES)
