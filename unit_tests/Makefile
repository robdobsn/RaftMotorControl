ROOTDIR = $(realpath $(CURDIR)/..)
WORKING_DIR = $(realpath $(CURDIR))
DOCKER ?= docker run --rm -v $(ROOTDIR):$(ROOTDIR) -w $(WORKING_DIR) espressif/idf:v5.0.2
CMD ?= idf.py -B ./build build
COM_PORT ?= COM3

all: buildall

buildall: Makefile $(wildcard main/*)
	$(DOCKER) $(CMD)

flashwsl: Makefile $(wildcard main/*)
	$(DOCKER) $(CMD)
	python.exe $(WORKING_DIR)/build/_deps/raftcore-src/scripts/flashUsingPartitionCSV.py partitions.csv build unittests.bin $(COM_PORT) ESP32S3
	python.exe $(WORKING_DIR)/build/_deps/raftcore-src/scripts/SerialMonitor.py $(COM_PORT)

clean:
	$(DOCKER) mv ./build/compile_commands.json . || true
	$(DOCKER) rm -rf ./build sdkconfig || true
	$(DOCKER) mkdir ./build || true
	$(DOCKER) mv ./compile_commands.json ./build/ || true
