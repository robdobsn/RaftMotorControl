ROOTDIR = $(realpath $(CURDIR)/..)
WORKING_DIR = $(realpath $(CURDIR))
DOCKER_ESP_IDF ?= docker run --rm -v $(ROOTDIR):$(ROOTDIR) -w $(WORKING_DIR) espressif/idf:v5.1.2
TARGET ?= esp32s3
CMD ?= idf.py -B ./build build -DIDF_TARGET=$(TARGET)
PORT ?= COM9

all: builddocker

builddocker: Makefile $(wildcard main/*)
	$(DOCKER_ESP_IDF) $(CMD)

flashwsl: Makefile $(wildcard main/*) builddocker
	python.exe $(WORKING_DIR)/build/_deps/raftcore-src/scripts/flashUsingPartitionCSV.py partitions.csv build unittests.bin $(PORT) $(TARGET)
	python.exe $(WORKING_DIR)/build/_deps/raftcore-src/scripts/SerialMonitor.py $(PORT)

cleandocker:
	$(DOCKER_ESP_IDF) mv ./build/compile_commands.json . || true
	$(DOCKER_ESP_IDF) rm -rf ./build sdkconfig || true
	$(DOCKER_ESP_IDF) mkdir ./build || true
	$(DOCKER_ESP_IDF) mv ./compile_commands.json ./build/ || true

flashwslnd:
	$(CMD)
	python.exe $(WORKING_DIR)/build/_deps/raftcore-src/scripts/flashUsingPartitionCSV.py partitions.csv build unittests.bin $(PORT) $(TARGET)
	python.exe $(WORKING_DIR)/build/_deps/raftcore-src/scripts/SerialMonitor.py $(PORT)
