rm -rf build_linux/
mkdir build_linux
cd build_linux
mkdir -p ./raftcorelib/src
cd ./raftcorelib/src
git clone --branch feature-generic-threading https://github.com/robdobsn/RaftCore .
cd linux_tests/linux_lib
chmod +x ./mk_lib.sh
./mk_lib.sh
cd ../../../../
cmake -DCMAKE_INSTALL_PREFIX=install ..
make -j$(nproc) VERBOSE=1
make install
cd ..
