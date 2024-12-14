rm -rf build/
mkdir build -p && cd build
cmake ..
make VERBOSE=1
cd ..
cp build/raft_kinematics.so .