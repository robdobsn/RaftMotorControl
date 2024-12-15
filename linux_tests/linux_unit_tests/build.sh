
cd ..
./build_linux.sh
cd linux_unit_tests
cmake -S . -B build
cmake --build build -v
