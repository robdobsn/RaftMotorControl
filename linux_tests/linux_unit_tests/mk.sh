
cd ../..
./mk_linux.sh
cd linux_tests/linux_unit_tests
cmake -S . -B build
cmake --build build -v
