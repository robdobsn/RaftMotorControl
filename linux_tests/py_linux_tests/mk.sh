cd ../..
./mk_linux.sh
cd linux_tests/py_linux_tests
cmake -S . -B build
cmake --build build -v

