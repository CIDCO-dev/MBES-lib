rm -rf build
mkdir -p build/test
mkdir -p build/reports
cd build
cmake ..
cmake --build .
cd ..
build/test/tests -r junit -o build/reports/mbes-lib-test-report.xml
