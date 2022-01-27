rm -rf build
mkdir -p build/test/reports
cd build
cmake ..
cmake --build .
cd ..
build/test/tests -r junit -o build/test/reports/mbes-lib-test-report.xml
