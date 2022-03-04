rmdir /Q /S build
mkdir build\test
mkdir build\reports
cd build
cmake ..
cmake --build .
cd ..
build\test\tests.exe -r junit -o build/reports/mbes-lib-test-report.xml