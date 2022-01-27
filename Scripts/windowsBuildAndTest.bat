rmdir /Q /S build
mkdir build\test\reports
cd build
cmake ..
cmake --build .
cd ..
build\test\tests.exe -r junit -o build/test/reports/mbes-lib-test-report.xml