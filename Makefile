CC=g++
OPTIONS=-Wall -std=c++11 -g
INCLUDES=-I/usr/include/eigen3
VERSION=0.1.0

FILES=src/datagrams/DatagramParser.cpp src/datagrams/DatagramParserFactory.cpp src/datagrams/s7k/S7kParser.cpp src/datagrams/kongsberg/KongsbergParser.cpp src/datagrams/xtf/XtfParser.cpp src/utils/NmeaUtils.cpp src/utils/StringUtils.cpp src/sidescan/SidescanPing.cpp

root=$(shell pwd)

exec_dir=build/bin
doc_dir=build/doc

test_exec_dir=build/test/bin
test_work_dir=build/test/work
test_result_dir=build/test-report
coverage_dir=build/coverage
coverage_exec_dir=build/coverage/bin
coverage_report_dir=build/coverage/report


default: prepare datagram-dump datagram-list georeference data-cleaning cidco-decoder
	echo "Building all"

georeference: prepare
	$(CC) $(OPTIONS) $(INCLUDES) -o $(exec_dir)/georeference src/examples/georeference.cpp $(FILES)

data-cleaning: prepare
	$(CC) $(OPTIONS) $(INCLUDES) -o $(exec_dir)/data-cleaning src/examples/data-cleaning.cpp $(FILES)

debugGeoreference: prepare
	$(CC) $(OPTIONS) -static $(INCLUDES) -o $(exec_dir)/georeference src/examples/georeference.cpp $(FILES)

cidco-decoder: prepare
	$(CC) $(OPTIONS) $(INCLUDES) -o $(exec_dir)/cidco-decoder src/examples/cidco-decoder.cpp $(FILES)

datagram-dump: prepare
	$(CC) $(OPTIONS) $(INCLUDES) -o $(exec_dir)/datagram-dump src/examples/datagram-dump.cpp $(FILES)

datagram-list: prepare
	$(CC) $(OPTIONS) $(INCLUDES) -o $(exec_dir)/datagram-list src/examples/datagram-list.cpp $(FILES)


test: default
	mkdir -p $(test_exec_dir)
	$(CC) $(OPTIONS) $(INCLUDES) -o $(test_exec_dir)/tests test/main.cpp $(FILES)
	mkdir -p $(test_result_dir)
	mkdir -p $(test_work_dir)
	cd $(test_work_dir)
	$(root)/$(test_exec_dir)/tests -r junit -o $(test_result_dir)/mbes-lib-test-report.xml

test-quick: default
	mkdir -p $(test_exec_dir)
	$(CC) $(OPTIONS) $(INCLUDES) -o $(test_exec_dir)/tests test/main.cpp $(FILES)
	mkdir -p $(test_result_dir)
	mkdir -p $(test_work_dir)
	cd $(test_work_dir)
	$(root)/$(test_exec_dir)/tests || true
	
test-debug: default
	mkdir -p $(test_exec_dir)
	$(CC) $(OPTIONS) -static $(INCLUDES) -o $(test_exec_dir)/tests test/main.cpp $(FILES)
	mkdir -p $(test_result_dir)
	mkdir -p $(test_work_dir)
	cd $(test_work_dir)
	$(root)/$(test_exec_dir)/tests || true


coverage: default
	mkdir -p $(coverage_dir)
	mkdir -p $(coverage_report_dir)
	mkdir -p $(coverage_exec_dir)
	mkdir -p $(test_work_dir)
	cppcheck --xml --xml-version=2 --enable=all --inconclusive --language=c++ src 2> $(coverage_report_dir)/cppcheck.xml
	$(CC) $(OPTIONS) $(INCLUDES) -fprofile-arcs -ftest-coverage -fPIC -O0 test/main.cpp $(FILES) -o $(coverage_exec_dir)/tests
	$(root)/$(coverage_exec_dir)/tests || true
	gcovr --branches -r $(root) --xml --xml-pretty -o $(coverage_report_dir)/gcovr-report.xml
	gcovr --branches -r $(root) --html --html-details -o $(coverage_report_dir)/gcovr-report.html

doc:
	rm -rf build/doxygen
	mkdir -p build/doxygen
	doxygen
	mkdir -p $(doc_dir)

clean:
	rm -rf build
	rm *.txt || true
	rm *.svp || true
	rm *.gcno || true
	rm *.gcda || true

datagram-list-test: default
	./build/bin/datagram-list test/data/s7k/20141016_150519_FJ-Saucier.s7k|sort|uniq -c

pcl-viewer: prepare
	rm -rf build/tempCMake
	mkdir -p build/tempCMake
	cd build/tempCMake && cmake ../../src/examples/viewer/ && make && mv viewer ../bin/
	rm -rf build/tempCMake

overlap: prepare
	rm -rf build/tempCMake
	mkdir -p build/tempCMake
	cd build/tempCMake && cmake ../../src/examples/overlap/ && make && mv overlap ../bin/
	rm -rf build/tempCMake

prepare:
	mkdir -p $(exec_dir)
.PHONY: all test clean doc
