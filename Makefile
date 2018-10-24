CC=g++
OPTIONS=-Wall -std=c++0x

VERSION=0.1.0

root=$(shell pwd)

exec_dir=build/bin
doc_dir=build/doc

test_exec_dir=build/test/bin
test_work_dir=build/test/work
test_result_dir=build/test-report

default:
	mkdir -p $(exec_dir)
	$(CC) $(OPTIONS) -o $(exec_dir)/datagram-dump src/examples/datagram-dump.cpp

test: default
	mkdir -p $(test_exec_dir)
	$(CC) $(OPTIONS) -o $(test_exec_dir)/tests test/main.cpp

run-test: clean test
	mkdir -p $(test_result_dir)
	mkdir -p $(test_work_dir)
	cd $(test_work_dir)
	$(root)/$(test_exec_dir)/tests

doc:
	rm -rf build/doxygen
	mkdir -p build/doxygen
	doxygen
	mkdir -p $(doc_dir)

clean:
	rm -rf build

.PHONY: all test clean doc
