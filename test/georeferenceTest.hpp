/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   georeferenceTest.hpp
 * Author: emile
 *
 * Created on April 25, 2019, 9:13 AM
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "catch.hpp"
#include "../src/utils/Exception.hpp"
using namespace std;
#ifdef _WIN32
static string binexec("..\\bin\\georeference.exe");
static string outputdir(".");
#else
static string binexec("build/bin/georeference");
static string outputdir(".");
#endif

std::stringstream system_call(const std::string& command){

     std::stringstream out;
     FILE *fp;
     char path[1035];

#ifdef _WIN32
     fp = _popen(command.c_str(), "r");
#else
     fp = popen(command.c_str(), "r");
#endif
     if (fp == NULL) {
	printf("Failed to run command\n" );
	exit(1);
     }

     while (fgets(path, sizeof(path)-1, fp) != NULL) {
	     out<<path;
     }

#ifdef _WIN32
     _pclose(fp);
#else
     pclose(fp);
#endif

     return out;
}

TEST_CASE("test if the parameter x y z are correctly get")
{
    string commX = " -x 1";
    string commY = " -y 1";
    string commZ = " -z 1";
    string commFile = " test/data/all/example.all";
    string commTest = binexec+commX+commY+commZ+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="1:1:1");
}

TEST_CASE("test the leverArm result without parameter")
{
    string commX = "";
    string commY = "";
    string commZ = "";
    string commFile = " test/data/all/example.all";
    string commTest = binexec+commX+commY+commZ+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="0:0:0");
}

TEST_CASE("test the leverArm result without all the parameter")
{
    string commX = " -x 1";
    string commY = " -y 1";
    string commZ = " -z 1";
    string commFile = " test/data/all/example.all";
    string commTest = binexec+commX+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="1:0:0");
    commTest = binexec+commX+commZ+commFile;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="1:0:1");
    commTest = binexec+commY+commZ+commFile;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="0:1:1");
}

TEST_CASE("test the extention of the file receive")
{
    string commFile = " test/data/all/example.all";
    string commTest = binexec+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()!="Error while parsing test/data/all/example.all: Unknown extension");
    commFile = " test/data/s7k/20141016_150519_FJ-Saucier.s7k";
    commTest = binexec+commFile;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()!="Error while parsing test/data/s7k/20141016_150519_FJ-Saucier.s7k: Unknown extension");
    commFile = " test/data/xtf/example.xtf";
    commTest = binexec+commFile;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()!="Error while parsing test/data/xtf/example.xtf: Unknown extension");
}

TEST_CASE("test if the file is invalid")
{
    string commFile = " test.txt";
    string commTest = binexec+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="Error while parsing test.txt: Unknown extension");
}

TEST_CASE("test if the file is not present")
{
    string commFile = " test/data/all/examplee.all";
    string commTest = binexec+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="Error while parsing test/data/all/examplee.all: File not found");
}

TEST_CASE("test if file parameter is not present")
{
    string commFile = "";
    string commTest = binexec+commFile;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="Error no file enter");
}