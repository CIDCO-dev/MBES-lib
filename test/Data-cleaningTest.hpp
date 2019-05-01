/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   data-cleaningTest.hpp
 * Author: emile
 *
 * Created on April 29, 2019, 2:22 PM
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include "catch.hpp"
#include "../src/utils/Exception.hpp"
using namespace std;
#ifdef _WIN32
static string DataBinexec("..\\bin\\data-cleaning.exe");
static string outputdir(".");
#else
static string dataBinexec("build/bin/data-cleaning");
static string dataOutputdir(".");
#endif

/**
 * Execute the data-cleaning main function
 * 
 * @param command the parameters for the execution
 */
std::stringstream DataSystem_call(const std::string& command){

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

/**Test the insane position filter*/
TEST_CASE("test insane position filter")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    int lineCount = 0;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(x<=1.00*100000000);
            REQUIRE(x>=-1.00*100000000);
            REQUIRE(y<=1.00*100000000);
            REQUIRE(y>=-1.00*100000000);
            REQUIRE(z<=1.00*100000000);
            REQUIRE(z>=-1.00*100000000);
            lineCount = lineCount+1;
        }
    }
    REQUIRE(lineCount>0);
}

/**Test when the quality parameter is enter*/
TEST_CASE("test with the quality parameter")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -q 8";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    int lineCount = 0;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(quality>=8);
            lineCount = lineCount+1;
        }
    }
    REQUIRE(lineCount>0);
}

/**Test when the intensity parameter is enter*/
TEST_CASE("test with the intensity parameter")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -i 9";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    int lineCount = 0;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(intensity>=9);
            lineCount = lineCount+1;
        }
    }
    REQUIRE(lineCount>0);
}

/**Test when there is a invalid quality parameter*/
TEST_CASE("test with invalid quality parameter")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -q oio 2>&1";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    getline(ss,line);
    REQUIRE(line=="Error: parameter QualityFilter invalid");
}

/**Test when there is a invalid intensity parameter*/
TEST_CASE("test with invalid intensity parameter")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -i oek 2>&1";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    getline(ss,line);
    REQUIRE(line=="Error: parameter IntensityFilter invalid");
}

/**Test when there is multiple parameter*/
TEST_CASE("test with multiple character parameter")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -q 9 -i 10";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    int lineCount = 0;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(quality>=9);
            REQUIRE(intensity>=10);
            lineCount = lineCount+1;
        }
    }
    REQUIRE(lineCount>0);
}

/**Test when there is a invalid line input*/
TEST_CASE("test with invalid line input")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -q 0 2>&1";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    int lineCount = 0;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)!=6)
        {
            int count;
            REQUIRE(sscanf(line.c_str(),"Error at line %d",&count)==1);
            lineCount = lineCount+1;
        }
    }
    REQUIRE(lineCount>0);
}

/**Test when all the lines do not respect the filters*/
TEST_CASE("test with no lines who respect the filters")
{
    std::ifstream inFile;
    inFile.open("test/data/dataCleanTest.dat");
    REQUIRE(inFile);
    string output = "cat test/data/dataCleanTest.dat | ./";
    string param = " -q 100 -i 100";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    int lineCount = 0;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            lineCount = lineCount+1;
        }
    }
    REQUIRE(lineCount==0);
}