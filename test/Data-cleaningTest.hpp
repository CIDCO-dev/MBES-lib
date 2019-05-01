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

/**Test when there is no parameter*/
TEST_CASE("test with no parameter")
{
    string output = "cat test/data/dataCleanTest.txt | ./";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(quality>=0);
        }
    }
}

/**Test when the quality parameter is enter*/
TEST_CASE("test with the quality parameter")
{
    string output = "cat test/data/dataCleanTest.txt | ./";
    string param = " -q 190000";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(quality>=190000);
        }
    }
}

/**Test when the intensity parameter is enter*/
TEST_CASE("test with the intensity parameter")
{
    string output = "cat test/data/dataCleanTest.txt | ./";
    string param = " -i 1800";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(intensity>=1800);
        }
    }
}

/**Test when there is a invalid quality parameter*/
TEST_CASE("test with invalid quality parameter")
{
    string output = "cat test/data/dataCleanTest.txt | ./";
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
    string output = "cat test/data/dataCleanTest.txt | ./";
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
    string output = "cat test/data/dataCleanTest.txt | ./";
    string param = " -q 19000 -i 1800";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)==6)
        {
            REQUIRE(quality>=19000);
            REQUIRE(intensity>=1800);
        }
    }
}

/**Test when there is a invalid line input*/
TEST_CASE("test with invalid line input")
{
    string output = "cat test/data/dataCleanTest.txt | ./";
    string param = " -q 0 2>&1";
    std::stringstream ss;
    ss = DataSystem_call(std::string(output+dataBinexec+param));
    string line;
    uint64_t microEpoch;
    double x,y,z;
    uint32_t quality;
    uint32_t intensity;
    while (getline(ss,line))
    {
        if(sscanf(line.c_str(),"%lu %lf %lf %lf %d %d",&microEpoch,&x,&y,&z,&quality,&intensity)!=6)
        {
            int linecount;
            REQUIRE(sscanf(line.c_str(),"Error at line %d",&linecount)==1);
        }
    }
}