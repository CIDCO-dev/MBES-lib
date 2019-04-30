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
 * Execute a main function
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

TEST_CASE("test with no parameter")
{
    std::stringstream ss;
    ss = DataSystem_call(std::string(dataBinexec));
    REQUIRE(ss.str()=="Quality filter 0 add\n");
}

TEST_CASE("test with one parameter")
{
    string param = " 19";
    std::stringstream ss;
    ss = DataSystem_call(std::string(dataBinexec+param));
    REQUIRE(ss.str()=="Quality filter 19 add\n");
}

TEST_CASE("test with multiple parameter")
{
    string param = " 19 16 12";
    std::stringstream ss;
    ss = DataSystem_call(std::string(dataBinexec+param));
    string line;
    getline(ss,line);
    REQUIRE(line=="Quality filter 19 add");
    getline(ss,line);
    REQUIRE(line=="Quality filter 16 add");
    getline(ss,line);
    REQUIRE(line=="Quality filter 12 add");
}
