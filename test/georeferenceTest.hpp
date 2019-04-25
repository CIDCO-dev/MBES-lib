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

TEST_CASE("test")
{
    string commX = " -x 1.0";
    string commY = " -y 1.0";
    string commZ = " -z 1.0";
    string commFile = " test/data/all/example.all";
    string commTest = binexec+commFile+commX+commY+commZ;
    std::stringstream ss;
    ss = system_call(std::string(commTest));
    REQUIRE(ss.str()=="0");
}
