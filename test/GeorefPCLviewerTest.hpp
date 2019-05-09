/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   GeorefPCLviewerTest.hpp
 * Author: emile
 *
 * Created on May 9, 2019, 3:30 PM
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
static string GeoDCLBinexec("..\\bin\\georefPCLviewer.exe");
static string GeoDCLoutputdir(".");
#else
static string GeoDCLBinexec("build/bin/georefPCLviewer");
static string GeoDCLOutputdir(".");
#endif

/**
 * Execute a main function
 * 
 * @param command the parameters for the execution
 */
std::stringstream GeoDCLSystem_call(const std::string& command){

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

