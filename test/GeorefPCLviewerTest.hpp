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
static string GeoPCLBinexec("..\\bin\\georefPCLviewer.exe");
static string GeoPCLoutputdir(".");
#else
static string GeoPCLBinexec("build/bin/georefPCLviewer");
static string GeoPCLOutputdir(".");
#endif

/**
 * Execute a main function
 * 
 * @param command the parameters for the execution
 */
std::stringstream GeoPCLSystem_call(const std::string& command){

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

TEST_CASE("test the extention of the file receive with PCL viewer")
{
    string commFile = " test/data/all/0008_20160909_135801_Panopee.all 2>&1";
    string commTest = GeoPCLBinexec+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    std::string line;
    getline(ss,line);
    getline(ss,line);
    REQUIRE(line!="Error while parsing test/data/all/example.all: Unknown extension");
    commFile = " test/data/s7k/20141016_150519_FJ-Saucier.s7k 2>&1";
    commTest = GeoPCLBinexec+commFile;
    ss = GeoPCLSystem_call(std::string(commTest));
    getline(ss,line);
    getline(ss,line);
    REQUIRE(line!="Error while parsing test/data/s7k/20141016_150519_FJ-Saucier.s7k: Unknown extension");
    commFile = " test/data/xtf/example.xtf 2>&1";
    commTest = GeoPCLBinexec+commFile;
    ss = GeoPCLSystem_call(std::string(commTest));
    getline(ss,line);
    getline(ss,line);
    REQUIRE(line!="Error while parsing test/data/xtf/example.xtf: Unknown extension");
}

/**Test with file extention invalid*/
TEST_CASE("test if the file is invalid with PCL viewer")
{
    string commFile = " -L test/data/badextension.bad 2>&1";
    string commTest = GeoPCLBinexec+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    string line;
    getline(ss,line);
    getline(ss,line);
    REQUIRE(line=="Error while parsing test/data/badextension.bad: Unknown extension");
}

/**Test with no file*/
TEST_CASE("test if the file is not present with PCL viewer")
{
    string commFile = " -L test/data/all/examplee.all 2>&1";
    string commTest = GeoPCLBinexec+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    string line;
    getline(ss,line);
    getline(ss,line);
    REQUIRE(line=="Error while parsing test/data/all/examplee.all: File not found");
}

/**Test with no existent file*/
TEST_CASE("test if file parameter is not present with PCL viewer")
{
    string commFile = " 2>&1";
    string commTest = GeoPCLBinexec+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    std::string result = "\n\
  NAME\n\n\
     georeference - Produit un nuage de points d'un fichier de datagrammes multifaisceaux\n\n\
  SYNOPSIS\n \
	   georeference [-x lever_arm_x] [-y lever_arm_y] [-z lever_arm_z] [-r roll_angle] [-p pitch_angle] [-h heading_angle] fichier\n\n\
  DESCRIPTION\n \
	   -L Use a local geographic frame (NED)\n \
	   -T Use a terrestrial geographic frame (WGS84 ECEF)\n\n \
  Copyright 2017-2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés\n";
    REQUIRE(ss.str()==result);
}

/**Test with parameter x y z invalid*/
TEST_CASE("test if the parameter x y z are invalid with PCL viewer")
{
    string commX = " -x sjdhsd";
    string commY = " -y gyhgj";
    string commZ = " -z gyigkb";
    string commFile = " test/data/all/0008_20160909_135801_Panopee.all 2>&1";
    string commTest = GeoPCLBinexec+commX+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    std::string line;
    getline(ss,line);
    REQUIRE(line=="Invalid lever arm X offset (-x)");
    commTest = GeoPCLBinexec+commY+commFile;
    ss = GeoPCLSystem_call(std::string(commTest));
    getline(ss,line);
    REQUIRE(line=="Invalid lever arm Y offset (-y)");
    commTest = GeoPCLBinexec+commZ+commFile;
    ss = GeoPCLSystem_call(std::string(commTest));
    getline(ss,line);
    REQUIRE(line=="Invalid lever arm Z offset (-z)");
}

/**Test with parameter r h p invalid*/
TEST_CASE("test if parameter r h p are invalid with PCL viewer")
{
    string commp = " -r sjdhsd";
    string commP = " -h gyhgj";
    string commt = " -p gyigkb";
    string commFile = " test/data/all/0008_20160909_135801_Panopee.all 2>&1";
    string commTest = GeoPCLBinexec+commp+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    std::string line;
    getline(ss,line);
    REQUIRE(line=="Invalid roll angle offset (-r)");
    commTest = GeoPCLBinexec+commP+commFile;
    ss = GeoPCLSystem_call(std::string(commTest));
    getline(ss,line);
    REQUIRE(line=="Invalid heading angle offset (-h)");
    commTest = GeoPCLBinexec+commt+commFile;
    ss = GeoPCLSystem_call(std::string(commTest));
    getline(ss,line);
    REQUIRE(line=="Invalid pitch angle offset (-p)");
}

TEST_CASE("test if parameter L T are not present with PCL viewer")
{
    string commFile = " test/data/all/0008_20160909_135801_Panopee.all 2>&1";
    string commTest = GeoPCLBinexec+commFile;
    std::stringstream ss;
    ss = GeoPCLSystem_call(std::string(commTest));
    string line;
    getline(ss,line);
    REQUIRE(line=="No georeferencing method defined (-L or -T)");
}