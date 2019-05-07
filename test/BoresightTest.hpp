/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BoresightTest.hpp
 * Author: emile
 *
 * Created on May 7, 2019, 9:49 AM
 */

#include <eigen3/Eigen/src/Core/Matrix.h>

#include "catch.hpp"
#include "../src/math/Boresight.hpp"
#include "../src/utils/Constants.hpp"

TEST_CASE("Test if the angles 0,0,0 return the matrix identity")
{
    Attitude boresightAngles(0,0,0,0);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << 1,0,0,
                    0,1,0,
                    0,0,1;
    REQUIRE(boresight == matrixSearch);
}

TEST_CASE("Test with the angle 1,2,3")
{
    Attitude boresightAngles(0,1,2,3);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << 0.998021196624068,-0.0517197397456515,0.0357597484569552,
                    0.0523040745924708,0.998509315434203,-0.0156022681730626,
                    -0.0348994967025010,0.0174417749028302,0.999238614955483;
    REQUIRE(boresight == matrixSearch);
}

TEST_CASE("Test with the angle 10,26,39")
{
    Attitude boresightAngles(0,10,26,39);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << 0.698494163262967,-0.560601422747957,0.444783035535905,
                    0.565629420690258,0.813244715626515,0.136734746700144,
                    -0.438371146789077,0.156073948237737,0.885139345156633;
    REQUIRE(boresight == matrixSearch);
}
