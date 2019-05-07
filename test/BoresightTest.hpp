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
    double ch = cos(3*D2R);
    double sh = sin(3*D2R);
    double cp = cos(2*D2R);
    double sp = sin(2*D2R);
    double cr = cos(1*D2R);
    double sr = sin(1*D2R);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << ch*cp , ch*sp*sr - sh*cr  , ch*sp*cr + sh*sr,
                    sh*cp , sh*sp*sr + ch*cr  , sh*sp*cr - ch*sr,
                    -sp   , cp*sr             , cp*cr;
    REQUIRE(boresight == matrixSearch);
}
