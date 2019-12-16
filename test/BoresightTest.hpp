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


#include <Eigen/Dense>

#include "catch.hpp"
#include "../src/math/Boresight.hpp"
#include "../src/utils/Constants.hpp"

/*Test the Boresight::buildMatrix with the angles 0,0,0*/
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

/*Test the Boresight::buildMatrix with the angles 1,2,3*/
TEST_CASE("Test with the angle 1,2,3")
{
    Attitude boresightAngles(0,1,2,3);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << 0.998021196624068,-0.0517197397456515,0.0357597484569552,
                    0.0523040745924708,0.998509315434203,-0.0156022681730626,
                    -0.0348994967025010,0.0174417749028302,0.999238614955483;
    REQUIRE(abs(boresight(0,0)-matrixSearch(0,0))< 1e-10);
    REQUIRE(abs(boresight(0,1)-matrixSearch(0,1))< 1e-10);
    REQUIRE(abs(boresight(0,2)-matrixSearch(0,2))< 1e-10);
    REQUIRE(abs(boresight(1,0)-matrixSearch(1,0))< 1e-10);
    REQUIRE(abs(boresight(1,1)-matrixSearch(1,1))< 1e-10);
    REQUIRE(abs(boresight(1,2)-matrixSearch(1,2))< 1e-10);
    REQUIRE(abs(boresight(2,0)-matrixSearch(2,0))< 1e-10);
    REQUIRE(abs(boresight(2,1)-matrixSearch(2,1))< 1e-10);
    REQUIRE(abs(boresight(2,2)-matrixSearch(2,2))< 1e-10);
}

/*Test the Boresight::buildMatrix with the angles 10,26,39*/
TEST_CASE("Test with the angle 10,26,39")
{
    Attitude boresightAngles(0,10,26,39);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << 0.698494163262967,-0.560601422747957,0.444783035535905,
                    0.565629420690258,0.813244715626515,0.136734746700144,
                    -0.438371146789077,0.156073948237737,0.885139345156633;
    REQUIRE(abs(boresight(0,0)-matrixSearch(0,0))< 1e-10);
    REQUIRE(abs(boresight(0,1)-matrixSearch(0,1))< 1e-10);
    REQUIRE(abs(boresight(0,2)-matrixSearch(0,2))< 1e-10);
    REQUIRE(abs(boresight(1,0)-matrixSearch(1,0))< 1e-10);
    REQUIRE(abs(boresight(1,1)-matrixSearch(1,1))< 1e-10);
    REQUIRE(abs(boresight(1,2)-matrixSearch(1,2))< 1e-10);
    REQUIRE(abs(boresight(2,0)-matrixSearch(2,0))< 1e-10);
    REQUIRE(abs(boresight(2,1)-matrixSearch(2,1))< 1e-10);
    REQUIRE(abs(boresight(2,2)-matrixSearch(2,2))< 1e-10);
}

/*Test the Boresight::buildMatrix with the angles 56,67,91*/
TEST_CASE("Test with the angle 56,67,91")
{
    Attitude boresightAngles(0,56,67,91);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    Eigen::Matrix3d matrixSearch;
    matrixSearch << -0.00681919846209329,-0.572426244871090,0.819927858238222,
                    0.390671618245856,0.753257618534275,0.529130084967049,
                    -0.920504853452440,0.323930786284440,0.218494074216318;
    REQUIRE(abs(boresight(0,0)-matrixSearch(0,0))< 1e-10);
    REQUIRE(abs(boresight(0,1)-matrixSearch(0,1))< 1e-10);
    REQUIRE(abs(boresight(0,2)-matrixSearch(0,2))< 1e-10);
    REQUIRE(abs(boresight(1,0)-matrixSearch(1,0))< 1e-10);
    REQUIRE(abs(boresight(1,1)-matrixSearch(1,1))< 1e-10);
    REQUIRE(abs(boresight(1,2)-matrixSearch(1,2))< 1e-10);
    REQUIRE(abs(boresight(2,0)-matrixSearch(2,0))< 1e-10);
    REQUIRE(abs(boresight(2,1)-matrixSearch(2,1))< 1e-10);
    REQUIRE(abs(boresight(2,2)-matrixSearch(2,2))< 1e-10);
}
