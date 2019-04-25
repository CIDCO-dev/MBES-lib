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
#include "catch.hpp"
#include "../src/examples/georeference.cpp"
#include "../src/utils/Exception.hpp"

TEST_CASE("test")
{
    REQUIRE(1==2);
}
