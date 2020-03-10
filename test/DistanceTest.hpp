/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   DistanceTest.hpp
 * Author: Christian
 *
 */

#ifndef DISTANCETEST_HPP
#define DISTANCETEST_HPP

#include "catch.hpp"
#include "../src/math/Distance.hpp"


TEST_CASE("DistanceTest.hpp: Haversine test") {
    
    double earthRadius = 6371000;

    double distanceDifferenceTreshold = 1e-3;

    double expectedDistance;
    double functionOutput;

    double latitude1;
    double longitude1;
    double latitude2;
    double longitude2;


    latitude1 = 0;
    longitude1 = -45;

    latitude2 = 0;
    longitude2 = 45;

    expectedDistance = PI / 2 * earthRadius;

    functionOutput = Distance::haversine( longitude1, latitude1, longitude2, latitude2 );
    REQUIRE( std::abs( expectedDistance - functionOutput ) < distanceDifferenceTreshold );


    latitude1 = 0;
    longitude1 = 10;

    latitude2 = 90;
    longitude2 = 10;

    expectedDistance = PI / 2 * earthRadius;

    functionOutput = Distance::haversine( longitude1, latitude1, longitude2, latitude2 );
    REQUIRE( std::abs( expectedDistance - functionOutput ) < distanceDifferenceTreshold );


    latitude1 = -80;
    longitude1 = 100;

    latitude2 = 40;
    longitude2 = 100;

    expectedDistance = 2 * PI / 3 * earthRadius;

    functionOutput = Distance::haversine( longitude1, latitude1, longitude2, latitude2 );
    REQUIRE( std::abs( expectedDistance - functionOutput ) < distanceDifferenceTreshold );


    latitude1 = 48;
    longitude1 = 120.0;

    latitude2 = 48.1;
    longitude2 = 120.2;

    expectedDistance = 18564.787118;    // Value computed in Python with two different methods:
                                        // 1) haversine 
                                        // 2) "From chord length" from https://en.wikipedia.org/wiki/Great-circle_distance

    functionOutput = Distance::haversine( longitude1, latitude1, longitude2, latitude2 );
    REQUIRE( std::abs( expectedDistance - functionOutput ) < distanceDifferenceTreshold );


    latitude1 = 10;
    longitude1 = -20;

    latitude2 = 80;
    longitude2 = 150;

    expectedDistance = 9990991.34780;       // Value computed in Python with two different methods:
                                            // 1) haversine 
                                            // 2) "From chord length" from https://en.wikipedia.org/wiki/Great-circle_distance

    functionOutput = Distance::haversine( longitude1, latitude1, longitude2, latitude2 );
    REQUIRE( std::abs( expectedDistance - functionOutput ) < distanceDifferenceTreshold );


}

#endif /* DISTANCETEST_HPP */

