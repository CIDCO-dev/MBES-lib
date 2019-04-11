/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SoundVelocityProfileTest.cpp
 * Author: jordan
 *
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <iomanip>
#include "catch.hpp"
#include "../src/SoundVelocityProfile.hpp"
#include "../src/utils/Exception.hpp"

/**Test the writing and the reading of a SVP file*/
TEST_CASE("Writing to SVP file"){
	//TODO: test prototype
    SoundVelocityProfile svp = SoundVelocityProfile(0,0,0);
    svp.add(0,0);
    svp.write("test");
    std::string cont;
    std::string contAttendu;
    contAttendu << "[SVP_VERSION_2]" << "\r\n";
    contAttendu << "test" << "\r\n";
    contAttendu << "Section " << "0-0 0:0:0" << " " << "0:0:0" << " " << "0:0:0" << "\r\n" ;
    contAttendu << "0 0";
    cont = svp.read("test");
	REQUIRE(contAttendu = cont);
};

TEST_CASE("Get speeds/depths"){
	//TODO: test prototype
	REQUIRE(1==2);
}

