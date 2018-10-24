#ifndef NMEAUTILSTEST_HPP
#define NMEAUTILSTEST_HPP

#include "catch.hpp"
#include "../src/utils/NmeaUtils.hpp"

#define DOUBLE_PRECISION 0.0000000000001

TEST_CASE( "Extract height from GGK") {
	std::string ggk("GPGGK,135900.907,090916,4822.32065998,N,00429.55086871,W,3,10,1.8,EHT50.886,M*55");

	double height=NmeaUtils::extractHeightFromGGK(ggk);

	REQUIRE(!std::isnan(height));
	REQUIRE(abs(height - 50.886) < DOUBLE_PRECISION);
}

TEST_CASE( "Extract height from invalid GGK") {
        std::string ggk("I am the walrus");

        double height=NmeaUtils::extractHeightFromGGK(ggk);

        REQUIRE(std::isnan(height));
}

TEST_CASE("Extract height from GGA"){
	std::string gga("GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F");

        double height=NmeaUtils::extractHeightFromGGA(gga);

        REQUIRE(!std::isnan(height));
        REQUIRE(abs(height - (-6.776)) < DOUBLE_PRECISION);

}

TEST_CASE( "Extract height from invalid GGA") {
        std::string gga("I am the walrus");

        double height=NmeaUtils::extractHeightFromGGA(gga);

        REQUIRE(std::isnan(height));
}


#endif

