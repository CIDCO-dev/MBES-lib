/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   RayTracerAppTest.hpp
 * Author: Jordan McManus
 */

#include "catch.hpp"
#include "../src/georeferencing/DatagramRayTracer.hpp"
#include "../src/svp/SvpNearestByTime.hpp"
#include "../src/svp/CarisSvpFile.hpp"
#include "../src/math/Boresight.hpp"



TEST_CASE("raytracer app test") {
    
    
    
    class TestRayTracer : public DatagramRayTracer {
        public:
            TestRayTracer(SvpSelectionStrategy & svpStrat) : DatagramRayTracer(svpStrat) {
                
            }

            void processRayTracedBeam( Eigen::Vector3d & rayTracedBeam) {
                
                REQUIRE(std::abs(rayTracedBeam(0) - 0) < epsXY);
                REQUIRE(std::abs(rayTracedBeam(1) - 0) < epsXY);
                REQUIRE(std::abs(rayTracedBeam(2) - approxSoundSpeed*oneWayTravelTime) < epsZ);
            }
            
        private:
            double epsXY = 0.001; // 1 mm
            double epsZ = 0.06; // 6 cm
            
            double approxSoundSpeed = 1506.0;
            double oneWayTravelTime = 0.01;
    };
    
    SvpSelectionStrategy * svpStrategy = NULL;
    svpStrategy = new SvpNearestByTime();
    
    TestRayTracer handler(*svpStrategy);
    
    
    //attitudes
    uint64_t timestamp_attitude1 = 1;
    double roll_1 = 0.0;
    double pitch_1 = 0.0;
    double heading_1 = 0.0;
    handler.processAttitude(timestamp_attitude1, heading_1, pitch_1, roll_1);
    
    uint64_t timestamp_attitude2 = 2;
    double roll_2 = 0.0;
    double pitch_2 = 0.0;
    double heading_2 = 0.0;
    handler.processAttitude(timestamp_attitude2, heading_2, pitch_2, roll_2);
    
    //positions
    uint64_t timestamp_position1 = 1;
    double longitude_1 = 0.0;
    double latitude_1 = 0.0;
    double height_1 = 0.0;
    handler.processPosition(timestamp_position1, longitude_1, latitude_1, height_1);
    
    uint64_t timestamp_position2 = 2;
    double longitude_2 = 0.0;
    double latitude_2 = 0.0;
    double height_2 = 0.0;
    handler.processPosition(timestamp_position2, longitude_2, latitude_2, height_2);
    
    // surface sound speed
    double sss = 1486.5;
    handler.processSwathStart(sss);
    
    // beam
    uint64_t timestamp_beam = 1.5;
    long id = 1l;
    double beamAngle = 0.0;
    double tiltAngle = 0.0;
    double oneWayTravelTime = 0.01;
    double twoWayTravelTime = 2*oneWayTravelTime;
    uint32_t quality = 1;
    int32_t intensity = 1;
    handler.processPing(timestamp_beam, id, beamAngle, tiltAngle, twoWayTravelTime, quality, intensity);
    
    // platform parameters
    //Lever arm
    double leverArmX = 0.0;
    double leverArmY = 0.0;
    double leverArmZ = 0.0;
    Eigen::Vector3d leverArm;
    leverArm << leverArmX,leverArmY,leverArmZ;

    //Boresight
    double boresight_roll     = 0.0;
    double boresight_pitch    = 0.0;
    double boresight_heading  = 0.0;
    Attitude boresightAngles(0,boresight_roll,boresight_pitch,boresight_heading);
    Eigen::Matrix3d boresight;
    Boresight::buildMatrix(boresight,boresightAngles);
    
    // SVP
    std::string	svpFilename = "test/data/SVP/SVP.txt";
    CarisSvpFile svps;
    svps.readSvpFile(svpFilename);
    
    
    // raytracing
    
    handler.raytrace(leverArm, boresight, svps.getSvps());
    
    
    
    
    
    delete svpStrategy;
}
