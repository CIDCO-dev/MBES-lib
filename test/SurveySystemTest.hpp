/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SurveySystemTest.cpp
 * Author: glm,jordan
 *
 */

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include "catch.hpp"
#include "../src/SurveySystem.hpp"
#include "../src/utils/Exception.hpp"
#include <cmath>


TEST_CASE("Reading metadata from original MSPAC") {
    SurveySystem params;

    REQUIRE(params.readFile("test/data/metadata/TestMetaData.txt"));
    /*
                MultibeamModel R2Sonic2022
                AntPositionOffsetX 1.31
                AntPositionOffsetY 1.45
                AntPositionOffsetZ 4.32
                MBEOffsetX 0.765
                MBEOffsetY 0.685
                MBEOffsetZ 1.359
                MBEDraft 0.70
                MBEOffset2X 0.364
                MBEOffset2Y 0.687
                MBEOffset2Z 1.093
                PitchRollAccuracy 0.050
                HeadingAccuracy 0.050
                HeaveAccuracy 0.050
                PositionAccuracy 0.010
                PitchAlignment 1.64
                RollAlignment 0.62
                HeadingAlignment 1.88
     */
    
    
    //Test draft
    /*
                MBEDraft 0.70
     */
    double draft = 0.7;
    double draftPrecision = 0.000000001;
    REQUIRE(std::abs(params.getDraft() - draft) < draftPrecision);

    //Test model name
    /*
                MultibeamModel R2Sonic2022
     */
    std::string testModelName = "R2Sonic2022";
    REQUIRE(params.getMBES_model().compare(testModelName) == 0);
    

    //Test antenna position
    /*
                AntPositionOffsetX 1.31
                AntPositionOffsetY 1.45
                AntPositionOffsetZ 4.32
     */
    double antennaPrecision = 0.000000001;
    Eigen::Vector3d testAntennatPosition(1.31, 1.45, -4.32);
    Eigen::Vector3d testAntenna = params.getAntennaPosition().array() - testAntennatPosition.array();
    REQUIRE(testAntenna.cwiseAbs().maxCoeff() < antennaPrecision);
    
    
    //Test echosounder transmitter position
    /*
                MBEOffsetX 0.765
                MBEOffsetY 0.685
                MBEOffsetZ 1.359
     */
    double transmitterPrecision = 0.000000001;
    Eigen::Vector3d testTransmitterPosition(0.765, 0.685, -1.359);
    Eigen::Vector3d testTransmitter = params.getEchosounderTransmitterPosition().array() - testTransmitterPosition.array();
    REQUIRE(testTransmitter.cwiseAbs().maxCoeff() < transmitterPrecision);

    //Test echosounder transmitter position
    /*
                MBEOffset2X 0.364
                MBEOffset2Y 0.687
                MBEOffset2Z 1.093
     */
    double receiverPrecision = 0.000000001;
    Eigen::Vector3d testReceiverPosition;
    testReceiverPosition << 0.364, 0.687, -1.093;
    Eigen::Vector3d testReceiver = params.getEchosounderReceivererPosition().array() - testReceiverPosition.array();
    REQUIRE(testReceiver.cwiseAbs().maxCoeff() < receiverPrecision);

    //Test echosounder transmitter position
    /*
                PitchRollAccuracy 0.050
                HeadingAccuracy 0.050
     */
    double attitudeAccuracyPrecision = 0.000000001;
    double testPitchRollAccuracy = 0.050*D2R;
    Attitude* acc = params.getAttitudeAccuracy();
    REQUIRE(abs(acc->getRoll()*D2R-testPitchRollAccuracy) < attitudeAccuracyPrecision);
    REQUIRE(abs(acc->getPitch()*D2R-testPitchRollAccuracy) < attitudeAccuracyPrecision);
}

TEST_CASE("Reading non-existing file") {
    SurveySystem params;

    REQUIRE(!params.readFile("/non/existant/file"));
}

//TODO: test case with a few invalid values
