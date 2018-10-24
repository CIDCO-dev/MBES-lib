/*
 * Copyright 2018 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SystemGeometry.h
 * Author: glm,jordan
 *
 * Created on August 20, 2018, 1:07 PM
 */

#ifndef SYSTEMGEOMETRY_H
#define SYSTEMGEOMETRY_H

#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "utils/Constants.hpp"
#include "utils/Exception.hpp"
#include "Attitude.hpp"

class SurveySystem {
public:
    SurveySystem();
    ~SurveySystem();

    std::string & getMBES_model() {
        return MBES_model;
    }

    Eigen::Vector3d & getAntennaPosition() {
        return antennaPosition;
    }

    Attitude * getAttitudeAccuracy() {
        return attitudeAccuracy;
    }

    Attitude * getBoresightPatchTest() {
        return boresightPatchTest;
    }

    double getDraft() {
        return draft;
    }

    Eigen::Vector3d & getEchosounderReceivererPosition() {
        return echosounderReceivererPosition;
    }

    Eigen::Vector3d & getEchosounderTransmitterPosition() {
        return echosounderTransmitterPosition;
    }

    Eigen::Vector3d & getPositionAccuracy() {
        return positionAccuracy;
    }

    bool readFile(const std::string & fileName);

private:
    std::string MBES_model;

    double draft; // in meters

    // positions in the IMU reference frame in meters
    Eigen::Vector3d antennaPosition;
    Eigen::Vector3d echosounderTransmitterPosition;
    Eigen::Vector3d echosounderReceivererPosition;

    // angles in degrees
    // will be converted internally to radians
    Attitude* boresightPatchTest = NULL;

    // accuracy in 2 sigma
    Attitude* attitudeAccuracy = NULL;
    Eigen::Vector3d positionAccuracy;
};

SurveySystem::SurveySystem() {
}

SurveySystem::~SurveySystem() {
    if (boresightPatchTest) {
        delete boresightPatchTest;
    }

    if (attitudeAccuracy) {
        delete attitudeAccuracy;
    }
}

bool SurveySystem::readFile(const std::string & fileName) {

    // Temporary place holder variables
    std::string NameDevice;
    double Patch_Roll = 0, Patch_Pitch = 0, Patch_Heading = 0;
    double MBES_X = 0, MBES_Y = 0, MBES_Z = 0;
    double MBES_RX = 0, MBES_RY = 0, MBES_RZ = 0;
    double AntX = 0, AntY = 0, AntZ = 0;
    double Draft = 0;

    double PitchRollAcc, HeadingAcc;
    double PosHorAcc, PosVerAcc;

    //double Eroll = 0, Epitch = 0, Eyaw = 0, Rroll = 0, Rpitch = 0, Ryaw = 0, Mroll = 0, Mpitch = 0, Myaw = 0;

    // Open file
    std::string line;
    std::ifstream file(fileName);

    /*
     * This is cancer, no validation of correctness or presence of data
     * TODO:
     * Should return false if data is missing or is invalid
     */
    if (!file) {
        return false;
    } else {
        for (std::string line; getline(file, line);) {
            //make a stream for the line itself
            std::istringstream in(line);

            std::string type;
            in >> type;

            if (type == "MultibeamModel") {
                in >> NameDevice;
            } else if (type == "AntennaPositionOffsetX") {
                in >> AntX;
            } else if (type == "AntennaPositionOffsetY") {
                in >> AntY;
            } else if (type == "AntennaPositionOffsetZ") {
                in >> AntZ;
            } else if (type == "MBETransmitterOffsetX") {
                in >> MBES_X;
            } else if (type == "MBETransmitterOffsetY") {
                in >> MBES_Y;
            } else if (type == "MBETransmitterOffsetZ") {
                in >> MBES_Z;
            } else if (type == "MBEDraft") {
                in >> Draft;
            } else if (type == "MBEReceiverOffsetX") {
                in >> MBES_RX;
            } else if (type == "MBEReceiverOffsetY") {
                in >> MBES_RY;
            } else if (type == "MBEReceiverOffsetZ") {
                in >> MBES_RZ;
            } else if (type == "PositionAccuracy") {
                in >> PosHorAcc;
                PosVerAcc = PosHorAcc * 1.5;
            } else if (type == "PitchRollAccuracy") {
                in >> PitchRollAcc;
            } else if (type == "HeadingAccuracy") {
                in >> HeadingAcc;
            } else if (type == "RollAlignment") {
                in >> Patch_Roll;
            } else if (type == "PitchAlignment") {
                in >> Patch_Pitch;
            } else if (type == "HeadingAlignment") {
                in >> Patch_Heading;
            }/* else if (type == "MBEOffsetR") {
                in >> Eroll;
            } else if (type == "MBEOffsetP") {
                in >> Epitch;
            } else if (type == "MBEOffsetH") {
                in >> Eyaw;
            } else if (type == "MBEOffset2R") {
                in >> Rroll;
            } else if (type == "MBEOffset2P") {
                in >> Rpitch;
            } else if (type == "MBEOffset2H") {
                in >> Ryaw;
            } else if (type == "MotionSensorR") {
                in >> Mroll;
            } else if (type == "MotionSensorP") {
                in >> Mpitch;
            } else if (type == "MotionSensorH") {
                in >> Myaw;
            }*/
        }

        file.close();

        MBES_model = NameDevice;

        draft = Draft;

        antennaPosition << AntX, AntY, -AntZ;
        echosounderTransmitterPosition << MBES_X, MBES_Y, -MBES_Z;
        echosounderReceivererPosition << MBES_RX, MBES_RY, -MBES_RZ;

        boresightPatchTest = new Attitude(Patch_Roll, Patch_Pitch, Patch_Heading);

        attitudeAccuracy = new Attitude(PitchRollAcc, PitchRollAcc, HeadingAcc);

        positionAccuracy << PosHorAcc, PosHorAcc, PosVerAcc;

	return true;
    }
}



#endif /* SYSTEMGEOMETRY_H */
