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

/*!
 * \brief Survey system class
 */
class SurveySystem {
public:
    
    /**Create a Survey system*/
    SurveySystem();
    
    /**Destroy the Survey system*/
    ~SurveySystem();

    /**Return the MBES model*/
    std::string & getMBES_model() {
        return MBES_model;
    }

    /**Return the antenna position*/
    Eigen::Vector3d & getAntennaPosition() {
        return antennaPosition;
    }

    /**Return the attitude accuracy*/
    Attitude * getAttitudeAccuracy() {
        return attitudeAccuracy;
    }

    /**Return the boresigth patch test*/
    Attitude * getBoresightPatchTest() {
        return boresightPatchTest;
    }

    /**Return the draft*/
    double getDraft() {
        return draft;
    }

    /**Return the echo sounder receiver position*/
    Eigen::Vector3d & getEchosounderReceivererPosition() {
        return echosounderReceivererPosition;
    }

    /**Return the echo sounder transmitter position*/
    Eigen::Vector3d & getEchosounderTransmitterPosition() {
        return echosounderTransmitterPosition;
    }

    /**Return the position accuracy*/
    Eigen::Vector3d & getPositionAccuracy() {
        return positionAccuracy;
    }

    /**
     * Change the Survey system values by reading a file
     * Return false if the file is not valid
     * 
     * @param filename name of the file who will be read
     */
    bool readFile(const std::string & fileName);

private:
    
    /**Name of the Survey system MBES model*/
    std::string MBES_model;

    /**Value of the Survey system draft (meter)*/
    double draft; // in meters

    // positions in the IMU reference frame in meters
    
    /**Vector3d of the Survey system antenna position*/
    Eigen::Vector3d antennaPosition;
    
    /**Vector3d of the Survey system echo sounder transmitter position*/
    Eigen::Vector3d echosounderTransmitterPosition;
    
    /**Vector3d of the Survey system echo sounder receiver position*/
    Eigen::Vector3d echosounderReceivererPosition;

    // angles in degrees
    // will be converted internally to radians
    /**Value of the Survey system boresigth path test (attitude)*/
    Attitude* boresightPatchTest = NULL;

    // accuracy in 2 sigma
    /**Value of the Survey system attitude accuracy (attitude)*/
    Attitude* attitudeAccuracy = NULL;
    
    /**Vector3d of the Survey system position accuracy*/
    Eigen::Vector3d positionAccuracy;
};

/**Create a Survey system*/
SurveySystem::SurveySystem() {
}

/**Destroy the Survey system and the boresigth path test and attitude accuracy*/
SurveySystem::~SurveySystem() {
    if (boresightPatchTest) {
        delete boresightPatchTest;
    }

    if (attitudeAccuracy) {
        delete attitudeAccuracy;
    }
}

/**
 * Change the Survey system values by reading a file
 * Return false if the file is not valid
 * 
 * @param filename name of the file who will be read
 */
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

        boresightPatchTest = new Attitude(0,Patch_Roll, Patch_Pitch, Patch_Heading);

        attitudeAccuracy = new Attitude(0,PitchRollAcc, PitchRollAcc, HeadingAcc);

        positionAccuracy << PosHorAcc, PosHorAcc, PosVerAcc;

	return true;
    }
}



#endif /* SYSTEMGEOMETRY_H */
