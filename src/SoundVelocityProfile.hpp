/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   SoundVelocityProfile.h
 * Author: glm,jordan
 *
 * Created on August 22, 2018, 10:30 AM
 */

#ifndef SOUNDVELOCITYPROFILE_HPP
#define SOUNDVELOCITYPROFILE_HPP

#include <list>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>

class SoundVelocityProfile {
public:

    SoundVelocityProfile();

    ~SoundVelocityProfile();

    Eigen::VectorXd & getDepth() {
        return depth;
    };

    Eigen::VectorXd & getGradient() {
        return gradient;
    };

    Eigen::VectorXd & getSoundSpeed() {
        return soundSpeed;
    };

    unsigned int getSize() {
        return size;
    };

    bool readFile(const std::string fileName, double draft);

private:
    unsigned int size;
    Eigen::VectorXd depth;
    Eigen::VectorXd soundSpeed;
    Eigen::VectorXd gradient;
};

SoundVelocityProfile::SoundVelocityProfile() {
};

SoundVelocityProfile::~SoundVelocityProfile() {
};

bool SoundVelocityProfile::readFile(const std::string fileName, double draft) {

    std::vector<double> depths;
    std::vector<double> speeds;

    // Open file
    int lineCounter = 0;
    int draftIndex = -1;
    std::string line;
    std::ifstream inputFile(fileName);

    if (!inputFile) {
        std::cerr << "Couldn't read SVP file: " << fileName << std::endl;
        return false;
    } else {

        while (std::getline(inputFile, line)) {
            double depth, speed;

            if (sscanf(line.c_str(), "%lf %lf", &depth, &speed) == 2) {
                if (-depth < draft) {
                    draftIndex++;
                }

                depths.push_back(-depth - draft);
                speeds.push_back(speed);
            } else {
                std::cerr << "Line #" << lineCounter << ": Invalid svp data " << line.c_str() << std::endl;
            }
            lineCounter++;
        }

    }

    if ((depths.size() != speeds.size()) && depths.size() < 2) {
        std::cerr << "Invalid sound speed data file" << std::endl;
        return false;
    }

    unsigned int n = speeds.size(); // depths.size() and speeds.size() are equal
    unsigned int k = 0;

    if (draftIndex != -1) {
        // SVP interpolation for draft
        double cGradient = (speeds[draftIndex + 1] - speeds[draftIndex]) / (depths[draftIndex + 1] - depths[draftIndex]);
        double cDraft = speeds[draftIndex] + cGradient * (0 - depths[draftIndex]); //0-depths[draftIndex] = z_i - z_0 (where z_i = 0)

        n = n - draftIndex;

        if (n < 2) {
            std::cerr << "Sound speed data is incompatible with draft." << std::endl;
            return false;
        }

        soundSpeed.resize(n);
        depth.resize(n);
        soundSpeed(0) = cDraft;
        depth(0) = 0;
        k = draftIndex + 1; //change k to ignore data above draft
    } else {
        // no SVP interpolation for draft
        soundSpeed.resize(n);
        depth.resize(n);
    }

    size = n;

    // k won't be 0 if svp is draft corrected
    for (; k < n; k++) {
        soundSpeed(k) = speeds[k];
        depth(k) = depths[k];
    }

    Eigen::VectorXd celerityDifferences = soundSpeed.tail(n - 1) - soundSpeed.head(n - 1);
    Eigen::VectorXd depthDifferences = depth.tail(n - 1) - depth.head(n - 1);

    gradient = celerityDifferences.cwiseQuotient(depthDifferences);

    return true;
};

#endif /* SOUNDVELOCITYPROFILE_HPP */
