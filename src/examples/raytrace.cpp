/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * \author Jordan McManus
 */

#ifndef MAIN_CPP
#define MAIN_CPP


#pragma comment(lib, "Ws2_32.lib")

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "../georeferencing/Raytracing.hpp"
#include "../svp/CarisSvpFile.hpp"
#include "../datagrams/DatagramParserFactory.hpp"
#include "../math/Boresight.hpp"


void printUsage() {
    std::cerr << "\n\
NAME\n\n\
	ray - Produces raytraced vectors for each layer in SVP \n\n\
SYNOPSIS\n \
	ray [-t twoWayTravelTime] [-s surfaceSoundSpeed] [-b acrossTrackAngleDegrees] [-i alongTrackAngleDegrees] [-r roll_angle] [-p pitch_angle] [-h heading_angle] [-e boresight_roll_angle] [-f boresight_pitch_angle] [-g boresight_heading_angle] [-d transducer_depth] svpfile\n\n\
\n \
Copyright 2017-2021 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés" << std::endl;

    exit(1);
}

/**
 * @param argc number of argument
 * @param argv value of the arguments
 */
int main(int argc, char ** argv) {


#ifdef __GNU__
    setenv("TZ", "UTC", 1);
#endif
#ifdef _WIN32
    putenv("TZ");
#endif

    if (argc < 2) {
        printUsage();
    }

    //draft
    double transducerDepth = 0.0;

    //Attitude
    double rollDegrees = 0.0;
    double pitchDegrees = 0.0;
    double headingDegrees = 0.0;

    //Boresight
    double rollBoresightDegrees = 0.0;
    double pitchBoresightDegrees = 0.0;
    double headingBoresightDegrees = 0.0;

    //Ping
    double twoWayTravelTime = 0.0;
    double acrossTrackAngleDegrees = 0.0;
    double alongTrackAngleDegrees = 0.0;
    double surfaceSoundSpeed = 0.0;

    int index;
    while ((index = getopt(argc, argv, "t:s:b:i:r:p:h:d:e:g:f:")) != -1) {
        switch (index) {
            case 't':
                if (sscanf(optarg, "%lf", &twoWayTravelTime) != 1) {
                    std::cerr << "Invalid two way travel time (-t): " << twoWayTravelTime << std::endl;
                    printUsage();
                }
                break;
                
            case 's':
                if (sscanf(optarg, "%lf", &surfaceSoundSpeed) != 1) {
                    std::cerr << "Invalid surface sound speed (-s): " << surfaceSoundSpeed << std::endl;
                    printUsage();
                }
                break;

            case 'b':
                if (sscanf(optarg, "%lf", &acrossTrackAngleDegrees) != 1) {
                    std::cerr << "Invalid acrossTrackAngle angle (-b): " << acrossTrackAngleDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'i':
                if (sscanf(optarg, "%lf", &alongTrackAngleDegrees) != 1) {
                    std::cerr << "Invalid alongTrackAngle angle (-i): " << alongTrackAngleDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'r':
                if (sscanf(optarg, "%lf", &rollDegrees) != 1) {
                    std::cerr << "Invalid roll angle (-r): " << rollDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'h':
                if (sscanf(optarg, "%lf", &headingDegrees) != 1) {
                    std::cerr << "Invalid heading angle (-h): " << headingDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'p':
                if (sscanf(optarg, "%lf", &pitchDegrees) != 1) {
                    std::cerr << "Invalid pitch angle (-p): " << pitchDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'e':
                if (sscanf(optarg, "%lf", &rollBoresightDegrees) != 1) {
                    std::cerr << "Invalid boresight roll angle (-r): " << rollBoresightDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'g':
                if (sscanf(optarg, "%lf", &headingBoresightDegrees) != 1) {
                    std::cerr << "Invalid boresight heading angle (-h): " << headingBoresightDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'f':
                if (sscanf(optarg, "%lf", &pitchBoresightDegrees) != 1) {
                    std::cerr << "Invalid boresight pitch angle (-p): " << pitchBoresightDegrees << std::endl;
                    printUsage();
                }
                break;

            case 'd':
                if (sscanf(optarg, "%lf", &transducerDepth) != 1) {
                    std::cerr << "Invalid transducer depth (-d): " << transducerDepth << std::endl;
                    printUsage();
                }
                break;
        }
    }

    std::string svp_filename(argv[argc - 1]);

    CarisSvpFile carisSvp;
    bool svpRead = carisSvp.readSvpFile(svp_filename);
    SoundVelocityProfile * svp = carisSvp.getSvps()[0];

    // Attitude Matrix
    Attitude attitude(0, rollDegrees, pitchDegrees, headingDegrees);
    Eigen::Matrix3d imu2ned;
    CoordinateTransform::getDCM(imu2ned, attitude);


    //Boresight Matrix
    Attitude boresightAngles(0, rollBoresightDegrees, pitchBoresightDegrees, headingBoresightDegrees);
    Eigen::Matrix3d boresightMatrix;
    Boresight::buildMatrix(boresightMatrix, boresightAngles);


    // Ping
    uint64_t microEpoch = 0;
    long id = 0;
    uint32_t quality = 0;
    double intensity = 0;

    Ping ping(
            microEpoch,
            id,
            quality,
            intensity,
            surfaceSoundSpeed,
            twoWayTravelTime,
            alongTrackAngleDegrees,
            acrossTrackAngleDegrees
            );

    ping.setTransducerDepth(transducerDepth);

    std::vector<Eigen::Vector2d> layerRays;
    std::vector<double> layerTravelTimes;

    Eigen::Vector2d ray;


    Raytracing::planarRayTrace(ray,
            layerRays,
            layerTravelTimes,
            ping,
            *svp,
            boresightMatrix,
            imu2ned);
    
    Eigen::Vector3d rayNED;
    Raytracing::rayTrace(rayNED, ping, *svp, boresightMatrix, imu2ned);
    
    
    // output result
    
    //3D NED ray
    std::cout << rayNED(0) << " " << rayNED(1) << " " << rayNED(2) << std::endl;
    
    //2D planar ray
    std::cout << ray(0) << " " << ray(1) << std::endl;
    
    //radial components per layer
    for(unsigned int i = 0; i<layerRays.size(); i++) {
        std::cout << layerRays[i](0);
        if(i<layerRays.size()-1) {
            std::cout << " ";
        } else {
            std::cout << std::endl;
        }
    }
    
    //depth components per layer
    for(unsigned int i = 0; i<layerRays.size(); i++) {
        std::cout << layerRays[i](1);
        if(i<layerRays.size()-1) {
            std::cout << " ";
        }  else {
            std::cout << std::endl;
        }
    }
}

#endif