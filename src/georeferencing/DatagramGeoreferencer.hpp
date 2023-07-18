/*
 * Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#ifndef DATAGRAMGEOREFERENCER_HPP
#define DATAGRAMGEOREFERENCER_HPP

#include "../Ping.hpp"
#include "../Position.hpp"
#include "../Attitude.hpp"
#include "Georeferencing.hpp"
#include "../svp/SoundVelocityProfileFactory.hpp"
#include "../svp/SoundVelocityProfile.hpp"
#include "../svp/SvpSelectionStrategy.hpp"
#include "../datagrams/DatagramEventHandler.hpp"
#include "../math/Interpolation.hpp"
#include "../math/CartesianToGeodeticFukushima.hpp"
#include "../sbet/SbetProcessor.hpp"

/*!
 * \brief Datagram Georeferencer class.
 * \author Guillaume Labbe-Morissette, Jordan McManus, Emile Gagne, Patrick Charron-Morneau
 *
 * Extention of the DatagramEventHandler class
 */
class DatagramGeoreferencer : public DatagramEventHandler, public SbetProcessor {
public:

    /**Create a datagram georeferencer*/
    DatagramGeoreferencer(Georeferencing & geo, SvpSelectionStrategy & svpStrat, std::string sbetFile = "") : georef(geo), svpStrategy(svpStrat), sbetFilePath(sbetFile) {

    }

    /**Destroy the datagram georeferencer*/
    ~DatagramGeoreferencer() {

    }

    /**
     * Add the information of a attitude in the vector attitudes
     * 
     * @param microEpoch the attitude timestamp
     * @param heading the attitude heading
     * @param pitch the attitude pitch
     * @param roll the attitude roll
     */
    void processAttitude(uint64_t microEpoch, double heading, double pitch, double roll) {
        attitudes.push_back(Attitude(microEpoch, roll, pitch, heading));
    };

    /**
     * Add the information of a position in the vector positions
     * 
     * @param microEpoch the position timestamp
     * @param longitude the position longitude
     * @param latitude the position latitude
     * @param height the position ellipsoidal height
     */
    void processPosition(uint64_t microEpoch, double longitude, double latitude, double height) {
        positions.push_back(Position(microEpoch, latitude, longitude, height));
    };

    /**
     * Add the information of a ping in the vector pings
     * 
     * @param microEpoch the ping timestamp
     * @param id the ping id
     * @param beamAngle the ping beam angle
     * @param tiltAngle the ping tilt angle
     * @param twoWayTravelTime the ping two way travel time
     * @param quality the ping quality
     * @param intensity the ping intensity
     */
    void processPing(uint64_t microEpoch, long id, double beamAngle, double tiltAngle, double twoWayTravelTime, uint32_t quality, int32_t intensity) {
        pings.push_back(Ping(microEpoch, id, quality, intensity, currentSurfaceSoundSpeed, twoWayTravelTime, tiltAngle, beamAngle));
    };

    /**
     * Change the current surface sound speed
     * 
     * @param surfaceSoundSpeed the new current surface sound speed
     */
    void processSwathStart(double surfaceSoundSpeed) {
        currentSurfaceSoundSpeed = surfaceSoundSpeed;
    };

    /**
     * Add a sound velocity profile in the vector svp
     * 
     * @param svp the sound velocity profile
     */
    void processSoundVelocityProfile(SoundVelocityProfile * svp) {
        svps.push_back(svp);
    }
    
    void processEntry(SbetEntry * entry){
			
		Position position(static_cast<uint64_t>(entry->time * 1000000), entry->latitude*R2D, entry->longitude*R2D, entry->altitude);			
		this->positions.push_back(position);
			
		Attitude attitude(static_cast<uint64_t>(entry->time * 1000000), entry->roll*R2D, entry->pitch*R2D, entry->heading*R2D);
		this->attitudes.push_back(attitude);
    }

    /**
     * Georeferences all pings
     * @param boresight boresight (dPhi,dTheta,dPsi)
     */
    virtual void georeference(Eigen::Vector3d & leverArm, Eigen::Matrix3d & boresight, std::vector<SoundVelocityProfile*> & externalSvps) {
    
    if(this->sbetFilePath.size() > 0){
    	positions.clear();
    	attitudes.clear();
    	readFile(this->sbetFilePath);   	
    }
    
	if(positions.size()==0){
		std::cerr << "[-] No position data found in file" << std::endl;
		return;
	}

        if(attitudes.size()==0){
                std::cerr << "[-] No attitude data found in file" << std::endl;
                return;
        }

        if(pings.size()==0){
                std::cerr << "[-] No ping data found in file" << std::endl;
                return;
        }



        if (externalSvps.size() > 0) {
            //Use svps specified by user
            for (unsigned int i = 0; i < externalSvps.size(); ++i) {
                svpStrategy.addSvp(externalSvps[i]);
            }

            std::cerr << "[+] Using SVP file" << std::endl;
        } else if(svps.size() > 0) {
            //Use svps contained inside sonar file
            for (unsigned int i = 0; i < svps.size(); ++i) {
                svpStrategy.addSvp(svps[i]);
            }

            std::cerr << "[+] Using SVP from sonar file" << std::endl;
        } else {
            //Default to fresh water
            svpStrategy.addSvp(SoundVelocityProfileFactory::buildFreshWaterModel());
            std::cerr << "[+] Using default SVP model" << std::endl;
        }

        //If no centroid defined for LGF georeferencing, compute one
        if (GeoreferencingLGF * lgf = dynamic_cast<GeoreferencingLGF*> (&georef)) {
            if (lgf->getCentroid() == NULL) {
                Position centroid(0, 0, 0, 0);

                for (auto i = positions.begin(); i != positions.end(); i++) {
                    centroid.getVector() += i->getVector();
                }

                centroid.getVector() /= (double) positions.size();

                lgf->setCentroid(centroid);

                std::cerr << "[+] Centroid: " << centroid << std::endl;
            }
        }

        //Sort everything
        std::sort(positions.begin(), positions.end(), &Position::sortByTimestamp);
        std::sort(attitudes.begin(), attitudes.end(), &Attitude::sortByTimestamp);
        std::sort(pings.begin(), pings.end(), &Ping::sortByTimestamp);

        // fprintf(stderr, "[+] Position data points: %ld [%lu to %lu]\n", positions.size(), positions[0].getTimestamp(), positions[positions.size() - 1].getTimestamp());
        // fprintf(stderr, "[+] Attitude data points: %ld [%lu to %lu]\n", attitudes.size(), attitudes[0].getTimestamp(), attitudes[attitudes.size() - 1].getTimestamp());
        // fprintf(stderr, "[+] Ping data points: %ld [%lu to %lu]\n", pings.size(), (pings.size() > 0) ? pings[0].getTimestamp() : 0, (pings.size() > 0) ? pings[pings.size() - 1].getTimestamp() : 0);


		if(this->sbetFilePath.size() > 0){
			uint64_t attitudeTimestamp = attitudes.at(0).getTimestamp();
			uint64_t positionTimestamp = positions.at(0).getTimestamp();
			uint64_t pingTimestamp = pings.at(0).getTimestamp();
			
			std::cerr << "First ping timestamp : " << pingTimestamp <<"\n";
			
			if( attitudeTimestamp == positionTimestamp ){
				
				// unix time 1st jan 1970 is a thursday
				// gps first day of the week starts on sundays
				// 4 day difference : 60 *60 *24 *4 = 345600 seconds
				uint64_t offset = 345600000000;
				
				uint64_t nbMicroSecondsPerWeek = 604800000000; // microseconds per week
				
				//std::cerr<<"micro sec per week : " << nbMicroSecondsPerWeek <<"\n";
				
				uint64_t nbWeek = (pingTimestamp + (nbMicroSecondsPerWeek - offset)) / nbMicroSecondsPerWeek; //nb week unix time
				
				//std::cerr<<"nb week : " << nbWeek << "\n";
				
				uint64_t startOfWeek = (nbMicroSecondsPerWeek * nbWeek) - offset; // micro sec from unix time to start of week
				
				for (auto i = pings.begin(); i != pings.end(); i++) {
					(*i).setTimestamp((*i).getTimestamp() - startOfWeek);
				}
			}
		}

        // For correct display of timestamps on Windows
        std::cerr <<  "[+] Position data points: " << positions.size() << " [" << positions[0].getTimestamp() << " to " 
                << positions[positions.size() - 1].getTimestamp() << "]\n";
        std::cerr <<  "[+] Attitude data points: " << attitudes.size() << " [" << attitudes[0].getTimestamp() << " to " 
                << attitudes[attitudes.size() - 1].getTimestamp() << "]\n";      
        std::cerr <<  "[+] Ping data points: " << pings.size() << " [" << ( (pings.size() > 0) ? pings[0].getTimestamp() : 0 ) << " to " 
                << ( (pings.size() > 0) ? pings[pings.size() - 1].getTimestamp() : 0 ) << "]\n";                            

        //interpolate attitudes and positions around pings
        unsigned int attitudeIndex = 0;
        unsigned int positionIndex = 0;

        //Georef pings
        for (auto i = pings.begin(); i != pings.end(); i++) {


            while (attitudeIndex + 1 < attitudes.size() && attitudes[attitudeIndex + 1].getTimestamp() < (*i).getTimestamp()) {
                attitudeIndex++;
            }

            //No more attitudes available
            if (attitudeIndex >= attitudes.size() - 1) {
                //std::cerr << "No more attitudes" << std::endl;
                break;
            }

            while (positionIndex + 1 < positions.size() && positions[positionIndex + 1].getTimestamp() < (*i).getTimestamp()) {
                positionIndex++;
            }
            //No more positions available
            if (positionIndex >= positions.size() - 1) {
                //std::cerr << "No more positions" << std::endl;
                break;
            }

            //No position or attitude smaller than ping, so discard this ping
            if (positions[positionIndex].getTimestamp() > (*i).getTimestamp() || attitudes[attitudeIndex].getTimestamp() > (*i).getTimestamp()) {
                std::cerr << "rejecting ping " << (*i).getId() << " " << (*i).getTimestamp() << " " << positions[positionIndex].getTimestamp() << " " << attitudes[attitudeIndex].getTimestamp() << std::endl;
                continue;
            }

            Attitude & beforeAttitude = attitudes[attitudeIndex];
            Attitude & afterAttitude = attitudes[attitudeIndex + 1];

            Position & beforePosition = positions[positionIndex];
            Position & afterPosition = positions[positionIndex + 1];

            Attitude * interpolatedAttitude = Interpolator::interpolateAttitude(beforeAttitude, afterAttitude, (*i).getTimestamp());
            Position * interpolatedPosition = Interpolator::interpolatePosition(beforePosition, afterPosition, (*i).getTimestamp());
            
            // Set the transducer depth to draft
            // If we have timestamped vertical motion, then this would need to
            // be processed and interpolated in the same way as Position and Attitude
            i->setTransducerDepth(transducerDraft); // i is the i-th ping

            //georeference
            Eigen::Vector3d georeferencedPing;
            georef.georeference(georeferencedPing, *interpolatedAttitude, *interpolatedPosition, (*i), *(svpStrategy.chooseSvp(*interpolatedPosition, *i)), leverArm, boresight);

            processGeoreferencedPing(georeferencedPing, (*i).getQuality(), (*i).getIntensity(), positionIndex, attitudeIndex);

            delete interpolatedAttitude;
            delete interpolatedPosition;
        }
    }

    virtual void processGeoreferencedPing(Eigen::Vector3d & georeferencedPing, uint32_t quality, int32_t intensity, int positionIndex, int attitudeIndex) {
        if(cart2geo) {
            Position p(0,0,0,0);
            cart2geo->ecefToLongitudeLatitudeElevation(georeferencedPing, p);
            std::cout << p.getLongitude() << " " << p.getLatitude() << " " << p.getEllipsoidalHeight() << " " << quality << " " << intensity << std::endl;
        } else {
            std::cout << georeferencedPing(0) << " " << georeferencedPing(1) << " " << georeferencedPing(2) << " " << quality << " " << intensity << std::endl;
        }
    }

    void setSvpStrategy(SvpSelectionStrategy& svpStrategy) {
        this->svpStrategy = svpStrategy;
    }
    
    void setCart2Geo(CartesianToGeodeticFukushima * c2g) {
        cart2geo = c2g;
    }
    
    void setTransducerDraft(double d) {
        transducerDraft = d;
    }


protected:

    /**the georeferencing method */
    Georeferencing & georef;
    
    /** the svp selection strategy*/
    SvpSelectionStrategy & svpStrategy;

    /**the current surface sound speed*/
    double currentSurfaceSoundSpeed;

    /**Vector of pings*/
    std::vector<Ping> pings;

    /**Vector of positions*/
    std::vector<Position> positions;

    /**vector of attitudes*/
    std::vector<Attitude> attitudes;

    /**Vector of SoundVelocityProfile*/
    std::vector<SoundVelocityProfile*> svps;
    
    /**the distance between transducer and water line*/
    double transducerDraft = 0.0;
    
    CartesianToGeodeticFukushima* cart2geo = NULL;
    
    std::string sbetFilePath = "";
};

#endif
