#ifndef DATAGRAMPROCESSOR_HPP
#define DATAGRAMPROCESSOR_HPP



/**
 * @author Guillaume Morissette
 * Fournit des methodes virtuelles pour le traitement des contenus des datagrammes de sonar
 */

class DatagramProcessor{
	public:
		DatagramProcessor(){};
		virtual ~DatagramProcessor(){};

                virtual void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){};
                virtual void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){};
                virtual void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){};
                virtual void processSwathStart(double surfaceSoundSpeed){};
};


#endif
