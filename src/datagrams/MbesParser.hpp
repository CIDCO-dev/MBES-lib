#ifndef MBESPARSER_HPP
#define MBESPARSER_HPP

#include <cstdint>

class MbesParser{
	public:
		MbesParser();
		virtual ~MbesParser(){};

        virtual void parse(std::string filename){};

		virtual void processAttitude(uint64_t microEpoch,double heading,double pitch,double roll){};
        virtual void processPosition(uint64_t microEpoch,double longitude,double latitude,double height){};
        virtual void processPing(uint64_t microEpoch,long id, double beamAngle,double tiltAngle,double twoWayTravelTime,uint32_t quality,uint32_t intensity){};
        virtual void processSwathStart(){};

};

MbesParser::MbesParser(){

}

#endif
