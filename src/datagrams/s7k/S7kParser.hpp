/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/*
 * File:   S7kParser.hpp
 * Author: glm,jordan
 *
 * Created on November 1, 2018, 4:30 PM
 */


#ifndef S7KPARSER_HPP
#define S7KPARSER_HPP

#include <cstdio>
#include "../DatagramParser.hpp"
#include "S7kTypes.hpp"
#include "../../utils/TimeUtils.hpp"
#include "../../utils/Constants.hpp"
#include <list>
#include "../../SoundVelocityProfile.hpp"

class S7kParser : public DatagramParser {
public:
    S7kParser(DatagramProcessor & processor);
    ~S7kParser();

    void parse(std::string & filename);

protected:
    void processDataRecordFrame(S7kDataRecordFrame & drf);
    void processAttitudeDatagram(S7kDataRecordFrame & drf, unsigned char * data);
    void processPositionDatagram(S7kDataRecordFrame & drf, unsigned char * data);
    void processPingDatagram(S7kDataRecordFrame & drf, unsigned char * data);
    void processSonarSettingsDatagram(S7kDataRecordFrame & drf, unsigned char * data);
    void processCtdDatagram(S7kDataRecordFrame & drf,unsigned char * data);

private:
    uint32_t computeChecksum(S7kDataRecordFrame * drf, unsigned char * data);
    uint64_t extractMicroEpoch(S7kDataRecordFrame & drf);

    //TODO Use a map instead
    std::list<S7kSonarSettings *> pingSettings;
};

S7kParser::S7kParser(DatagramProcessor & processor) : DatagramParser(processor) {

}

S7kParser::~S7kParser() {

}

void S7kParser::parse(std::string & filename) {
    FILE * file = fopen(filename.c_str(), "rb");

    if (file) {
        S7kDataRecordFrame drf;

        while (!feof(file)) {

            //Read the DRF
            int nbItemsRead = fread(&drf, sizeof (S7kDataRecordFrame), 1, file);

            //Check that we read the required amount of data
            if (nbItemsRead == 1) {

                //Sanity check on the DRF
                if (drf.SyncPattern == SYNC_PATTERN) {
                    processDataRecordFrame(drf);

                    int dataSectionSize = drf.Size - sizeof (S7kDataRecordFrame); // includes checksum
                    unsigned char * data = (unsigned char*) malloc(dataSectionSize);

                    //Now read in the data section and the checksum
                    nbItemsRead = fread(data, dataSectionSize, 1, file);

                    //We can haz data
                    if (nbItemsRead == 1) {

                        //Verify it
                        uint32_t checksum = *((uint32_t*) & data[dataSectionSize - sizeof (uint32_t)]);
                        uint32_t computedChecksum = computeChecksum(&drf, data);

                        if (checksum == computedChecksum) {
                            processor.processDatagramTag(drf.RecordTypeIdentifier);

			    //Process data according to record type
                            if (drf.RecordTypeIdentifier == 1016) {
                                //Attitude
                                processAttitudeDatagram(drf, data);
                            }
			    else if (drf.RecordTypeIdentifier == 1003) {
                                //Position
                                processPositionDatagram(drf, data);
                            }
			    else if(drf.RecordTypeIdentifier == 7027) {
                                //Ping
				processPingDatagram(drf, data);
                            }
			    else if(drf.RecordTypeIdentifier == 7000){
				//Sonar settings
				processSonarSettingsDatagram(drf,data);
			    }
			    else if(drf.RecordTypeIdentifier == 1010){
				//CTD
                                processCtdDatagram(drf,data);
                            }
                            //TODO: process other stuff

                        } else {
                            printf("Checksum error\n");
                            //Checksum error...lets ignore the packet for now
                            //throw "Checksum error";
                            continue;
                        }
                    }

                    free(data);
                } else {
                    throw "Couldn't find sync pattern";
                }
            }//Negative items mean something went wrong
            else if (nbItemsRead < 0) {
                throw "Read error";
            }

            //zero bytes means EOF. Nothing to do
        }
    } else {
        throw "File not found";
    }
}

void S7kParser::processDataRecordFrame(S7kDataRecordFrame & drf) {
    //TODO: remove later, leave derived classes decide what to do
/*    printf("--------------------\n");
    printf("%d-%03d %02d:%02d:%f\n", drf.Timestamp.Year, drf.Timestamp.Day, drf.Timestamp.Hours, drf.Timestamp.Minutes, drf.Timestamp.Seconds);
    printf("Type: %d\n", drf.RecordTypeIdentifier);
    printf("Bytes: %d\n", drf.Size);
    printf("--------------------\n");
*/
}

uint32_t S7kParser::computeChecksum(S7kDataRecordFrame * drf, unsigned char * data) {
    uint32_t checksum = 0;

    unsigned int dataSize = drf->Size - sizeof (S7kDataRecordFrame) - sizeof (uint32_t); //exclude checksum

    for (unsigned int i = 0; i< sizeof (S7kDataRecordFrame); i++) {
        checksum += (unsigned char) ((unsigned char*) drf)[i];
    }

    for (unsigned int i = 0; i < dataSize; i++) {
        checksum += (unsigned char) data[i];
    }

    return checksum;
}

void S7kParser::processAttitudeDatagram(S7kDataRecordFrame & drf, unsigned char * data) {
    uint64_t microEpoch  = extractMicroEpoch(drf);
    uint8_t  nEntries    = ((uint8_t*)data)[0];
    S7kAttitudeRD *entry = (S7kAttitudeRD*)(data+1);

    for(unsigned int i = 0;i<nEntries;i++){
	double heading = (double)entry[i].heading*R2D;
	double pitch   = (double)entry[i].pitch*R2D;
	double roll    = (double)entry[i].roll*R2D;

        processor.processAttitude(
		microEpoch + entry[i].timeDifferenceFromRecordTimeStamp * 1000,
                heading,
                (pitch<0)?pitch+360:pitch,
                (roll<0)?roll+360:roll
	);
    }
}

void S7kParser::processSonarSettingsDatagram(S7kDataRecordFrame & drf, unsigned char * data){
    S7kSonarSettings * settings = (S7kSonarSettings*)data;

    S7kSonarSettings * settingsCopy = (S7kSonarSettings *)malloc(sizeof(S7kSonarSettings));
    memcpy(settingsCopy,settings,sizeof(S7kSonarSettings));

    pingSettings.push_back(settingsCopy);
}

void S7kParser::processPositionDatagram(S7kDataRecordFrame & drf, unsigned char * data) {
    uint64_t microEpoch = extractMicroEpoch(drf);
    S7kPosition *position = (S7kPosition*) data;

    // only process WGS84, ignore grid coordinates
    if(position->DatumIdentifier == 0 && position->PositioningMethod == 0) {
        processor.processPosition(microEpoch, (double)position->LongitudeOrEasting * R2D, (double)position->LatitudeOrNorthing * R2D, (double)position->Height);
    }
}

void S7kParser::processPingDatagram(S7kDataRecordFrame & drf, unsigned char * data) {
    uint64_t microEpoch = extractMicroEpoch(drf);

    S7kRawDetectionDataRTH *swath = (S7kRawDetectionDataRTH*) data;

    uint32_t nEntries = swath->numberOfDetectionPoints;

    double tiltAngle = swath->transmissionAngle*R2D;
    double samplingRate = swath->samplingRate;

    S7kSonarSettings * settings = NULL;

    for(auto i=pingSettings.begin();i!=pingSettings.end();i++){
	if((*i)->sequentialNumber==swath->pingNumber){
		settings = (*i);
                pingSettings.remove((*i));
		break;
	}
    }

    if(settings){
	double surfaceSoundVelocity = settings->soundVelocity;

	processor.processSwathStart(surfaceSoundVelocity);

	for(unsigned int i = 0;i<nEntries;i++) {
		S7kRawDetectionDataRD *ping = (S7kRawDetectionDataRD*)(data+sizeof(S7kRawDetectionDataRTH) + i*swath->dataFieldSize);
		double twoWayTravelTime = (double)ping->detectionPoint / samplingRate; // see Appendix F p. 190
		double intensity = swath->dataFieldSize > 22 ? ping->signalStrength : 0; //see p. 79-80
		processor.processPing(microEpoch,(long)ping->beamDescriptor,(double)ping->receptionAngle*R2D,tiltAngle,twoWayTravelTime,ping->quality,intensity);
        }

        free(settings);
    }
    else{
	fprintf(stderr,"No settings for ping #%d\n",swath->pingNumber);
    }
}

uint64_t S7kParser::extractMicroEpoch(S7kDataRecordFrame & drf) {
    long microSeconds = drf.Timestamp.Seconds * 1e6;

    uint64_t res = build_time(drf.Timestamp.Year, drf.Timestamp.Day, drf.Timestamp.Hours, drf.Timestamp.Minutes, microSeconds);

    return res;
}

void S7kParser::processCtdDatagram(S7kDataRecordFrame & drf,unsigned char * data){
        S7kCtdRTH * ctd = (S7kCtdRTH*) data;

	SoundVelocityProfile * svp = new SoundVelocityProfile();

	uint64_t timestamp = extractMicroEpoch(drf);

	svp->setTimestamp(timestamp);

	if(
		ctd->sampleContentValidity & 0x0C //depth & sound velocity OK
		&&
		ctd->pressureFlag == 1 //depth
          ){
		//Get position if available
		if(ctd->positionFlag){
			svp->setLongitude(ctd->longitude);
			svp->setLatitude(ctd->latitude);
		}

		//Get SVP samples
		S7kCtdRD * rd = (S7kCtdRD *) (((unsigned char *)ctd) + sizeof(S7kCtdRTH));
		for(unsigned int i = 0;i < ctd->nbSamples;i++  ){
			svp->add(rd[i].pressureDepth,rd[i].soundVelocity);
		}

		processor.processSoundVelocityProfile(svp);
	}
}

#endif /* S7KPARSER_HPP */
