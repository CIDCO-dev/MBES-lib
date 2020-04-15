/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef S7KPARSER_CPP
#define S7KPARSER_CPP

#include "S7kParser.hpp"
#include "../../utils/Exception.hpp"

S7kParser::S7kParser(DatagramEventHandler & processor) : DatagramParser(processor) {

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
                            else if(drf.RecordTypeIdentifier == 1012) {
                                //roll pitch heave
                                //processRollPitchHeaveDatagram(drf, data);
                            }
                            else if(drf.RecordTypeIdentifier == 1013) {
                                //heading
                                //processHeadingDatagram(drf, data);
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
                            std::cout << "checksum: " << checksum << std::endl;
                            std::cout << "computedChecksum: " << computedChecksum << std::endl;
                            std::cout << "drf.RecordTypeIdentifier: " << drf.RecordTypeIdentifier << std::endl;
                            printf("Checksum error\n");
                            //Checksum error...lets ignore the packet for now
                            //throw new Exception("Checksum error");
                            continue;
                        }
                    }

                    free(data);
                } else {
                    throw new Exception("Couldn't find sync pattern");
                }
            }//Negative items mean something went wrong
            else if (nbItemsRead < 0) {
                throw new Exception("Read error");
            }

            //zero bytes means EOF. Nothing to do
        }
    } else {
        throw new Exception("File not found");
    }
}

std::string S7kParser::getName(int tag)
{
    switch(tag)
    {
        case 1000:
            return "Reference Point";
        break;

        case 1001:
            return "Sensor Offset Position";
        break;

        case 1002:
            return "Sensor Offset Position Calibrated";
        break;

        case 1003:
            return "Position";
        break;

        case 1004:
            return "Custom Attitude Information";
        break;

        case 1005:
            return "Tide";
        break;

        case 1006:
            return "Altitude";
        break;

        case 1007:
            return "Motion Over Ground";
        break;

        case 1008:
            return "Depth";
        break;

        case 1009:
            return "Sound Velocity Profile";
        break;

        case 1010:
            return "CTD";
        break;

        case 1011:
            return "Geodesy";
        break;

        case 1012:
            return "Roll Pitch Heave";
        break;

        case 1013:
            return "Heading";
        break;

        case 1014:
            return "Survey Line";
        break;

        case 1015:
            return "Navigation";
        break;

        case 1016:
            return "Attitude";
        break;

        case 1017:
            return "Pan Tilt";
        break;

        case 1020:
            return "Sonar Installation Identifiers";
        break;

        case 2004:
            return "Sonar Pipe Environment";
        break;

        case 7000:
            return "7k Sonar Settings";
        break;

        case 7001:
            return "7k Configuration";
        break;

        case 7002:
            return "7k Match Filter";
        break;

        case 7003:
            return "7k Firmware and Hardware Configuration";
        break;

        case 7004:
            return "7k Beam Geometry";
        break;

        case 7006:
            return "7k Bathymetric Data";
        break;

        case 7007:
            return "7k Side Scan Data";
        break;

        case 7008:
            return "7k Generic Water Column Data";
        break;

        case 7010:
            return "TVG Values";
        break;

        case 7011:
            return "7k Image Data";
        break;

        case 7012:
            return "7k Ping Motion Data";
        break;

        case 7017:
            return "7k Detection Data Setup";
        break;

        case 7018:
            return "7k Beamformed Data";
        break;

        case 7019:
            return "Vernier Processing Data";
        break;

        case 7021:
            return "7k Built-In Test Environment Data";
        break;

        case 7022:
            return "7kCenter Version";
        break;

        case 7023:
            return "8k Wet End Version";
        break;

        case 7027:
            return "7k RAW Detection Data";
        break;

        case 7028:
            return "7k Snippet Data";
        break;

        case 7030:
            return "Sonar Installation Parameters";
        break;

        case 7031:
            return "7k Built-In Test Environment Data (Summary)";
        break;

        case 7041:
            return "Compressed Beamformed Magnitude Data";
        break;

        case 7042:
            return "Compressed Watercolumn Data";
        break;

        case 7048:
            return "7k Calibrated Beam Data";
        break;

        case 7050:
            return "7k System Events";
        break;

        case 7051:
            return "7k System Event Message";
        break;

        case 7052:
            return "RDR Recording Status";
        break;

        case 7053:
            return "7k Subscriptions";
        break;

        case 7055:
            return "Calibration Status";
        break;

        case 7057:
            return "Calibrated Side-Scan Data";
        break;

        case 7058:
            return "Calibrated Snippet Data";
        break;

        case 7059:
            return "MB2 specific status";
        break;

        case 7200:
            return "7k File Header";
        break;

        case 7300:
            return "7k File Catalog Record";
        break;

        case 7400:
            return "7k Time Message";
        break;

        case 7500:
            return "7k Remote Control";
        break;

        case 7501:
            return "7k Remote Control Acknowledge";
        break;

        case 7502:
            return "7k Remote Control Not Acknowledge";
        break;

        case 7503:
            return "Remote Control Sonar Settings";
        break;

        case 7504:
            return "7P Common System Settings";
        break;

        case 7510:
            return "SV Filtering";
        break;

        case 7511:
            return "System Lock Status";
        break;

        case 7610:
            return "7k Sound Velocity";
        break;

        case 7611:
            return "7k Absorption Loss";
        break;

        case 7612:
            return "7k Spreading Loss";
        break;

        default:
            return "Invalid tag";
	break;
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
    if(position->DatumIdentifier == 0 && position->PositionTypeFlag == 0) {
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
		double intensity = swath->dataFieldSize > 22 ? ping->signalStrength : 0; 
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

    uint64_t res = TimeUtils::build_time(drf.Timestamp.Year, drf.Timestamp.Day, drf.Timestamp.Hours, drf.Timestamp.Minutes, microSeconds);

    return res;
}

void S7kParser::processCtdDatagram(S7kDataRecordFrame & drf,unsigned char * data){
        S7kCtdRTH * ctd = (S7kCtdRTH*) data;

	SoundVelocityProfile * svp = new SoundVelocityProfile();

	uint64_t timestamp = extractMicroEpoch(drf);

	svp->setTimestamp(timestamp);

        //TODO: get nearest position
        svp->setLatitude(0);
        svp->setLongitude(0);

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


#endif