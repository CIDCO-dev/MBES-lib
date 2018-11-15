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

class S7kParser : public DatagramParser {
public:
    S7kParser(DatagramProcessor & processor);
    ~S7kParser();

    void parse(std::string & filename);

protected:
    void processDataRecordFrame(S7kDataRecordFrame & drf);
    void processAttitudeDatagram(S7kDataRecordFrame & drf, unsigned char * data);
    void processPositionDatagram(S7kDataRecordFrame & drf, unsigned char * data);

private:
    uint32_t computeChecksum(S7kDataRecordFrame * drf, unsigned char * data);
    uint64_t extractMicroEpoch(S7kDataRecordFrame & drf);
};

S7kParser::S7kParser(DatagramProcessor & processor) : DatagramParser(processor) {

}

S7kParser::~S7kParser() {

}

/*
        The wise programmer is told about Tao and follows it.
        The average programmer is told about Tao and searches for it.
        The foolish programmer is told about Tao and laughs at it.
        If it were not for laughter, there would be no Tao.
 */

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

                            //Process data according to record type
                            if (drf.RecordTypeIdentifier == 1016) {
                                //Attitude
                                processAttitudeDatagram(drf, data);
                            } else if (drf.RecordTypeIdentifier == 1003) {
                                //Position
                                processPositionDatagram(drf, data);
                            }
                            //TODO: process pings and other stuff
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
            }                //Negative items mean something went wrong
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
    printf("--------------------\n");
    printf("%d-%03d %02d:%02d:%f\n", drf.Timestamp.Year, drf.Timestamp.Day, drf.Timestamp.Hours, drf.Timestamp.Minutes, drf.Timestamp.Seconds);
    std::cout << extractMicroEpoch(drf) << std::endl;
    std::cout << "drf.Timestamp.Seconds: " << drf.Timestamp.Seconds << std::endl;
    printf("Type: %d\n", drf.RecordTypeIdentifier);
    printf("Bytes: %d\n", drf.Size);
    printf("--------------------\n");
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
    uint64_t timestamp = extractMicroEpoch(drf);

    //TODO: normalize data to fit the format of DataProcessor, then call DataProcessor's processAttitude()
}

void S7kParser::processPositionDatagram(S7kDataRecordFrame & drf, unsigned char * data) {
    uint64_t timestamp = extractMicroEpoch(drf);

    //TODO: normalize data to fit the format of DataProcessor, then call DataProcessor's processPosition()
}

uint64_t S7kParser::extractMicroEpoch(S7kDataRecordFrame & drf) {
    long microSeconds = drf.Timestamp.Seconds*1e6;

    //yday is 1-366 in s7k's, shift by 1 to 0-365
    uint64_t res = build_time(drf.Timestamp.Year, drf.Timestamp.Day, drf.Timestamp.Hours, drf.Timestamp.Minutes, microSeconds);

    return res;
}


#endif /* S7KPARSER_HPP */

