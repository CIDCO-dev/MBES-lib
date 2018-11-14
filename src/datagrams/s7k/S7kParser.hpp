/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   S7kParser.hpp
 * Author: jordan
 *
 * Created on November 1, 2018, 4:30 PM
 */



#ifndef S7KPARSER_HPP
#define S7KPARSER_HPP

#include <fstream>
#include "../DatagramParser.hpp"
#include "S7kTypes.hpp"

class S7kParser : public DatagramParser {
public:
    S7kParser(DatagramProcessor & processor);
    ~S7kParser();

    void parse(std::string & filename);
    void packetHistogram(std::string & filename);
    void test(std::string & filename);

private:

    void readS7kFileHeader(std::ifstream & input, uint32_t OptionalDataIdentifier);
    bool hasValidChecksum(std::ifstream & input, S7kDataRecordFrame & drf, uint8_t* bytes, uint32_t numBytes);


};

S7kParser::S7kParser(DatagramProcessor & processor) : DatagramParser(processor) {

}

S7kParser::~S7kParser() {

}

void S7kParser::parse(std::string & filename) {
    
    
    std::ifstream input;
    input.open(filename, std::ios::binary);

    if (!input) {
        std::cerr << "Couldn't read " << filename << std::endl;
    } else {

        S7kDataRecordFrame drf;

        while (input.peek() != EOF) {
            std::ifstream::streampos drfStart = input.tellg();
            input.read((char*) &drf, sizeof (drf));
            std::ifstream::streampos drfEnd = input.tellg();

            uint32_t numBytes = drf.Size - 4;
            uint8_t entireDataRecordFrameExceptChecksum[numBytes];
            input.seekg(drfStart);
            input.read((char*) &entireDataRecordFrameExceptChecksum, numBytes);
            
            

            if (hasValidChecksum(input, drf, entireDataRecordFrameExceptChecksum, numBytes)) {
                std::cout << "Valid checksum" << std::endl;
                std::ifstream::streampos nextDRF = input.tellg();
                
                if (drf.RecordTypeIdentifier == 7200) {
                    input.seekg(drfEnd);
                    readS7kFileHeader(input, drf.OptionalDataIdentifier);
                    
                } else {
                    // this packet type isn't implemented

                }
                
                input.seekg(nextDRF);
            } else {
                continue;
            }
        }
    }

    input.close();
    
    
}

void S7kParser::readS7kFileHeader(std::ifstream & input, uint32_t OptionalDataIdentifier) {
    /* RecordTypeIdentifier == 7200 */
    S7kFileHeader fileHeader;
    input.read((char*) &fileHeader, sizeof (fileHeader));

    uint32_t N = fileHeader.NumberOfDevices;
    S7kFileHeaderRecordDatum data[N];
    input.read((char*) &data, N * sizeof (data));
    for (uint32_t i = 0; i < N; i++) {
        std::cout << data[i] << std::endl;
    }

    if (OptionalDataIdentifier == 7300) {
        S7kFileHeaderOptionalData catalogInformation;
        input.read((char*) &catalogInformation, sizeof (catalogInformation));
    }
};

bool S7kParser::hasValidChecksum(std::ifstream & input, S7kDataRecordFrame & drf, uint8_t* bytes, uint32_t numBytes) {
    if (drf.Flags == 1 || drf.Flags == 32769) {
        uint32_t checksum;
        input.read((char*) &checksum, sizeof (checksum));

        uint32_t calculatedChecksum = 0;

        for (uint32_t i = 0; i < numBytes; i++) {
            calculatedChecksum += (unsigned char) bytes[i];
        }

        if (calculatedChecksum == checksum) {
            return true;
        } else {
            return false;
        }
    }

    return true;
};

void S7kParser::packetHistogram(std::string & filename) {
    int numberOfRecordTypes = 88000; // see page 20-21 of s7k data format
    
    int usedRecordIdentifiers[numberOfRecordTypes];
    for(int i=0; i<numberOfRecordTypes; i++) {
        usedRecordIdentifiers[i]=0;
    }
    
    std::ifstream input;
    input.open(filename, std::ios::binary);

    if (!input) {
        std::cerr << "Couldn't read " << filename << std::endl;
    } else {

        S7kDataRecordFrame dataRecordFrame;

        while (input.peek() != EOF) {
            // Read dataRecordFrame and save initial and final stream positions
            std::ifstream::streampos dataRecordFrameStart = input.tellg();
            input.read((char*) &dataRecordFrame, sizeof (dataRecordFrame));
            
            uint32_t numBytes = dataRecordFrame.Size;
            uint8_t entireDataRecordFrame[numBytes];
            input.seekg(dataRecordFrameStart);
            input.read((char*) &entireDataRecordFrame, numBytes);
            
            usedRecordIdentifiers[dataRecordFrame.RecordTypeIdentifier]++;
        }
    }

    input.close();
    
    
    for(int i=0; i<numberOfRecordTypes; i++) {
        if(usedRecordIdentifiers[i]!=0) {
            std::cout << i << ": " << usedRecordIdentifiers[i] << std::endl;
        }
    }
};

void S7kParser::test(std::string & filename) {
    std::ifstream input;
    input.open(filename, std::ios::binary);

    if (!input) {
        std::cerr << "Couldn't read " << filename << std::endl;
    } else {

        S7kDataRecordFrame dataRecordFrame;

        while (input.peek() != EOF) {
            // Read dataRecordFrame and save initial and final stream positions
            std::ifstream::streampos dataRecordFrameStart = input.tellg();
            input.read((char*) &dataRecordFrame, sizeof (dataRecordFrame));
            std::ifstream::streampos dataRecordFrameEnd = input.tellg();
            
            uint32_t numBytes = dataRecordFrame.Size;
            uint8_t entireDataRecordFrame[numBytes];
            input.seekg(dataRecordFrameStart);
            input.read((char*) &entireDataRecordFrame, numBytes);
            std::ifstream::streampos nextDataRecordFrame = input.tellg();
            
            if(dataRecordFrame.RecordTypeIdentifier == 1016) {
                
                std::cout << dataRecordFrame << std::endl;
                input.seekg(dataRecordFrameEnd);
                
                S7kAttitudeRTH attitudeHeader;
                input.read((char*) &attitudeHeader, sizeof (attitudeHeader));
                
                std::cout << "attitudeHeader.NumberOfAttitudeDataSets: " << attitudeHeader.NumberOfAttitudeDataSets << std::endl;
                
                input.seekg(nextDataRecordFrame);
            }
        }
    }

    input.close();
};

#endif /* S7KPARSER_HPP */

