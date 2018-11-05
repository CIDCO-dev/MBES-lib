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

private:

    void readS7kFileHeader(std::ifstream & input, uint32_t OptionalDataIdentifier);
    bool hasValidChecksum(std::ifstream & input, S7kDataRecordFrame & drf, uint8_t* bytes, uint32_t numBytes);


};

S7kParser::S7kParser(DatagramProcessor & processor) : DatagramParser(processor) {

}

S7kParser::~S7kParser() {

}

void S7kParser::parse(std::string & filename) {
    int usedRecordIdentifiers[8000];
    for(int i=0; i<8000; i++) {
        usedRecordIdentifiers[i]=0;
    }
    
    std::ifstream input;
    input.open(filename, std::ios::binary);

    if (!input) {
        std::cerr << "Couldn't read " << filename << std::endl;
    } else {

        S7kDataRecordFrame drf;

        while (input) {
            std::ifstream::streampos drfStart = input.tellg();
            std::cout << "drfStart: " << drfStart << std::endl;
            input.read((char*) &drf, sizeof (drf));
            std::ifstream::streampos drfEnd = input.tellg();
            std::cout << "drfEnd: " << drfEnd << std::endl;

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
                    usedRecordIdentifiers[drf.RecordTypeIdentifier]++;

                }
                
                input.seekg(nextDRF);
            } else {
                continue;
            }
        }
    }

    input.close();
    
    for(int i=0; i<8000; i++) {
        if(usedRecordIdentifiers[i]!=0) {
            
            std::cout << i << ": " << usedRecordIdentifiers[i] << std::endl;
        }
    }
}

void S7kParser::readS7kFileHeader(std::ifstream & input, uint32_t OptionalDataIdentifier) {
    S7kFileHeader fileHeader;
    input.read((char*) &fileHeader, sizeof (fileHeader));
    std::cout << fileHeader << std::endl;

    uint32_t N = fileHeader.NumberOfDevices;
    S7kFileHeaderRecordDatum data[N];
    input.read((char*) &data, N * sizeof (data));
    for (uint32_t i = 0; i < N; i++) {
        std::cout << data[i] << std::endl;
    }

    if (OptionalDataIdentifier == 7300) {
        S7kFileHeaderOptionalData catalogInformation;
        input.read((char*) &catalogInformation, sizeof (catalogInformation));
        std::cout << catalogInformation << std::endl;
    }
};

bool S7kParser::hasValidChecksum(std::ifstream & input, S7kDataRecordFrame & drf, uint8_t* bytes, uint32_t numBytes) {
    if (drf.Flags == 1 || drf.Flags == 32769) {
        uint32_t checksum;
        input.read((char*) &checksum, sizeof (checksum));
        std::cout << "checksum:" << checksum << std::endl;

        uint32_t calculatedChecksum = 0;

        for (uint32_t i = 0; i < numBytes; i++) {
            calculatedChecksum += (unsigned char) bytes[i];
        }
        
        std::cout << "calculatedChecksum:" << calculatedChecksum << std::endl;

        if (calculatedChecksum == checksum) {
            return true;
        } else {
            return false;
        }
    }

    return true;
};

#endif /* S7KPARSER_HPP */

