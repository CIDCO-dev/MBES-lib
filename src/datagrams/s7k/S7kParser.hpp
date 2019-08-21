/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef S7KPARSER_HPP
#define S7KPARSER_HPP

#include <cstdio>
#include "../DatagramParser.hpp"
#include "S7kTypes.hpp"
#include "../../utils/TimeUtils.hpp"
#include "../../utils/Constants.hpp"
#include <list>
#include "../../svp/SoundVelocityProfile.hpp"

/*!
 * \brief S7k parser class extention of Datagram parser
 * \author Guillaume Labbe-Morissette, Jordan McManus
 * \date November 1, 2018, 4:30 PM
 */
class S7kParser : public DatagramParser {
public:

    /**
     * Creates an S7k parser
     *
     * @param processor the datagram processor
     */
    S7kParser(DatagramEventHandler & processor);

    /**Destroys the S7k parser*/
    ~S7kParser();

    /**
     * Read the file and loop through it
     *
     * @param filename name of the file to read
     */
    void parse(std::string & filename);

    std::string getName(int tag);

protected:

    /**
     * Sets the S7k data record frame
     *
     * @param drf the new S7k data record frame
     */
    void processDataRecordFrame(S7kDataRecordFrame & drf);

    /**
     * Processes the Attitude
     *
     * @param drf the S7k data record frame
     * @param data the datagram
     */
    void processAttitudeDatagram(S7kDataRecordFrame & drf, unsigned char * data);

    /**
     * Processes the Position
     *
     * @param drf the S7k data record frame
     * @param data the datagram
     */
    void processPositionDatagram(S7kDataRecordFrame & drf, unsigned char * data);

    /**
     * Processes the Ping
     *
     * @param drf the S7k data record frame
     * @param data the datagram
     */
    void processPingDatagram(S7kDataRecordFrame & drf, unsigned char * data);

    /**
     * Processes the Sonar setting
     *
     * @param drf the S7k data record frame
     * @param data datagram
     */
    void processSonarSettingsDatagram(S7kDataRecordFrame & drf, unsigned char * data);

    /**
     * Processes the Sound Velocity Profile base on the Ctd
     *
     * @param drf the S7k data record frame
     * @param data the datagram
     */
    void processCtdDatagram(S7kDataRecordFrame & drf,unsigned char * data);

    /**
     * Returns a human readable name for a given datagram tag
     */
     /*std::string getName(int tag);*/


private:

    /**
     * Returns the 'Check summary' of the S7k data record frame
     *
     * @param drf the S7k data record frame
     * @param data the datagram
     */
    uint32_t computeChecksum(S7kDataRecordFrame * drf, unsigned char * data);

    /**
     * Gets the S7k data record frame
     *
     * @param drf the S7k data record frame
     */
    uint64_t extractMicroEpoch(S7kDataRecordFrame & drf);

    //TODO Use a map instead
    /**List of ping settings*/
    std::list<S7kSonarSettings *> pingSettings;
};




#endif /* S7KPARSER_HPP */
