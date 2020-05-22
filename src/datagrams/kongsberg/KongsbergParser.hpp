/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef KONGSBERG_HPP
#define KONGSBERG_HPP


#include <string>
#include <cstdio>
#include <iostream>
#include <cmath>
#include <map>

#include "../DatagramParser.hpp"
#include "../../utils/NmeaUtils.hpp"
#include "../../utils/TimeUtils.hpp"
#include "../../utils/Exception.hpp"
#include "KongsbergTypes.hpp"

/*!
* \brief Kongsberg parser class extention of Datagram parser class
* \author Guillaume Labbe-Morissette
*/
class KongsbergParser : public DatagramParser{
public:

  /**
  * Creates a Kongsberg parser
  *
  * @param processor the datagram processor
  */
  KongsbergParser(DatagramEventHandler & processor);

  /**Destroys the Kongsberg parser*/
  ~KongsbergParser();

  //interface methods
  /**
  * Read the file and loop through it
  *
  * @param filename name of the file to read
  */
  void parse(std::string & filename, bool ignoreChecksum = false);

  std::string getName(int tag);

protected:

  /**
  * Processes the datagram depending on the type of the Kongsberg Header
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processDatagram(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Depth
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processDepth(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Water Height
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processWaterHeight(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Attitude
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processAttitudeDatagram(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Position
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processPositionDatagram(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Quality Factor
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processQualityFactor(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Seabed Image Data
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processSeabedImageData(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Processes the Sound Speed Profile
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processSoundSpeedProfile(KongsbergHeader & hdr,unsigned char * datagram);


  /**
  * Processes range and beam data
  *
  * @param hdr the Kongsberg header
  * @param datagram the datagram
  */
  void processRawRangeAndBeam78(KongsbergHeader & hdr,unsigned char * datagram);

  /**
  * Returns the timestamp in microsecond
  *
  * @param datagramDate the datagram date
  * @param datagramTime the datagram time
  */
  uint64_t convertTime(uint32_t datagramDate,uint32_t datagramTime);

  /**
  * Returns a human readable name for a given datagram tag
  */
  /*std::string getName(int tag);*/
};

#endif
