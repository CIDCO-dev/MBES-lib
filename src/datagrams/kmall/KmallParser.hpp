/*
* Copyright 2023 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef KMALL_HPP
#define KMALL_HPP


//#include <string>
//#include <cstdio>
#include <iostream>
//#include <cmath>
//#include <map>

#include "../DatagramParser.hpp"
//#include "../../utils/NmeaUtils.hpp"
//#include "../../utils/TimeUtils.hpp"
//#include "../../utils/Exception.hpp"
#include "KmallTypes.hpp"

/*!
* \brief kmall parser class extention of Datagram parser class
* \author Patrick Charron-Morneau
*/
class KmallParser : public DatagramParser{
public:
/**
  * Creates a Kongsberg parser
  *
  * @param processor the datagram processor
  */
  KmallParser(DatagramEventHandler & processor);

  /**Destroys the Kongsberg parser*/
  ~KmallParser();

  //interface methods
  /**
  * Read the file and loop through it
  *
  * @param filename name of the file to read
  */
  void parse(std::string & filename, bool ignoreChecksum = false);
    
protected:

  void processDatagram(EMdgmHeader & header, unsigned char * datagram);
	
  void processSVP(EMdgmHeader & header, unsigned char * datagram);
  
  void processSPO(EMdgmHeader & header, unsigned char * datagram);
  
  void processSKM(EMdgmHeader & header, unsigned char * datagram);
  
  void processSCL(EMdgmHeader & header, unsigned char * datagram);
  
  void processSVT(EMdgmHeader & header, unsigned char * datagram);
  
  void processSDE(EMdgmHeader & header, unsigned char * datagram);
  
  void processSHI(EMdgmHeader & header, unsigned char * datagram);
  
  void processMRZ(EMdgmHeader & header, unsigned char * datagram);
  
  
};

#endif
