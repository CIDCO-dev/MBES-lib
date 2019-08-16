/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef DATAGRAMPARSER_CPP
#define DATAGRAMPARSER_CPP

#include "DatagramParser.hpp"

/**
* Creates a datagram parser
*
* @param processor the datagram processor
*/
DatagramParser::DatagramParser(DatagramEventHandler & processor) : processor(processor){

}

#endif

