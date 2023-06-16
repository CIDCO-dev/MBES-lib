/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef DATAGRAMPARSERFACTORY_HPP
#define DATAGRAMPARSERFACTORY_HPP

#include "DatagramParser.hpp"
#include "kongsberg/KongsbergParser.hpp"
#include "xtf/XtfParser.hpp"
#include "s7k/S7kParser.hpp"
#include "../utils/StringUtils.hpp"
#include "../utils/Exception.hpp"
#include "DatagramEventHandler.hpp"
#include "../hydroblock/Hydroblock20Parser.hpp"
#include "kmall/KmallParser.hpp"

/*!
* \brief Datagram parser factory class
* \author Guillaume Labbe-Morissette
*
* Creates an appropriate parser
*/
class DatagramParserFactory{
public:
	/**
	* Creates the appropriate parser for the given file. Throws exception for unknown formats
	* @param filename the name of the file
	*/
	static DatagramParser * build(std::string & fileName,DatagramEventHandler & handler);
};

#endif
