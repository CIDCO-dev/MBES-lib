/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef DATAGRAMPARSERFACTORY_CPP
#define DATAGRAMPARSERFACTORY_CPP

#include "DatagramParserFactory.hpp"

/**
* Creates the appropriate parser for the given file. Throws exception for unknown formats
* @param filename the name of the file
*/
DatagramParser * DatagramParserFactory::build(std::string & fileName,DatagramEventHandler & handler){
        DatagramParser * parser;

        if(StringUtils::ends_with(fileName.c_str(),".all")){
                parser = new KongsbergParser(handler);
        }
        else if(StringUtils::ends_with(fileName.c_str(),".xtf")){
                parser = new XtfParser(handler);
        }
        else if(StringUtils::ends_with(fileName.c_str(),".s7k")){
                parser = new S7kParser(handler);
        }
        else{
                throw new Exception("Unknown extension");
        }

        return parser;
}

#endif