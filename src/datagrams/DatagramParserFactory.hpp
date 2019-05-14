#ifndef DATAGRAMPARSERFACTORY_HPP
#define DATAGRAMPARSERFACTORY_HPP

#include "DatagramParser.hpp"
#include "kongsberg/KongsbergParser.hpp"
#include "xtf/XtfParser.hpp"
#include "s7k/S7kParser.hpp"
#include "../utils/StringUtils.hpp"
#include "../utils/Exception.hpp"

/*!
 * \brief Datagram parser factory class
 * \author ?
 *
 * Creates an appropriate parser
 */
class DatagramParserFactory{
	public:
                /**
                 * Creates the appropriate parser for the given file. Throws exception for unknown formats
                 * @param filename the name of the file
                 */
                static DatagramParser * build(std::string & fileName,DatagramEventHandler & handler){
                        DatagramParser * parser;

                        if(ends_with(fileName.c_str(),".all")){
                                parser = new KongsbergParser(handler);
                        }
                        else if(ends_with(fileName.c_str(),".xtf")){
                                parser = new XtfParser(handler);
                        }
                        else if(ends_with(fileName.c_str(),".s7k")){
                                parser = new S7kParser(handler);
                        }
                        else{
                                throw new Exception("Unknown extension");
                        }

                        return parser;
                };

};

#endif
