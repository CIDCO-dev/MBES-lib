#ifndef DATAGRAMPARSER_HPP
#define DATAGRAMPARSER_HPP

#include <cstdint>
#include "DatagramEventHandler.hpp"

/*!
 * \brief Datagram parser class
 */
class DatagramParser{
	public:
                /**
                 * Create a datagram parser
                 * 
                 * @param processor the datagram processor
                 */
		DatagramParser(DatagramEventHandler & processor);
                
                /**
                 * Destroy the datagram parser
                 */
		virtual ~DatagramParser(){};

                /**
                 * Read a file and change the datagram parser depending on the information
                 * 
                 * @param filename name of the file to read
                 */
        	virtual void parse(std::string & filename){};

		/**
		 * Creates the appropriate parser for the given file. Throws exception for unknown formats
		 * @param filename the name of the file
		 */
		static DatagramParser * build(std::string & filename){
			DatagramParser * parser;

			if(ends_with(fileName.c_str(),".all")){
                		parser = new KongsbergParser(cloud);
        		}
        		else if(ends_with(fileName.c_str(),".xtf")){
                		parser = new XtfParser(cloud);
        		}
        		else if(ends_with(fileName.c_str(),".s7k")){
                		parser = new S7kParser(cloud);
        		}
        		else{
                		throw new Exception("Unknown extension");
        		}
			return parser;
		}

	protected:

            /**The datagram processor*/
		DatagramEventHandler & processor;
};

/**
 * Create a datagram parser
 * 
 * @param processor the datagram processor
 */
DatagramParser::DatagramParser(DatagramEventHandler & processor) : processor(processor){

}

#endif
