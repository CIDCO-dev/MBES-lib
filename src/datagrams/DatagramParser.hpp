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
