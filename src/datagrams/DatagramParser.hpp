#ifndef DATAGRAMPARSER_HPP
#define DATAGRAMPARSER_HPP

#include <cstdint>
#include "DatagramProcessor.hpp"

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
		DatagramParser(DatagramProcessor & processor);
                
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
		DatagramProcessor & processor;
};

/**
 * Create a datagram parser
 * 
 * @param processor the datagram processor
 */
DatagramParser::DatagramParser(DatagramProcessor & processor) : processor(processor){

}

#endif
