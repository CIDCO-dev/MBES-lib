#ifndef DATAGRAMPARSER_HPP
#define DATAGRAMPARSER_HPP

#include <cstdint>
#include "DatagramProcessor.hpp"

class DatagramParser{
	public:
		DatagramParser(DatagramProcessor & processor);
		virtual ~DatagramParser(){};

        	virtual void parse(std::string & filename){};

	protected:

		DatagramProcessor & processor;
};

DatagramParser::DatagramParser(DatagramProcessor & processor) : processor(processor){

}

#endif
