#ifndef HYDROBLOCK20PARSER_HPP
#define HYDROBLOCK20PARSER_HPP

#include "../utils/TimeUtils.hpp"
#include "../datagrams/DatagramParser.hpp"
//#include "../svp/SvpSelectionStrategy.hpp"
//#include "../svp/SvpNearestByTime.hpp"
#include "../georeferencing/DatagramGeoreferencer.hpp"
#include <filesystem>

/*
Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*
 author Patrick Charron-Morneau
 hydroblock 2.0 parser
*/

class Hydroblock20Parser : public DatagramParser{
	public:
				/**
                 * Create an XTF parser
                 *
                 * @param processor the datagram processor
                 */
		Hydroblock20Parser(DatagramEventHandler & processor);

		~Hydroblock20Parser();

		void parse(std::string & dirPath, bool ignoreChecksum);

};
#endif

