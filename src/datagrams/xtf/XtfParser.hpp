/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef XTFPARSER_HPP
#define XTFPARSER_HPP

//TODO: under windows, in winsock.h
#ifdef _WIN32
#include <winsock.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#endif

#include <string>
#include <stdio.h>
#include <string.h>
#include <cstdio>
#include "XtfTypes.hpp"
#include "../DatagramParser.hpp"
#include "../../utils/TimeUtils.hpp"
#include "../../utils/Exception.hpp"
#include <vector>
#include "../../Ping.hpp"
#include "../../math/SlantRangeCorrection.hpp"

#define MAGIC_NUMBER 123
#define PACKET_MAGIC_NUMBER 0xFACE

/**
 * @author Guillaume Morissette
 *
 * Warning: this code runs on little-endian machines. It has not been shielded for variations in endianness, especially when big-endian vendor-datagrams are handled.
 */


/*!
 * \brief XTF parser class extention datagram parser
 * \author Guillaume Morissette
 */
class XtfParser : public DatagramParser{
	public:

                /**
                 * Create an XTF parser
                 *
                 * @param processor the datagram processor
                 */
		XtfParser(DatagramEventHandler & processor);

                /**Destroy the XTF parser*/
		~XtfParser();

                /**
                 * Parse an XTF file
                 *
                 * @param filename name of the file to read
                 */
		void parse(std::string & filename);

                std::string getName(int tag);

                /**Return the number channels in the file*/
		int getTotalNumberOfChannels();

	protected:

                /**
                 * Process the contents of the XtfPacketHeader
                 *
                 * @param hdr the XTF PacketHeader
                 */
		void processPacketHeader(XtfPacketHeader & hdr);

                /**
                 * Dispatch processing to the appropriate callback depending on the content of the XTF Packet header
                 *
                 * @param hdr the XTF Packet Header
                 * @param packet the packet
                 */
		void processPacket(XtfPacketHeader & hdr,unsigned char * packet);

                /**
                 * Process the contents of the PingHeader
                 *
                 * @param hdr the XTF PingHeader
                 */
	        void processPingHeader(XtfPingHeader & hdr);

                /**
                 * Process the contents of the PingChanHeader
                 * 
                 * @param pingChanHdr the ping chan header
                 */
                void processPingChanHeader(XtfPingChanHeader & pingChanHdr);
                
                /**
                 * Process the contents of the FileHeader
                 *
                 * @param hdr the XTF FileHeader
                 */
	        void processFileHeader(XtfFileHeader & hdr);

                /**
                 * Process the contents of the file ChanInfo
                 *
                 * @param c the XTF ChanInfo
                 */
	        void processChanInfo(XtfChanInfo * c);

                /**
                 * Process sidescan data
                 * @param hdr 
                 * @param data
                 */
                void processSidescanData(XtfPingHeader & pingHdr,XtfPingChanHeader & pingChanHdr,void * data);
                
                
                /**
                 * Process Quinsy R2Sonic packets
                 */
                void processQuinsyR2SonicBathy(XtfPacketHeader & hdr,unsigned char * packet);

                /**the XTF FileHeader*/
		XtfFileHeader fileHeader;
                
                std::vector<XtfChanInfo*> channels;
                
                //FIXME: use a channel map to allow for different settings per channel
                unsigned int bytesPerSample;
                unsigned int sampleFormat;
};



#endif
