#ifndef XTFPARSER_HPP
#define XTFPARSER_HPP

#include <string>
#include <stdio.h>
#include <string.h>
#include <cstdio>
#include "XtfTypes.hpp"
#include "../DatagramParser.hpp"
#include "../../utils/TimeUtils.hpp"

#define MAGIC_NUMBER 123
#define PACKET_MAGIC_NUMBER 0xFACE

/**
 * @author Guillaume Morissette
 *
 */

/*!
 * \brief XTF parser class extention datagram parser
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
	        void processChanInfo(XtfChanInfo & c);

                /**the XTF FileHeader*/
		XtfFileHeader fileHeader;
};

/**
 * Create an XTF parser 
 * 
 * @param processor the datagram processor
 */
XtfParser::XtfParser(DatagramEventHandler & processor):DatagramParser(processor){

}

/**Destroy the XTF parser*/
XtfParser::~XtfParser(){

}

/**
 * Read a file and change the XTF parser depending on the information
 * 
 * @param filename name of the file to read
 */
void XtfParser::parse(std::string & filename){
	FILE * file = fopen(filename.c_str(),"rb");

        if(file){
                //Lire Header
		memset(&fileHeader,0,sizeof(XtfFileHeader));
		int elementsRead = fread (&fileHeader,sizeof(XtfFileHeader),1,file);

		if(elementsRead==1){
			if(fileHeader.FileFormat == MAGIC_NUMBER){

				processFileHeader(fileHeader);

				int channels = this->getTotalNumberOfChannels();

				//Lire structs CHANINFO dans le header
				int channelsInHeader = (channels > 6)?6:channels;

				for(int i=0;i<channelsInHeader;i++){
					processChanInfo(fileHeader.Channels[i]);
				}

				//Lire les structs CHANINFO qui suivent le header
				if(channels>6){
					int channelsLeft = channels;
					XtfChanInfo buf[8];

					do{
						memset(buf,0,sizeof(XtfChanInfo)*8);
						elementsRead = fread(&buf,sizeof(XtfChanInfo),8,file);

						if(elementsRead == 8){
							for(int i=0;i<8;i++){
								if(channelsLeft > 0){
									processChanInfo(buf[i]);
									channelsLeft--;
								}
								else{
									break;
								}
							}
						}
						else{
							//TODO: whine and log error while reading
							printf("Error while reading CHANINFO\n");
						}
					}
					while(channelsLeft > 0);
				}

				//Lire packets
				while(!feof(file)){
					// parse a packet header
					XtfPacketHeader packetHeader;

					elementsRead = fread (&packetHeader,sizeof(XtfPacketHeader),1,file);

					if(elementsRead == 1){
						if (packetHeader.MagicNumber==PACKET_MAGIC_NUMBER){
							processPacketHeader(packetHeader);

							unsigned char * packet = (unsigned char*) malloc(packetHeader.NumBytesThisRecord-sizeof(XtfPacketHeader));

							elementsRead = fread (packet,packetHeader.NumBytesThisRecord-sizeof(XtfPacketHeader),1,file);

							if(elementsRead == 1){
								processPacket(packetHeader,packet);
							}
							else{
								printf("Error while reading packet\n");
							}

							free(packet);
						}
						else{
							printf("Invalid packet header\n");
						}
					}
					else{
						//TODO: whine and log error while reading
						//printf("Error while reading packet header\n");
					}
				}
			}
			else{
				fclose(file);
				throw "Invalid file format";
			}
		}
		else{
			fclose(file);
			throw "Couldn't read from file";
		}

		fclose(file);
	}
	else{
		throw "File not found";
	}
}

std::string XtfParser::getName(int tag)
{
    switch(tag)
    {
        case 0:
            return "XTF_HEADER_SONAR";
        break;
        
        case 1:
            return "XTF_HEADER_NOTES";
        break;
        
        case 2:
            return "XTF_HEADER_BATHY";
        break;
        
        case 3:
            return "XTF_HEADER_ATTITUDE";
        break;
        
        case 4:
            return "XTF_HEADER_FORWARD";
        break;
        
        case 5:
            return "XTF_HEADER_ELAC";
        break;
        
        case 6:
            return "XTF_HEADER_RAW_SERIAL";
        break;
        
        case 7:
            return "XTF_HEADER_EMBED_HEAD";
        break;
        
        case 8:
            return "XTF_HEADER_HIDDEN_SONAR";
        break;
        
        case 9:
            return "XTF_HEADER_SEAVIEW_PROCESSED_BATHY";
        break;
        
        case 10:
            return "XTF_HEADER_SEAVIEW_DEPTHS";
        break;
        
        case 11:
            return "XTF_HEADER_RSVD_HIGHSPEED_SENSOR";
        break;
        
        case 12:
            return "XTF_HEADER_ECHOSTRENGTH";
        break;
        
        case 13:
            return "XTF_HEADER_GEOREC";
        break;
        
        case 14:
            return "XTF_HEADER_KLEIN_RAW_BATHY";
        break;
        
        case 15:
            return "XTF_HEADER_HIGHSPEED_SENSOR2";
        break;
        
        case 16:
            return "XTF_HEADER_ELAC_XSE";
        break;
        
        case 17:
            return "XTF_HEADER_BATHY_XYZA";
        break;
        
        case 18:
            return "XTF_HEADER_K5000_BATHY_IQ";
        break;
        
        case 19:
            return "XTF_HEADER_BATHY_SNIPPET";
        break;
        
        case 20:
            return "XTF_HEADER_GPS";
        break;
        
        case 21:
            return "XTF_HEADER_STAT";
        break;
        
        case 22:
            return "XTF_HEADER_SINGLEBEAM";
        break;
        
        case 23:
            return "XTF_HEADER_GYRO";
        break;
        
        case 24:
            return "XTF_HEADER_TRACKPOINT";
        break;
        
        case 25:
            return "XTF_HEADER_MULTIBEAM";
        break;
        
        case 26:
            return "XTF_HEADER_Q_SINGLEBEAM";
        break;
        
        case 27:
            return "XTF_HEADER_Q_MULTITX";
        break;
        
        case 28:
            return "XTF_HEADER_Q_MULTIBEAM";
        break;
        
        case 50:
            return "XTF_HEADER_TIME";
        break;
        
        case 60:
            return "XTF_HEADER_BENTHOS_CAATI_SARA";
        break;
        
        case 61:
            return "XTF_HEADER_7125";
        break;
        
        case 62:
            return "XTF_HEADER_7125_SNIPPET";
        break;
        
        case 65:
            return "XTF_HEADER_QINSY_R2SONIC_BATHY";
        break;
        
        case 66:
            return "XTF_HEADER_QINSY_R2SONIC_FTS";
        break;
        
        case 68:
            return "XTF_HEADER_R2SONIC_BATHY";
        break;
        
        case 69:
            return "XTF_HEADER_R2SONIC_FTS";
        break;
        
        case 70:
            return "XTF_HEADER_CODA_ECHOSCOPE_DATA";
        break;
        
        case 71:
            return "XTF_HEADER_CODA_ECHOSCOPE_CONFIG";
        break;
        
        case 72:
            return "XTF_HEADER_CODA_ECHOSCOPE_IMAGE";
        break;
        
        case 73:
            return "XTF_HEADER_EDGETECH_4600";
        break;
        
        case 78:
            return "XTF_HEADER_RESON_7018_WATERCOLUMN";
        break;
        
        case 100:
            return "XTF_HEADER_POSITION";
        break;
        
        case 102:
            return "XTF_HEADER_BATHY_PROC";
        break;
        
        case 103:
            return "XTF_HEADER_ATTITUDE_PROC";
        break;
        
        case 104:
            return "XTF_HEADER_SINGLEBEAM_PROC";
        break;
        
        case 105:
            return "XTF_HEADER_AUX_PROC";
        break;
        
        case 107:
            return "XTF_HEADER_POS_RAW_NAVIGATION";
        break;
        
        case 108:
            return "XTF_HEADER_KLEINV4_DATA_PAGE";
        break;
        
        case 200:
            return "XTF_HEADER_USERDEFINED";
        break;
        
        default:
            return "Invalid tag";
	break;
    }
}

/**Return the number channels in the file header*/
int XtfParser::getTotalNumberOfChannels(){
	return  this->fileHeader.NumberOfSonarChannels+
		this->fileHeader.NumberOfBathymetryChannels+
		this->fileHeader.NumberOfSnippetChannels+
		this->fileHeader.NumberOfEchoStrengthChannels+
		this->fileHeader.NumberOfInterferometryChannels;
};

/**
 * show the contain of the file FileHeader
 * 
 * @param f the XTF FileHeader
 */
void XtfParser::processFileHeader(XtfFileHeader & f){
    /*
        printf("------------\n");
        printf("FileFormat: %d\n",f.FileFormat);
        printf("SystemType: %d\n",f.SystemType);
        printf("RecordingProgramName: %s\n",f.RecordingProgramName);
        printf("RecordingProgramVersion: %s\n",f.RecordingProgramVersion);
        printf("SonarName: %s\n",f.SonarName);
        printf("sonarType: %d (%s)\n",f.SonarType,SonarTypes[f.SonarType].c_str());
        printf("NoteString: %s\n",f.NoteString);
        printf("ThisFileName: %s\n",f.ThisFileName);
        printf("NavUnits: %d\n",f.NavUnits);
        printf("NumberOfSonarChannels: %d\n",f.NumberOfSonarChannels);
        printf("NumberOfBathymetryChannels: %d\n",f.NumberOfBathymetryChannels);
        printf("NumberOfSnippetChannels: %d\n",f.NumberOfSnippetChannels);
        printf("NumberOfForwardLookArrays: %d\n",f.NumberOfForwardLookArrays);
        printf("NumberOfEchoStrengthChannels: %d\n",f.NumberOfEchoStrengthChannels);
        printf("NumberOfInterferometryChannels: %d\n",f.NumberOfInterferometryChannels);
        printf("Reserved1: %d\n",f.Reserved1);
        printf("Reserved2: %d\n",f.Reserved2);
        printf("ReferencePointHeight: %f\n",f.ReferencePointHeight);
        //TODO
        //printf("ProjectionType: ");
        //print(f.ProjectionType,12);
        //printf("\n");
        //printf("SpheriodType: ");
        //print(f.SpheriodType,10);
        //printf("\n");

        printf("NavigationLatency: %d\n",f.NavigationLatency);
        printf("OriginY: %f\n",f.OriginY);
        printf("OriginX: %f\n",f.OriginX);
        printf("NavOffsetY: %f\n",f.NavOffsetY);
        printf("NavOffsetX: %f\n",f.NavOffsetX);
        printf("NavOffsetZ: %f\n",f.NavOffsetZ);
        printf("NavOffsetYaw: %f\n",f.NavOffsetYaw);
        printf("MRUOffsetY: %f\n",f.MRUOffsetY);
        printf("MRUOffsetX: %f\n",f.MRUOffsetX);
        printf("MRUOffsetZ: %f\n",f.MRUOffsetZ);
        printf("MRUOffsetYaw: %f\n",f.MRUOffsetYaw);
        printf("MRUOffsetPitch: %f\n",f.MRUOffsetPitch);
        printf("MRUOffsetRoll: %f\n",f.MRUOffsetRoll);
        printf("------------\n");
        */
}

/**
 * show the contain of the file ChanInfo
 * 
 * @param c the XTF ChanInfo
 */
void XtfParser::processChanInfo(XtfChanInfo & c){
    /*
        printf("------------\n");
        printf("TypeOfChannel: %d\n",c.TypeOfChannel);
        printf("SubChannelNumber: %d\n",c.SubChannelNumber);
        printf("CorrectionFlags: %d\n",c.CorrectionFlags);
        printf("UniPolar: %d\n",c.UniPolar);
        printf("BytesPerSample: %d\n",c.BytesPerSample);
        printf("Reserved: %d\n",c.Reserved);
        printf("ChannelName: %s\n",c.ChannelName);
        printf("VoltScale: %f\n",c.VoltScale);
        printf("Frequency: %f\n",c.Frequency);
        printf("HorizBeamAngle: %f\n",c.HorizBeamAngle);
        printf("TiltAngle: %f\n",c.TiltAngle);
        printf("BeamWidth: %f\n",c.BeamWidth);
        printf("OffsetX: %f\n",c.OffsetX));
        printf("OffsetY: %f\n",c.OffsetY);
        printf("OffsetZ: %f\n",c.OffsetZ);
        printf("OffsetYaw: %f\n",c.OffsetYaw);
        printf("OffsetPitch: %f\n",c.OffsetPitch);
        printf("OffsetRoll: %f\n",c.OffsetRoll);
        printf("BeamsPerArray: %d\n",c.BeamsPerArray);
        printf("ReservedArea2: %s\n",c.ReservedArea2);
        printf("------------\n");
        */
}

/**
 * show the contain of the file PacketHeader
 * 
 * @param hdr the XTF PacketHeader
 */
void XtfParser::processPacketHeader(XtfPacketHeader & hdr){
    /*
        printf("------------\n");
        printf("MagicNumber: %d (%s)\n",hdr.MagicNumber,(hdr.MagicNumber==PACKET_MAGIC_NUMBER)?"OK":"FAIL");
        printf("HeaderType: %d\n",hdr.HeaderType);
        printf("SubChannelNumber: %d\n",hdr.SubChannelNumber);
        printf("NumChansToFollow: %d\n",hdr.NumChansToFollow);
        //printf("Reserved: %",x.Reserved);
        printf("NumBytesThisRecord: %d\n",hdr.NumBytesThisRecord);
        printf("------------\n");
*/
}

/**
 * show the contain of the file PingHeader
 * 
 * @param hdr the XTF PingHeader
 */
void XtfParser::processPingHeader(XtfPingHeader & hdr){
    processor.processSwathStart(hdr.SoundVelocity);
}

/**
 * Set the processor depending by the content of the XTF Packet header
 * 
 * @param hdr the XTF Packet Header 
 * @param packet the packet
 */
void XtfParser::processPacket(XtfPacketHeader & hdr,unsigned char * packet){
	processor.processDatagramTag(hdr.HeaderType);

	if(hdr.HeaderType==XTF_HEADER_ATTITUDE){
		uint64_t microEpoch = 0;
		XtfAttitudeData* attitude = (XtfAttitudeData*)packet;

		if(attitude->SourceEpoch){
			microEpoch = attitude->SourceEpoch * 1000000 + attitude->EpochMicroseconds;
		}
		else{
			microEpoch = TimeUtils::build_time(attitude->Year,attitude->Month-1,attitude->Day,attitude->Hour,attitude->Minutes,attitude->Seconds,attitude->Milliseconds,0);
		}

        	processor.processAttitude(
			microEpoch,
			attitude->Heading,
			(attitude->Pitch < 0) ? attitude->Pitch + 360 : attitude->Pitch,
			(attitude->Roll  < 0) ? attitude->Roll  + 360 : attitude->Pitch
		);

	}
	else if(hdr.HeaderType==XTF_HEADER_Q_MULTIBEAM){
		XtfPingHeader * pingHdr = (XtfPingHeader*) packet;

		processPingHeader(*pingHdr);

	        //printf("%d Pings\n",hdr.NumChansToFollow);

		XtfQpsMbEntry * ping = (XtfQpsMbEntry*) ((uint8_t*)packet + sizeof(XtfPingHeader));

	        uint64_t microEpoch = TimeUtils::build_time(pingHdr->Year,pingHdr->Month-1,pingHdr->Day,pingHdr->Hour,pingHdr->Minute,pingHdr->Second,pingHdr->HSeconds * 10,0);

		for(unsigned int i = 0;i < hdr.NumChansToFollow;i++){
            		processor.processPing(microEpoch,ping[i].Id,ping[i].BeamAngle,ping[i].TiltAngle,ping[i].TwoWayTravelTime,ping[i].Quality,ping[i].Intensity);
		}
	}
	else if(hdr.HeaderType==XTF_HEADER_POSITION){
		XtfPosRawNavigation* position = (XtfPosRawNavigation*)packet;
        	uint64_t microEpoch = TimeUtils::build_time(position->Year,position->Month-1,position->Day,position->Hour,position->Minutes,position->Seconds,position->MicroSeconds/1000,position->MicroSeconds%1000);
        	processor.processPosition(microEpoch,position->RawXcoordinate,position->RawYcoordinate,position->RawAltitude);
	}
	else{
		printf("Unknown packet type: %d\n",hdr.HeaderType);
	}
}

#endif
