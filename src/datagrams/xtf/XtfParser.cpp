/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef XTFPARSER_CPP
#define XTFPARSER_CPP

#include "XtfParser.hpp"


/**
 * @author Guillaume Morissette
 */

/**
 * Create an XTF parser
 *
 * @param processor the datagram processor
 */
XtfParser::XtfParser(DatagramEventHandler & processor):DatagramParser(processor){

}

/**Destroy the XTF parser*/
XtfParser::~XtfParser(){
    for(auto i=channels.begin();i!=channels.end();i++){
        free(*i);
    }
}

/**
 * Read a file and change the XTF parser depending on the information
 *
 * @param filename name of the file to read
 */
void XtfParser::parse(std::string & filename, bool ignoreChecksum){
    
    //TODO: reinit internal structures if called twice
    
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
                                    processChanInfo(&fileHeader.Channels[i]);
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
									processChanInfo(&buf[i]);
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
				throw new Exception("Invalid file format");
			}
		}
		else{
			fclose(file);
			throw new Exception("Couldn't read from file");
		}

		fclose(file);
	}
	else{
		throw new Exception("File not found");
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

        case 106:
            return "XTF_HEADER_KLEIN3000_DATA_PAGE";
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
        //fprintf(stderr,"[+] XTF File Header:\n\n");        
        //fprintf(stderr,"FileFormat: %d\n",f.FileFormat);
        //fprintf(stderr,"SystemType: %d\n",f.SystemType);
        //fprintf(stderr,"RecordingProgramName: %s\n",f.RecordingProgramName);
        //fprintf(stderr,"RecordingProgramVersion: %s\n",f.RecordingProgramVersion);
        //fprintf(stderr,"SonarName: %s\n",f.SonarName);
        //fprintf(stderr,"sonarType: %d (%s)\n",f.SonarType,SonarTypes[f.SonarType].c_str());
        //fprintf(stderr,"NoteString: %s\n",f.NoteString);
        //fprintf(stderr,"ThisFileName: %s\n",f.ThisFileName);
        //fprintf(stderr,"NavUnits: %d\n",f.NavUnits);
        //fprintf(stderr,"NumberOfSonarChannels: %d\n",f.NumberOfSonarChannels);
        //fprintf(stderr,"NumberOfBathymetryChannels: %d\n",f.NumberOfBathymetryChannels);
        //fprintf(stderr,"NumberOfSnippetChannels: %d\n",f.NumberOfSnippetChannels);
        //fprintf(stderr,"NumberOfForwardLookArrays: %d\n",f.NumberOfForwardLookArrays);
        //fprintf(stderr,"NumberOfEchoStrengthChannels: %d\n",f.NumberOfEchoStrengthChannels);
        //fprintf(stderr,"NumberOfInterferometryChannels: %d\n",f.NumberOfInterferometryChannels);
        //fprintf(stderr,"Reserved1: %d\n",f.Reserved1);
        //fprintf(stderr,"Reserved2: %d\n",f.Reserved2);
        //fprintf(stderr,"ReferencePointHeight: %f\n",f.ReferencePointHeight);
        
        std::map<std::string,std::string> * fileProperties = new std::map<std::string,std::string>();
        
        fileProperties->insert(std::pair<std::string,std::string>("Channels (Sonar)",std::to_string(f.NumberOfSonarChannels)));
        fileProperties->insert(std::pair<std::string,std::string>("Channels (Bathymetry)",std::to_string(f.NumberOfBathymetryChannels)));
        fileProperties->insert(std::pair<std::string,std::string>("Channels (Snippet)",std::to_string(f.NumberOfSnippetChannels)));
        fileProperties->insert(std::pair<std::string,std::string>("Channels (Interferometry)",std::to_string(f.NumberOfInterferometryChannels)));
        fileProperties->insert(std::pair<std::string,std::string>("Channels (Forward Look)",std::to_string(f.NumberOfForwardLookArrays)));
        fileProperties->insert(std::pair<std::string,std::string>("Channels (Echo Strength)",std::to_string(f.NumberOfEchoStrengthChannels)));
        fileProperties->insert(std::pair<std::string,std::string>("File Format",std::to_string(f.FileFormat)));
        fileProperties->insert(std::pair<std::string,std::string>("System Type",std::to_string(f.SystemType)));
        fileProperties->insert(std::pair<std::string,std::string>("Recording Program Name",f.RecordingProgramName));
        fileProperties->insert(std::pair<std::string,std::string>("Recording Program Version",f.RecordingProgramVersion));
        fileProperties->insert(std::pair<std::string,std::string>("Sonar Name",f.SonarName));
        fileProperties->insert(std::pair<std::string,std::string>("Sonar Type",SonarTypes[f.SonarType]));
        fileProperties->insert(std::pair<std::string,std::string>("Note String",f.NoteString));
        fileProperties->insert(std::pair<std::string,std::string>("Nav Units",std::to_string(f.NavUnits)));
        fileProperties->insert(std::pair<std::string,std::string>("Original File Name",f.ThisFileName));
        fileProperties->insert(std::pair<std::string,std::string>("Reference Point Height",std::to_string(f.ReferencePointHeight)));
        fileProperties->insert(std::pair<std::string,std::string>("Origin Y",std::to_string(f.OriginY)));
        fileProperties->insert(std::pair<std::string,std::string>("Origin X",std::to_string(f.OriginX)));
        fileProperties->insert(std::pair<std::string,std::string>("Nav Offset X",std::to_string(f.NavOffsetX)));
        fileProperties->insert(std::pair<std::string,std::string>("Nav Offset Y",std::to_string(f.NavOffsetY)));
        fileProperties->insert(std::pair<std::string,std::string>("Nav Offset Z",std::to_string(f.NavOffsetZ)));
        fileProperties->insert(std::pair<std::string,std::string>("Nav Offset Yaw",std::to_string(f.NavOffsetYaw)));
        fileProperties->insert(std::pair<std::string,std::string>("MRU Offset X",std::to_string(f.MRUOffsetX)));
        fileProperties->insert(std::pair<std::string,std::string>("MRU Offset Y",std::to_string(f.MRUOffsetY)));
        fileProperties->insert(std::pair<std::string,std::string>("MRU Offset Z",std::to_string(f.MRUOffsetZ)));        
        fileProperties->insert(std::pair<std::string,std::string>("MRU Offset Yaw",std::to_string(f.MRUOffsetYaw)));
        fileProperties->insert(std::pair<std::string,std::string>("MRU Offset Pitch",std::to_string(f.MRUOffsetPitch)));
        fileProperties->insert(std::pair<std::string,std::string>("MRU Offset Roll",std::to_string(f.MRUOffsetRoll)));         
        

        processor.processFileProperties(fileProperties);
        
        //TODO
        //printf("ProjectionType: ");
        //print(f.ProjectionType,12);
        //printf("\n");
        //printf("SpheriodType: ");
        //print(f.SpheriodType,10);
        //printf("\n");

        //fprintf(stderr,"NavigationLatency: %d\n",f.NavigationLatency);
        //fprintf(stderr,"OriginY: %f\n",f.OriginY);
        //fprintf(stderr,"OriginX: %f\n",f.OriginX);
        //fprintf(stderr,"NavOffsetY: %f\n",f.NavOffsetY);
        //fprintf(stderr,"NavOffsetX: %f\n",f.NavOffsetX);
        //fprintf(stderr,"NavOffsetZ: %f\n",f.NavOffsetZ);
        //fprintf(stderr,"NavOffsetYaw: %f\n",f.NavOffsetYaw);
        //fprintf(stderr,"MRUOffsetY: %f\n",f.MRUOffsetY);
        //fprintf(stderr,"MRUOffsetX: %f\n",f.MRUOffsetX);
        //fprintf(stderr,"MRUOffsetZ: %f\n",f.MRUOffsetZ);
        //fprintf(stderr,"MRUOffsetYaw: %f\n",f.MRUOffsetYaw);
        //fprintf(stderr,"MRUOffsetPitch: %f\n",f.MRUOffsetPitch);
        //fprintf(stderr,"MRUOffsetRoll: %f\n",f.MRUOffsetRoll);
        //fprintf(stderr,"------------\n\n");
}

/**
 * show the contain of the file ChanInfo
 *
 * @param c the XTF ChanInfo
 */
void XtfParser::processChanInfo(XtfChanInfo * c){
    XtfChanInfo * channel = (XtfChanInfo *) malloc(sizeof(XtfChanInfo));
    memcpy(channel,c,sizeof(XtfChanInfo));
    
    channels.push_back(channel);
    
    //fprintf(stderr,"[+] XTF Channel Information\n\n");
    //fprintf(stderr,"TypeOfChannel: %d\n",channel->TypeOfChannel);
    //fprintf(stderr,"SubChannelNumber: %d\n",channel->SubChannelNumber);
    //fprintf(stderr,"CorrectionFlags: %d\n",channel->CorrectionFlags);
    //fprintf(stderr,"UniPolar: %d\n",channel->UniPolar);
    //fprintf(stderr,"BytesPerSample: %d\n",channel->BytesPerSample);
    //fprintf(stderr,"Reserved: %d\n",channel->Reserved);
    //fprintf(stderr,"ChannelName: %s\n",channel->ChannelName);
    //fprintf(stderr,"VoltScale: %f\n",channel->VoltScale);
    //fprintf(stderr,"Frequency: %f\n",channel->Frequency);
    //fprintf(stderr,"HorizBeamAngle: %f\n",channel->HorizBeamAngle);
    //fprintf(stderr,"TiltAngle: %f\n",channel->TiltAngle);
    //fprintf(stderr,"BeamWidth: %f\n",channel->BeamWidth);
    //fprintf(stderr,"OffsetX: %f\n",channel->OffsetX);
    //fprintf(stderr,"OffsetY: %f\n",channel->OffsetY);
    //fprintf(stderr,"OffsetZ: %f\n",channel->OffsetZ);
    //fprintf(stderr,"OffsetYaw: %f\n",channel->OffsetYaw);
    //fprintf(stderr,"OffsetPitch: %f\n",channel->OffsetPitch);
    //fprintf(stderr,"OffsetRoll: %f\n",channel->OffsetRoll);
    //fprintf(stderr,"BeamsPerArray: %d\n",channel->BeamsPerArray);
    //fprintf(stderr,"SampleFormat: %d\n",channel->SampleFormat);
    //fprintf(stderr,"ReservedArea2: %s\n",channel->ReservedArea2);
    //fprintf(stderr,"------------\n");
    
    std::map<std::string,std::string> * properties = new std::map<std::string,std::string>();
    
    properties->insert(std::pair<std::string,std::string>("Channel Type",std::to_string(channel->TypeOfChannel)));
    properties->insert(std::pair<std::string,std::string>("Channel Number",std::to_string(channel->SubChannelNumber)));
    properties->insert(std::pair<std::string,std::string>("Correction Flags",std::to_string(channel->CorrectionFlags)));
    properties->insert(std::pair<std::string,std::string>("UniPolar",std::to_string(channel->UniPolar)));
    properties->insert(std::pair<std::string,std::string>("Bytes Per Sample",std::to_string(channel->BytesPerSample)));
    properties->insert(std::pair<std::string,std::string>("Channel Name",channel->ChannelName));
    properties->insert(std::pair<std::string,std::string>("Volt Scale",std::to_string(channel->VoltScale)));
    properties->insert(std::pair<std::string,std::string>("Frequency",std::to_string(channel->Frequency)));
    properties->insert(std::pair<std::string,std::string>("Horizontal Beam Angle",std::to_string(channel->HorizBeamAngle)));
    properties->insert(std::pair<std::string,std::string>("Tilt Angle",std::to_string(channel->TiltAngle)));
    properties->insert(std::pair<std::string,std::string>("Beam Width",std::to_string(channel->BeamWidth)));
    properties->insert(std::pair<std::string,std::string>("Offset X",std::to_string(channel->OffsetX)));
    properties->insert(std::pair<std::string,std::string>("Offset Y",std::to_string(channel->OffsetY)));
    properties->insert(std::pair<std::string,std::string>("Offset Z",std::to_string(channel->OffsetZ)));
    properties->insert(std::pair<std::string,std::string>("Offset Yaw",std::to_string(channel->OffsetYaw)));
    properties->insert(std::pair<std::string,std::string>("Offset Pitch",std::to_string(channel->OffsetPitch)));
    properties->insert(std::pair<std::string,std::string>("Offset Roll",std::to_string(channel->OffsetRoll)));
    properties->insert(std::pair<std::string,std::string>("Beams Per Array",std::to_string(channel->BeamsPerArray)));
    properties->insert(std::pair<std::string,std::string>("Sample Format",std::to_string(channel->SampleFormat)));
    
    processor.processChannelProperties(channel->SubChannelNumber,channel->ChannelName,channel->TypeOfChannel,properties);
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
 * @param hdr the XTF PingHeader
 */
void XtfParser::processPingHeader(XtfPingHeader & hdr){
    
    double sss = hdr.SoundVelocity;
    
    if(hdr.SoundVelocity < 800){
        //Not the best check, but if it's using the one-way format, we'll want to double it up
        sss *= 2;
    }
    
    processor.processSwathStart(sss);
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

        	microEpoch = TimeUtils::build_time(
                        attitude->Year,
                        attitude->Month-1,
                        attitude->Day,
                        attitude->Hour,
                        attitude->Minutes,
                        attitude->Seconds,
                        attitude->Milliseconds,
                        0);

        	processor.processAttitude(
			microEpoch,
			attitude->Heading,
			(attitude->Pitch < 0) ? attitude->Pitch + 360 : attitude->Pitch,
			(attitude->Roll  < 0) ? attitude->Roll  + 360 : attitude->Roll
		);

	}
	else if(hdr.HeaderType==XTF_HEADER_Q_MULTIBEAM){
		XtfPingHeader * pingHdr = (XtfPingHeader*) packet;

		processPingHeader(*pingHdr);

	        //printf("%d Pings\n",hdr.NumChansToFollow);

		XtfQpsMbEntry * ping = (XtfQpsMbEntry*) ((uint8_t*)packet + sizeof(XtfPingHeader));

	        uint64_t microEpoch = TimeUtils::build_time(
                        pingHdr->Year,
                        pingHdr->Month-1,
                        pingHdr->Day,
                        pingHdr->Hour,
                        pingHdr->Minute,
                        pingHdr->Second,
                        pingHdr->HSeconds * 10,
                        0
                );

		for(unsigned int i = 0;i < hdr.NumChansToFollow;i++){
            		processor.processPing(
                            microEpoch + (ping[i].DeltaTime * 1000000),
                            ping[i].Id,
                            ping[i].BeamAngle,
                            ping[i].TiltAngle,
                            ping[i].TwoWayTravelTime,
                            ping[i].Quality,
                            ping[i].Intensity
                        );
		}
	}
	else if(hdr.HeaderType==XTF_HEADER_POSITION){
		XtfPosRawNavigation* position = (XtfPosRawNavigation*)packet;

        	uint64_t microEpoch = TimeUtils::build_time(
                        position->Year,
                        position->Month-1,
                        position->Day,
                        position->Hour,
                        position->Minutes,
                        position->Seconds,
                        0,
                        position->TenthsOfMilliseconds *100
                );

        	processor.processPosition(
                        microEpoch,
                        position->RawXcoordinate,
                        position->RawYcoordinate,
                        position->RawAltitude
                );
	}
        else if(hdr.HeaderType==XTF_HEADER_POS_RAW_NAVIGATION){
		XtfHeaderNavigation_type42 * position = (XtfHeaderNavigation_type42*)packet;

        	uint64_t microEpoch = TimeUtils::build_time(
                        position->Year,
                        position->Month-1,
                        position->Day,
                        position->Hour,
                        position->Minute,
                        position->Second,
                        0,
                        position->Microseconds
                );

        	processor.processPosition(
                        microEpoch,
                        position->RawXCoordinate,
                        position->RawYCoordinate,
                        position->RawAltitude
                );
        }
        else if(hdr.HeaderType==XTF_HEADER_RESON_REMOTE_CONTROL_SETTINGS) {
            //Custom raw packets have a header packets that differ
            //XtfRawCustomHeaderLastPart * rawCustomHeader = (XtfRawCustomHeaderLastPart*) packet;
            
            //ATTENTION: SubChannelNumber is used to define ManufacturerID in custom Raw packets
            if(hdr.SubChannelNumber == XTF_RESON_MANUFACTURER_ID) { 
                processResonSettingsDatagram(hdr,packet+sizeof(XtfRawCustomHeaderLastPart));
            }
        }
        else if(hdr.HeaderType==XTF_HEADER_RESON_BATHY) {
            XtfPingHeader * pingHdr = (XtfPingHeader*) packet;
            processPingHeader(*pingHdr);
            processReson7027Bathy(hdr,packet+sizeof(XtfPingHeader));
        }
        else if(hdr.HeaderType==XTF_HEADER_QUINSY_R2SONIC_BATHY){
		XtfPingHeader * pingHdr = (XtfPingHeader*) packet;
		processPingHeader(*pingHdr);
                processQuinsyR2SonicBathy(hdr,packet+sizeof(XtfPingHeader));
        }
        else if(hdr.HeaderType==XTF_HEADER_SONAR){
            unsigned int sampleBytesRead = 0;

            //sidescan data
            XtfPingHeader * pingHdr = (XtfPingHeader*) packet;
            
            processPingHeader(*pingHdr);
            
            for(unsigned int i=0;i<hdr.NumChansToFollow;i++){
                XtfPingChanHeader * pingChanHdr = (XtfPingChanHeader *) (packet+sizeof(XtfPingHeader) + i*sizeof(XtfPingChanHeader) + sampleBytesRead);
                processPingChanHeader(*pingChanHdr);
                
                unsigned char * data = ((unsigned char *)pingChanHdr) + sizeof(XtfPingChanHeader);
                
                processSidescanData(*pingHdr,*pingChanHdr,data);
                
                sampleBytesRead += pingChanHdr->NumSamples * channels[pingChanHdr->ChannelNumber]->BytesPerSample;
            }
        }
	else{
		std::cerr << "Unknown packet type: " << (int)hdr.HeaderType << std::endl;
	}
}

void XtfParser::processPingChanHeader(XtfPingChanHeader & pingChanHdr){
    
}

void XtfParser::processSidescanData(XtfPingHeader & pingHdr,XtfPingChanHeader & pingChanHdr,void * data){   
    std::vector<double> rawSamples; //we will boil down all the types to double. This is not a pretty hack, but we need to support every sample type
    
    for(unsigned int i=0;i<pingChanHdr.NumSamples;i++){
        double sample = 0;
        
        if(channels[pingChanHdr.ChannelNumber]->SampleFormat == 0){
            //legacy
            if(channels[pingChanHdr.ChannelNumber]->BytesPerSample == 1){
                sample = ((uint8_t*)data)[i];
            }
            else if(channels[pingChanHdr.ChannelNumber]->BytesPerSample == 2){
                sample = ((uint16_t*)data)[i];
            }
            else if(channels[pingChanHdr.ChannelNumber]->BytesPerSample == 4){
                sample = ((uint32_t*)data)[i];
            }
            else{
                std::cerr << "[-] Bytes per sample: " << channels[pingChanHdr.ChannelNumber]->BytesPerSample << std::endl;
                throw new std::invalid_argument("Bad bytes per sample format");                
            }
        }
        else if(channels[pingChanHdr.ChannelNumber]->SampleFormat == 1){
            //TODO: wtf is an "IBM float" in C?
            throw new std::invalid_argument("[-] Sample format is IBM float");
        }
        else if(channels[pingChanHdr.ChannelNumber]->SampleFormat == 2){
            sample = ((uint32_t*)data)[i];
        }
        else if(channels[pingChanHdr.ChannelNumber]->SampleFormat == 3){
            sample = ((uint16_t*)data)[i];
        }
        else if(channels[pingChanHdr.ChannelNumber]->SampleFormat == 5){
            sample = ((float*)data)[i];
        }
        else if(channels[pingChanHdr.ChannelNumber]->SampleFormat == 8){
            sample = ((uint8_t*)data)[i];
        }        
        else{
            std::cerr << "[-] Sample Format: " << channels[pingChanHdr.ChannelNumber]->SampleFormat << std::endl;
            throw new std::invalid_argument("Sample format unused");
        }

        rawSamples.push_back(sample);
    }
    
    
    SidescanPing * ping = new SidescanPing();
    
    uint64_t microEpoch = TimeUtils::build_time(
                pingHdr.Year,
                pingHdr.Month-1,
                pingHdr.Day,
                pingHdr.Hour,
                pingHdr.Minute,
                pingHdr.Second,
                pingHdr.HSeconds * 10,
                0
        );
    
    ping->setTimestamp(microEpoch);
    
    if(pingHdr.SensorXcoordinate != 0.0 && pingHdr.SensorYcoordinate != 0.0){ //this would cause weird issues at coordinates... (0.0,0.0)
        ping->setPosition(
            new Position(
                    microEpoch,
                    // pingHdr.SensorXcoordinate,
                    // pingHdr.SensorYcoordinate,                    
                    pingHdr.SensorYcoordinate,
                    pingHdr.SensorXcoordinate,                  
                    pingHdr.SensorPrimaryAltitude
            )
        );
        
        //also get layback and sensor depth
        ping->setLayback(pingHdr.Layback);
        ping->setSensorDepth(pingHdr.SensorDepth);
    }
    
    ping->setChannelNumber(pingChanHdr.ChannelNumber);
    
    if(channels[pingChanHdr.ChannelNumber]->CorrectionFlags == 2){
        //ground ranged images, use as-is
        ping->setSamples(rawSamples);
        ping->setDistancePerSample(pingChanHdr.GroundRange/(double)rawSamples.size());
    }
    else{       
        //Slant-range image, apply corrections to raw samples
        std::vector<double> correctedSamples;

        //Get beam angle , between nadir and slant        
        double beamAngle = 20;

        if(channels[pingChanHdr.ChannelNumber]->TiltAngle > 0){
            beamAngle = channels[pingChanHdr.ChannelNumber]->TiltAngle;
        }
        
        //Apply corrections
        SlantRangeCorrection::correct(rawSamples,pingChanHdr.SlantRange,0,beamAngle,correctedSamples);
        
        ping->setSamples(correctedSamples);
        ping->setDistancePerSample((double)pingChanHdr.SlantRange/(double)rawSamples.size());
        
        processor.processSidescanData(ping);
    }
}

void XtfParser::processResonSettingsDatagram(XtfPacketHeader & hdr, unsigned char * data) {
    //S7kDataRecordFrame * drf = (S7kDataRecordFrame*) data;
    //skip the Data Record Frame
    S7kSonarSettings * settings = (S7kSonarSettings*) (data+sizeof(S7kDataRecordFrame));
    S7kSonarSettings * settingsCopy = (S7kSonarSettings *) malloc(sizeof (S7kSonarSettings));
    memcpy(settingsCopy, settings, sizeof (S7kSonarSettings));
    pingSettings.push_back(settingsCopy);
}

void XtfParser::processReson7027Bathy(XtfPacketHeader & hdr,unsigned char * packet) {
    
    S7kDataRecordFrame * drf = (S7kDataRecordFrame*) packet;
    
    if(drf->RecordTypeIdentifier == 7027) { //Reson bathy packet
        
        long microSeconds = drf->Timestamp.Seconds * 1e6;
        uint64_t microEpoch = TimeUtils::build_time(
                drf->Timestamp.Year,
                drf->Timestamp.Day,
                drf->Timestamp.Hours,
                drf->Timestamp.Minutes,
                microSeconds);
        
        S7kRawDetectionDataRTH *swath = (S7kRawDetectionDataRTH*) (packet+sizeof(S7kDataRecordFrame));
        
        uint32_t nEntries = swath->numberOfDetectionPoints;
        double tiltAngle = swath->transmissionAngle*R2D;
        double samplingRate = swath->samplingRate;
        
        S7kSonarSettings * settings = NULL;
        for (auto i = pingSettings.begin(); i != pingSettings.end(); i++) {
            if ((*i)->sequentialNumber == swath->pingNumber) {
                settings = (*i);
                pingSettings.remove((*i));
                break;
            }
        }
        
        if (settings) {
            double surfaceSoundVelocity = settings->soundVelocity;

            processor.processSwathStart(surfaceSoundVelocity);

            for (unsigned int i = 0; i < nEntries; i++) {
                S7kRawDetectionDataRD *ping = (S7kRawDetectionDataRD*) (packet + sizeof(S7kDataRecordFrame) + sizeof (S7kRawDetectionDataRTH) + i * swath->dataFieldSize);
                double twoWayTravelTime = (double) ping->detectionPoint / samplingRate; // see Appendix F p. 190
                double intensity = swath->dataFieldSize > 22 ? ping->signalStrength : 0;
                processor.processPing(microEpoch, (long) ping->beamDescriptor, (double) ping->receptionAngle*R2D, tiltAngle, twoWayTravelTime, ping->quality, intensity);
            }

            free(settings);
        } else {
            
            std::cerr << "No settings for ping #" << swath->pingNumber << std::endl;
        }

    } else {
        std::cerr << "Unknown packet Type: " << ((S7kDataRecordFrame*)packet)->RecordTypeIdentifier << std::endl;
    }
}


/**
 * Processes a QUINSy R2Sonic packet
 *
 * BEWARE: While the XTF datagrams are little-endian, this packet is in BIG-ENDIAN. Because life is short and we're all going to die soon enough.
 * @param hdr
 * @param packet
 */
void XtfParser::processQuinsyR2SonicBathy(XtfPacketHeader & hdr,unsigned char * packet){

    if(htonl(((XtfHeaderQuinsyR2SonicBathy*)packet)->PacketName)==0x42544830){ //BTH0
        uint32_t nbBytes = htonl(((XtfHeaderQuinsyR2SonicBathy*)packet)->PacketSize);

        unsigned int packetIndex = sizeof(XtfHeaderQuinsyR2SonicBathy); //start after the header

        uint16_t nbBeams = 0;

        std::vector<Ping> pings;

        while(packetIndex < nbBytes){
            uint16_t sectionName  =  htons( * ((uint16_t*) (packet + packetIndex)));
            uint16_t sectionBytes =  htons( * ((uint16_t*) (packet + packetIndex + sizeof(uint16_t))));

            //printf("%c%c (%u bytes)\n",((char*)&sectionName)[1],((char*)&sectionName)[0],sectionBytes);

            if(sectionName==0x4830){
                //H0 - Main header
                XtfHeaderQuinsyR2SonicBathy_H0 * h0 = (XtfHeaderQuinsyR2SonicBathy_H0*) (packet + packetIndex);
                nbBeams = htons(h0->Points);
                uint64_t microEpoch =  ((uint64_t)htonl(h0->TimeSeconds)*(uint64_t)1000000) + ((uint64_t)htonl((uint64_t)h0->TimeNanoseconds)/(uint64_t)1000);

                //Init ping array
                for(unsigned int i=0;i<nbBeams;i++){
                    Ping p(i);
                    p.setTimestamp(microEpoch);
                    pings.push_back(p);
                }

                //surfaceSoundSpeed = htonl( * ((uint32_t*) & h0->SoundSpeed));
            }
            else if(sectionName==0x4130){
                //A0 - equi-angle mode
                XtfHeaderQuinsyR2SonicBathy_A0 * a0 = (XtfHeaderQuinsyR2SonicBathy_A0*) (packet + packetIndex);
                uint32_t first = htonl( *((uint32_t*) &a0->AngleFirst) );
                uint32_t last  = htonl( *((uint32_t*) &a0->AngleLast) );

                double step = (   (*((float*)&first))   -   (*((float*)&last))   )/(double)nbBeams;

                double angle = (*((float*)&first));

                for(unsigned int i =0; i < nbBeams ;i++ ){
                    pings[i].setAcrossTrackAngle(angle);
                    angle += step;
                }
            }
            else if(sectionName==0x4132){
                //A2 - equidistant angle mode
                XtfHeaderQuinsyR2SonicBathy_A2 * a2 = (XtfHeaderQuinsyR2SonicBathy_A2*) (packet + packetIndex);
                uint32_t first         = htonl( *((uint32_t*)&a2->AngleFirst));
                float    angleFirst    = *((float*)&first);

                uint32_t scale         = htonl( *((uint32_t*)&a2->ScalingFactor));
                float    scalingFactor = *((float*)&scale);
                uint32_t sum           = 0;

                for(unsigned int i=0;i<nbBeams;i++){
                    sum += htons(((uint16_t*)&(a2->AngleStepArray))[i]);
                    float angle = ( angleFirst + sum * scalingFactor ) * R2D;
                    pings[i].setAcrossTrackAngle(angle);
                }
            }
            else if(sectionName==0x4931){
                //I1
                XtfHeaderQuinsyR2SonicBathy_I1 * i1 = (XtfHeaderQuinsyR2SonicBathy_I1*) (packet + packetIndex);
                uint32_t scale         = htonl( *((uint32_t*)&i1->ScalingFactor));
                float    scalingFactor = *((float*)&scale);

                for(unsigned int i=0;i<nbBeams;i++){
                    double microPascals = htons(((uint16_t*)&(i1->IntensityArray))[i]) * scalingFactor;

                    // dbSPL = 20 * LOG10(uPa * 1000000/0.00002)

                    double intensityDb = (double)20 * log10((double)microPascals * (double)1000000 / (double)0.00002);

                    pings[i].setIntensity( intensityDb );
                }
            }
            else if(sectionName==0x4730){
                //G0
                //XtfHeaderQuinsyR2SonicBathy_G0 * g0 = (XtfHeaderQuinsyR2SonicBathy_G0*) (packet + packetIndex);
                //TODO: process depth gates settings?
            }
            else if(sectionName==0x4731){
                //G1
                //XtfHeaderQuinsyR2SonicBathy_G1 * g1 = (XtfHeaderQuinsyR2SonicBathy_G1*) (packet + packetIndex);
                //TODO: process depth gates settings?
            }
            else if(sectionName==0x5130){
                //TODO: process quality data
                //Q0
                //XtfHeaderQuinsyR2SonicBathy_Q0 * q0 = (XtfHeaderQuinsyR2SonicBathy_Q0*) (packet + packetIndex);
                for(unsigned int i=0;i<nbBeams;i++){
                    uint32_t quality = 0;
                    pings[i].setQuality( quality );
                }
            }
            else if(sectionName==0x5230){
                //R0
                XtfHeaderQuinsyR2SonicBathy_R0 * r0 = (XtfHeaderQuinsyR2SonicBathy_R0*) (packet + packetIndex);
                uint16_t * ranges = &r0->RangeArray;
                uint32_t scalingFactor=htonl(* ((uint32_t *) &r0->ScalingFactor));

                for(unsigned int i=0;i<nbBeams;i++){
                    double twtt = (*((float*) &scalingFactor )) * htons(ranges[i]);
                    pings[i].setTwoWayTravelTime( twtt );
                }
            }
            else{
                printf("Unknown QUINSy R2Sonic section type %.4X\n",sectionName);
            }

            packetIndex += sectionBytes;
        }

        //Process complete pings
        for(auto i=pings.begin();i!=pings.end();i++){

            processor.processPing(
                    (*i).getTimestamp(),
                    (*i).getId(),
                    (*i).getAcrossTrackAngle(),
                    (*i).getAlongTrackAngle(),
                    (*i).getTwoWayTravelTime(),
                    (*i).getQuality(),
                    (*i).getIntensity()
            );

        }
    }
    else{
        printf("Bad QUINSy R2Sonic header\n");
    }
}

#endif
