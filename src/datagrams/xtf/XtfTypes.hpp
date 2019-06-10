/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef XTFTYPES_HPP
#define XTFTYPES_HPP
/**
 *  Description des datagrammes pour le format XTF
 *  Référence: Triton Imaging, Inc. eXtended Triton Format (XTF) Rev. 40 (9/22/2015)
 *  Auteurs: Jérémy Viau Trudel,Guillaume Morissette
 */


#include <stdint.h>
#include <string>
#include "vendors/QuinsyR2Sonic.hpp"

#pragma pack(1)
typedef struct {
	uint8_t TypeOfChannel;
	uint8_t SubChannelNumber;
	uint16_t CorrectionFlags;
	uint16_t UniPolar;
	uint16_t BytesPerSample;
	uint32_t Reserved;
	char ChannelName[16];
	float VoltScale;
	float Frequency;
	float HorizBeamAngle;
	float TiltAngle;
	float BeamWidth;
	float OffsetX;
	float OffsetY;
	float OffsetZ;
	float OffsetYaw;
	float OffsetPitch;
	float OffsetRoll;
	uint16_t BeamsPerArray;
	char ReservedArea2[54];
} XtfChanInfo;
#pragma pack()


#pragma pack(1)
typedef struct{
	uint8_t FileFormat;
	uint8_t SystemType;
	char RecordingProgramName[8];
	char RecordingProgramVersion[8];
	char SonarName[16];
	uint16_t SonarType;

	char NoteString[64];
	char ThisFileName[64];
	uint16_t NavUnits;
	uint16_t NumberOfSonarChannels;
	uint16_t NumberOfBathymetryChannels;
	uint8_t NumberOfSnippetChannels;
	uint8_t NumberOfForwardLookArrays;
	uint16_t NumberOfEchoStrengthChannels;
	uint8_t NumberOfInterferometryChannels;
	uint8_t Reserved1;
	uint16_t Reserved2;
	float ReferencePointHeight;
	// navigation system parameters
	int8_t ProjectionType[12];
	int8_t SpheriodType[10];
	int32_t NavigationLatency;
	float OriginY;
	float OriginX;
	float NavOffsetY;
	float NavOffsetX;
	float NavOffsetZ;
	float NavOffsetYaw;
	float MRUOffsetY;
	float MRUOffsetX;
	float MRUOffsetZ;
	float MRUOffsetYaw;
	float MRUOffsetPitch;
	float MRUOffsetRoll;
	XtfChanInfo Channels[6];
} XtfFileHeader;
#pragma pack()


#pragma pack(1)
typedef struct{
        uint16_t MagicNumber;
        uint8_t HeaderType;
        uint8_t SubChannelNumber;
        uint16_t NumChansToFollow;
        uint16_t Reserved1[2];
        uint32_t NumBytesThisRecord;
} XtfPacketHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t HSeconds;
	uint16_t JulianDay;
	uint32_t EventNumber;
	uint32_t PingNumber;
	float SoundVelocity;
	float OceanTide;
	uint32_t Reserved2;
	float ConductivityFreq;
	float TemperatureFreq;
	float PressureFreq;
	float PressureTemp;
	float Conductivity;
	float WaterTemperature;
	float Pressure;
	float ComputedSoundVelocity;
	float MagX;
	float MagY;
	float MagZ;
	float AuxVal1;
	float AuxVal2;
	float AuxVal3;
	float AuxVal4;
	float AuxVal5;
	float AuxVal6;
	float SpeedLog;
	float Turbidity;
	float ShipSpeed;
	float ShipGyro;
	double ShipYcoordinate;
	double ShipXcoordinate;
	uint16_t ShipAltitude;
	uint16_t ShipDepth;
	uint8_t FixTimeHour;
	uint8_t FixTimeMinute;
	uint8_t FixTimeSecond;
	uint8_t FixTimeHsecond;
	float SensorSpeed;
	float KP;
	double SensorYcoordinate;
	double SensorXcoordinate;
	uint16_t SonarStatus;
	uint16_t RangeToFish;
	uint16_t BearingToFish;
	uint16_t CableOut;
	float Layback;
	float CableTension;
	float SensorDepth;
	float SensorPrimaryAltitude;
	float SensorAuxAltitude;
	float SensorPitch;
	float SensorRoll;
	float SensorHeading;
	float Heave;
	float Yaw;
	uint32_t AttitudeTimeTag;
	float DOT;
	uint32_t NavFixMilliseconds;
	uint8_t ComputerClockHour;
	uint8_t ComputerClockMinute;
	uint8_t ComputerClockSecond;
	uint8_t ComputerClockHsec;
	short FishPositionDeltaX;
	short FishPositionDeltaY;
	unsigned char FishPositionErrorCode;
	unsigned int OptionalOffsey;  // triton 7125 only
	uint8_t CableOutHundredths;
	uint8_t ReservedSpace2[6];
} XtfPingHeader;
#pragma pack()


#pragma pack(1)
typedef struct {
	uint16_t ChannelNumber;
	uint16_t DownsampleMethod;
	float SlantRange;
	float GroundRange;
	float TimeDelay;
	float TimeDuration;
	float SecondsPerPing;
	uint16_t ProcessingFlags;
	uint16_t Frequency;
	uint16_t InitialGainCode;
	uint16_t GainCode;
	uint16_t BandWidth;
	uint32_t ContactNumber;
	uint16_t ContactClassification;
	uint8_t ContactSubNumber;
	uint8_t ContactType;
	uint32_t NumSamples;
	uint16_t MillivoltScale;
	float ContactTimeOffTrack;
	uint8_t ContactCloseNumber;
	uint8_t Reserved2;
	float FixedVSOP;
	int16_t Weight;
	uint8_t ReservedSpace[4];
} XtfPingChanHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t ReservedBytes[35];
	char NotesText[200];
}XtfNotesHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint32_t Reserved2[2];
	uint32_t EpochMicroseconds;
	uint32_t SourceEpoch;
	float Pitch;
	float Roll;
	float Heave;
	float Yaw;
	uint32_t TimeTag;
	float Heading;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minutes;
	uint8_t Seconds;
	uint16_t Milliseconds;
	uint8_t Reserved3[1];
} XtfAttitudeData;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t SerialPort;
	uint16_t NumChansToFollow;
	uint16_t Reserved[2];
	uint32_t NumBytesThisRecord;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t HSeconds;
	uint16_t JulianDay;
	uint32_t TimeTag;
	uint16_t StringSize;
}XtfRawSerialHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t SubChannelNumber;
	uint16_t NumChansToFollow;
	uint16_t Reserved1[2];
	uint32_t NumBytesThisRecord;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t HSeconds;
	uint32_t NumSensorBytes;
	uint32_t RelativeBathyPingNum;
	uint8_t Reserved3[34];
}XtfHighSpeedSensor;
#pragma pack()

#pragma pack(1)
typedef struct {
	double dPosOffsetTrX;
	double dPosOffsetTrY;
	float fDepth;
	double dTime;
	short usAmpl;
	uint8_t ucQuality;
}XtfBeamXYZA;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint32_t Id;
	uint16_t  HeaderSize;
	uint16_t  DataSize;
	uint32_t  PingNumber;
	uint32_t Seconds;
	uint32_t Millisec;
	uint16_t Latency;
	uint16_t SonarID[2];
	uint16_t SonarModel;
	uint16_t Frequency;
	uint16_t SSpeed;
	uint16_t SampleRate;
	uint16_t PingRate;
	uint16_t Range;
	uint16_t Power;
	uint16_t Gain;
	uint16_t  PulseWidth;
	uint16_t Spread;
	uint16_t Absorb;
	uint16_t Proj;
	uint16_t ProjWidth;
	uint16_t SpacingNum;
	uint16_t SpacingDen;
	int16_t ProjAngle;
	uint16_t MinRange;
	uint16_t MaxRange;
	uint16_t MinDepth;
	uint16_t MaxDepth;
	uint16_t Filters;
	uint8_t bFlags[2];
	int16_t HeadTemp;
	uint16_t BeamCnt;
}SNP0;
#pragma pack()


#pragma pack(1)
typedef struct {
	uint32_t ID;
	uint16_t HeaderSize;
	uint16_t DataSize;
	uint32_t PingNumber;
	uint16_t Beam;
	uint16_t SnipSamples;
	uint16_t GainStart;
	uint16_t GainEnd;
	uint16_t FragOffset;
	uint16_t FragSamples;
}SNP1;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minutes;
	uint8_t Seconds;
	uint16_t MicroSeconds; //Hundredths of microseconds (0-9999)
	double RawYcoordinate;
	double RawXcoordinate;
	double RawAltitude;
	float Pitch;
	float Roll;
	float Heave;
	float Heading;
	uint8_t Reserved2;
}XtfPosRawNavigation;
#pragma pack()



#pragma pack(1)
typedef struct {
	uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t SubChannelNumber;
	uint16_t NumChansToFollow;
	uint16_t Reserved1[2];
	uint32_t NumBytesThisRecord;
	uint32_t TimeTag;
	int32_t Id;
	float SoundVelocity;
	float Intensity;
	int32_t Quality;
	float TwoWayTravelTime;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint16_t MilliSeconds;
	uint8_t Reserved[7];
}XtfQpsSingleBeam;
#pragma pack()

#pragma pack(1)
typedef struct {
	int32_t Id;
	float Intensity;
	int32_t Quality;
	float TwoWayTravelTime;
	float DeltaTime;
	float OffsetX;
	float OffsetY;
	float OffsetZ;
	float Reserved[4];
}XtfQpsMultiTxEntry;
#pragma pack()

#pragma pack(1)
typedef struct {
	int32_t Id;
	double Intensity;
	int32_t Quality;
	double TwoWayTravelTime;
	double DeltaTime;
	double BeamAngle;
	double TiltAngle;
	float Reserved[4];
}XtfQpsMbEntry;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t ManufacturerID;
	uint16_t SonarID;
	uint16_t PacketID;
	uint16_t Reserved1[1];
	uint32_t NumBytesThisRecord;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t Hseconds;
	uint16_t JulianDay;
	uint16_t Reserved2[2];
	uint32_t PingNumber;
	uint32_t TimeTag;
	uint32_t NumCustomerB;
	uint8_t Reserved3[24];
}XtfRawCustomHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
	/*uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t Reserved[7];
	uint32_t NumBytesThisRecord;*/
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint32_t Microseconds;
	uint32_t SourceEpoch;
	uint32_t TimeTag;
	double RawYCoordinate;
	double RawXCoordinate;
	double RawAltitude;
	uint8_t TimeFlag;
	uint8_t Reserved1[6];
}XtfHeaderNavigation_type42;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t Reserved[7];
	uint32_t NumBytesThisRecord;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint32_t Microseconds;
	uint32_t SourceEpoch;
	uint32_t TimeTag;
	float Gyro;
	uint8_t TimeFlag;
	uint8_t Reserved1[26];
}XtfHeaderNavigation_type84;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint16_t MagicNumber;
	uint8_t HeaderType;
	uint8_t Reserved [7];
	uint32_t NumBytesThisRecord;
	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint32_t Microseconds;
	uint32_t SourceEpoch;
	uint32_t TimeTag;
	float Gyro;
	uint8_t TimeFlag;
	uint8_t Reserved1[26];
}XtfHeaderGyro;
#pragma pack()


#define XTF_HEADER_ATTITUDE 3
#define XTF_HEADER_Q_MULTIBEAM 28
#define XTF_HEADER_POS_RAW_NAVIGATION 42
#define XTF_HEADER_POSITION 107
#define XTF_HEADER_QUINSY_R2SONIC_BATHY 65


const std::string SonarTypes[]{
	"NONE , default",
	"JAMSTEC, Jamstec chirp 2-channel subbottom",
	"ANALOG_C31, PC31 8-channel",
	"SIS1000, Chirp SIS-1000 sonar",
	"ANALOG_32CHAN, Spectrum with 32-channel DSPlink card",
	"KLEIN2000, Klein system 2000 with digital interface",
	"RWS, Standard PC31 analog with special nav code",
	"DF1000, EG&G DF1000 digitalinterface",
	"SEABAT, Reson SEABAT 900x analog/serial",
	"KLEIN595, 4-chan Klein 595, same as ANALOG_C31",
	"EGG260, 2-channel EGG260, same as ANALOG_C31",
	"SONATECH_DDS, Sonatech Diver Detection System on Spectrum DSP32C",
	"ECHOSCAN, Odom EchoScanII multibeam (with simultaneous analog sidescan)",
	"ELAC, Elac multibeam system",
	"KLEIN5000, Klein system 5000 with digital interface",
	"Reson Seabat 8101",
	"Imagenex model 858",
	"USN SILOS with 3-channel analog",
	"Sonatech Super-high res sidescan sonar",
	"Delph AU32 Analog input (2 channel)",
	"Generic sonar using the memory-mapped file interface",
	"Simrad SM200",
	"0 Multibeam Echo Sounder",
	"Standard multimedia audio",
	"Edgetech (EG&G) ACI card for 260 sonar through PC31 card",
	"Edgetech Black Box",
	"Fugro deeptow",
	"C&C's Edgetech Chirp conversion program",
	"DTI SAS Synthetic Aperture processor (memmap file)",
	"Fugro's Osiris AUV Sidescan data",
	"Fugro's Osiris AUV Multibeam data",
	"Geoacoustics SLS",
	"Simrad EM2000/EM3000",
	"Klein system 3000",
	"SHRSSS Chirp system",
	"Benthos C3D SARA/CAATI",
	"Edgetech MP-X",
	"CMAX",
	"Benthos sis1624",
	"Edgetech 4200",
	"Benthos SIS1500",
	"Benthos SIS1502",
	"Benthos SIS3000",
	"Benthos SIS7000",
	"DF1000 DCU",
	"NONE_SIDESCAN",
	"NONE_MULTIBEAM",
	"Reson 7125",
	"CODA Echoscope",
	"Kongsberg SAS",
	"QINSy",
	"GeoAcoustics DSSS",
	"CMAX_USB",
	"SwathPlus Bathy",
	"R2Sonic QINSy",
	"R2Sonic Triton",
	"Converted SwathPlus Bathy",
	"Edgetech 4600",
	"Klein 3500",
	"Klein 5900",
	"EM2040",
	"Klein5Kv2",
	"DT100",
	"Kraken",
	"Kraken",
	"Klein 4900",
	"FSI HMS622",
	"SI HMS6x4",
	"FSI HMS6x5"
};

#endif
