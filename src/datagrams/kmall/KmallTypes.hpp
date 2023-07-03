/*h+*/
 #ifndef _EMDGMFORMAT_H
 #define _EMDGMFORMAT_H
  
 #include <stdint.h>
  
 #ifndef _VXW
 #pragma pack(4)
 #endif
  
 /*
  
   Revision History:
  
   01  01 SEP 2016  Rev A.
   02  01 MAR 2017  Rev B.
   03  05 JUL 2017  Rev C.
   04  08 DES 2017  Rev D.
   05  25 MAY 2018  Rev E.
   06  16 NOV 2018  Rev F.
   07  01 NOV 2019  Rev G.
   08  20 MAR 2020  Rev H.
   09  04 JUN 2021  Rev I.
  */
  
 #define EM_DGM_FORMAT_VERSION "Rev I 2021-06-04" 
 #define MAX_NUM_BEAMS 1024 
 #define MAX_EXTRA_DET 1024 
 #define MAX_EXTRA_DET_CLASSES 11   
 #define MAX_SIDESCAN_SAMP 60000   
 #define MAX_SIDESCAN_EXTRA_SAMP 15000   
 #define MAX_NUM_TX_PULSES 9  
 #define MAX_ATT_SAMPLES  148 
 #define MAX_SVP_POINTS 2000  
 #define MAX_SVT_SAMPLES 1  
 #define MAX_DGM_SIZE 64000  
 #define MAX_NUM_MST_DGMS 256 
 #define MAX_NUM_MWC_DGMS 256 
 #define MAX_NUM_MRZ_DGMS 32  
 #define MAX_NUM_FCF_DGMS 1   
 #define MAX_SPO_DATALENGTH 250  
 #define MAX_ATT_DATALENGTH 250  
 #define MAX_SVT_DATALENGTH 64   
 #define MAX_SCL_DATALENGTH 64   
 #define MAX_SDE_DATALENGTH 32   
 #define MAX_SHI_DATALENGTH 32   
 #define MAX_CPO_DATALENGTH 250  
 #define MAX_CHE_DATALENGTH 64   
 #define MAX_F_FILENAME_LENGTH 64 
 #define MAX_F_FILE_SIZE 63000    
 #define UNAVAILABLE_POSFIX 0xffff 
 #define UNAVAILABLE_LATITUDE 200.0f 
 #define UNAVAILABLE_LONGITUDE 200.0f 
 #define UNAVAILABLE_SPEED -1.0f 
 #define UNAVAILABLE_COURSE -4.0f 
 #define UNAVAILABLE_ELLIPSOIDHEIGHT -999.0f 
 /*********************************************
             Datagram names
  *********************************************/
  
 /* I-datagrams */
 #define EM_DGM_I_INSTALLATION_PARAM        "#IIP"
 #define EM_DGM_I_OP_RUNTIME                "#IOP"
  
 /* S-datagrams */
 #define EM_DGM_S_POSITION                  "#SPO"
 #define EM_DGM_S_KM_BINARY                 "#SKM"
 #define EM_DGM_S_SOUND_VELOCITY_PROFILE    "#SVP"
 #define EM_DGM_S_SOUND_VELOCITY_TRANSDUCER "#SVT"
 #define EM_DGM_S_CLOCK                     "#SCL"
 #define EM_DGM_S_DEPTH                     "#SDE"
 #define EM_DGM_S_HEIGHT                    "#SHI"
  
 /* M-datagrams */
 #define EM_DGM_M_RANGE_AND_DEPTH           "#MRZ"
 #define EM_DGM_M_WATER_COLUMN              "#MWC"
  
 /* C-datagrams */
 #define EM_DGM_C_POSITION                  "#CPO"
 #define EM_DGM_C_HEAVE                     "#CHE"
  
 /* F-datagrams */
 #define EM_DGM_F_CALIBRATION_FILE          "#FCF"
  
 /*********************************************
  
    General datagram header
  
  *********************************************/
  
 struct EMdgmHeader_def
 {
         uint32_t numBytesDgm;           
         uint8_t dgmType[4];                     
         uint8_t dgmVersion;         
         uint8_t systemID;                       
         uint16_t echoSounderID;         
         uint32_t time_sec;          
         uint32_t time_nanosec;      
 };
  
 typedef struct EMdgmHeader_def EMdgmHeader, *pEMdgmHeader;
  
 /*********************************************
  
    Sensor datagrams
  
  *********************************************/
  
 struct EMdgmScommon_def
 {
         uint16_t numBytesCmnPart;  
         uint16_t sensorSystem;     
         uint16_t sensorStatus;     
         uint16_t padding;
 };
  
 typedef struct EMdgmScommon_def EMdgmScommon, *pEMdgmScommon;
  
 struct EMdgmSdataInfo_def
 {
         uint16_t numBytesInfoPart;  
         uint16_t numSamplesArray;  
         uint16_t numBytesPerSample;  
         uint16_t numBytesRawSensorData; 
 };
  
 typedef struct EMdgmSdataInfo_def EMdgmSdataInfo, *pEMdgmSdataInfo;
  
  
 /************************************
    #SPO - Sensor Position data
  ************************************/
 struct EMdgmSPOdataBlock_def
 {
         uint32_t timeFromSensor_sec;            
         uint32_t timeFromSensor_nanosec;            
         float posFixQuality_m;  
         double correctedLat_deg;   
         double correctedLong_deg;   
         float speedOverGround_mPerSec;  
         float courseOverGround_deg;   
         float ellipsoidHeightReRefPoint_m;  
         int8_t posDataFromSensor[MAX_SPO_DATALENGTH]; 
 };
  
 typedef struct EMdgmSPOdataBlock_def EMdgmSPOdataBlock, *pEMdgmSPOdataBlock;
  
 struct EMdgmSPO_def
 {
         struct EMdgmHeader_def header;
         struct EMdgmScommon_def cmnPart;
         struct EMdgmSPOdataBlock_def sensorData;
 };
  
 #define SPO_VERSION 0
 typedef struct EMdgmSPO_def EMdgmSPO, *pEMdgmSPO;
  
  
 /************************************
    #SKM - KM binary sensor data
  ************************************/
 struct EMdgmSKMinfo_def
 {
         uint16_t numBytesInfoPart;  
         uint8_t sensorSystem;     
         uint8_t sensorStatus;     
         uint16_t sensorInputFormat;  
         uint16_t numSamplesArray;  
         uint16_t numBytesPerSample;  
         uint16_t sensorDataContents; 
 };
  
 typedef struct EMdgmSKMinfo_def EMdgmSKMinfo, *pEMdgmSKMinfo;
 struct KMbinary_def
 {
         uint8_t dgmType[4];                     
         uint16_t numBytesDgm;           
         uint16_t dgmVersion;         
         uint32_t time_sec;       
         uint32_t time_nanosec;   
         uint32_t status;   
         double latitude_deg;   
         double longitude_deg;  
         float ellipsoidHeight_m;  
         float roll_deg;     
         float pitch_deg;    
         float heading_deg;  
         float heave_m;  
         float rollRate;  
         float pitchRate;  
         float yawRate;  
         float velNorth; 
         float velEast;  
         float velDown;  
         float latitudeError_m;   
         float longitudeError_m;   
         float ellipsoidHeightError_m;   
         float rollError_deg;     
         float pitchError_deg;   
         float headingError_deg;  
         float heaveError_m;   
         float northAcceleration;   
         float eastAcceleration;    
         float downAcceleration;    
 };
  
 typedef struct KMbinary_def KMbinary, *pKMbinary;
  
 struct KMdelayedHeave_def
 {
         uint32_t time_sec;
         uint32_t time_nanosec;
         float delayedHeave_m;  
 };
 typedef struct KMdelayedHeave_def KMdelayedHeave, *pKMdelayedHeave;
  
 struct EMdgmSKMsample_def
 {
         struct KMbinary_def KMdefault;
         struct KMdelayedHeave_def delayedHeave;
 };
  
 typedef struct EMdgmSKMsample_def EMdgmSKMsample, *pEMdgmSKMsample;
  
 struct EMdgmSKM_def
 {
         struct EMdgmHeader_def header;
         struct EMdgmSKMinfo_def infoPart;
         struct EMdgmSKMsample_def sample[MAX_ATT_SAMPLES];
 };
  
 #define SKM_VERSION 1
 typedef struct EMdgmSKM_def EMdgmSKM, *pEMdgmSKM;
  
  
 /************************************
     #SVP - Sound Velocity Profile
  ************************************/
 struct EMdgmSVPpoint_def
 {
         float depth_m;  
         float soundVelocity_mPerSec;  
         uint32_t padding;  
         float temp_C;     
         float salinity; 
 };
  
 typedef struct EMdgmSVPpoint_def EMdgmSVPpoint, *pEMdgmSVPpoint;
  
 struct EMdgmSVP_def
 {
         struct EMdgmHeader_def header;
         uint16_t numBytesCmnPart;  
         uint16_t numSamples;  
         uint8_t sensorFormat[4];      
         uint32_t time_sec;     
         double latitude_deg;  
         double longitude_deg; 
         struct EMdgmSVPpoint_def sensorData[MAX_SVP_POINTS];  
 };
  
 #define SVP_VERSION 1
 typedef struct EMdgmSVP_def EMdgmSVP, *pEMdgmSVP;
  
 /************************************
 * #SVT - Sensor sound Velocity measured at Transducer
 ************************************/
 struct EMdgmSVTinfo_def
 {
     uint16_t numBytesInfoPart;  
     uint16_t sensorStatus;     
     uint16_t sensorInputFormat;  
     uint16_t numSamplesArray;  
     uint16_t numBytesPerSample;  
     uint16_t sensorDataContents; 
     float filterTime_sec; 
     float soundVelocity_mPerSec_offset; 
 };
  
 struct EMdgmSVTsample_def
 {
     uint32_t time_sec;           
     uint32_t time_nanosec;       
     float soundVelocity_mPerSec; 
     float temp_C;   
     float pressure_Pa;  
     float salinity; 
 };
  
 typedef struct EMdgmSVTsample_def EMdgmSVTsample, *pEMdgmSVTsample;
  
 struct EMdgmSVT_def
 {
     struct EMdgmHeader_def header;
     struct EMdgmSVTinfo_def infoPart;
     struct EMdgmSVTsample_def sensorData[MAX_SVT_SAMPLES];
 };
  
 #define SVT_VERSION 0
 typedef struct EMdgmSVT_def EMdgmSVT, *pEMdgmSVT;
  
 /************************************
     #SCL - Sensor CLock datagram
  ************************************/
 struct EMdgmSCLdataFromSensor_def
 {
         float offset_sec;  
         int32_t clockDevPU_microsec;    
         uint8_t dataFromSensor[MAX_SCL_DATALENGTH];   
 };
  
 typedef struct EMdgmSCLdataFromSensor_def EMdgmSCLdataFromSensor, *pEMdgmSCLdataFromSensor;
  
 struct EMdgmSCL_def
 {
         struct EMdgmHeader_def header;
         struct EMdgmScommon_def cmnPart;
         struct EMdgmSCLdataFromSensor_def sensData;
 };
  
 #define SCL_VERSION 0
 typedef struct EMdgmSCL_def EMdgmSCL, *pEMdgmSCL;
  
  
 /************************************
     #SDE - Sensor Depth data
  ************************************/
 struct EMdgmSDEdataFromSensor_def
 {
         float depthUsed_m;  
     float depthRaw_m;   
         float offset;  
         float scale;  
         double latitude_deg;  
         double longitude_deg; 
         uint8_t dataFromSensor[MAX_SDE_DATALENGTH];  
 };
  
 typedef struct EMdgmSDEdataFromSensor_def EMdgmSDEdataFromSensor, *pEMdgmSDEdataFromSensor;
  
 struct EMdgmSDE_def
 {
         struct EMdgmHeader_def header;
         struct EMdgmScommon_def cmnPart;
         struct EMdgmSDEdataFromSensor_def sensorData;
 };
  
 #define SDE_VERSION 1
 typedef struct EMdgmSDE_def EMdgmSDE, *pEMdgmSDE;
  
 /************************************
     #SHI - Sensor Height data
  ************************************/
 struct EMdgmSHIdataFromSensor_def
 {
         uint16_t sensorType;  
         float heigthUsed_m;  
         uint8_t dataFromSensor[MAX_SHI_DATALENGTH];  
 };
  
 typedef struct EMdgmSHIdataFromSensor_def EMdgmSHIdataFromSensor, *pEMdgmSHIdataFromSensor;
  
 struct EMdgmSHI_def
 {
         struct EMdgmHeader_def  header;
         struct EMdgmScommon_def cmnPart;
         struct EMdgmSHIdataFromSensor_def sensData;
 };
  
 #define SHI_VERSION 0
 typedef struct EMdgmSHI_def EMdgmSHI, *pEMdgmSHI;
  
  
 /*********************************************
  
    Multibeam datagrams
  
  *********************************************/
 struct EMdgmMpartition_def
 {
         uint16_t numOfDgms;   
         uint16_t dgmNum;      
 };
  
 typedef struct EMdgmMpartition_def EMdgmMpartition, *pEMdgmMpartition;
  
 struct EMdgmMbody_def
 {
         uint16_t numBytesCmnPart;    
         uint16_t pingCnt;  
         uint8_t rxFansPerPing;    
         uint8_t rxFanIndex;       
         uint8_t swathsPerPing;     
         uint8_t swathAlongPosition;  
         uint8_t txTransducerInd;   
         uint8_t rxTransducerInd;   
         uint8_t numRxTransducers;  
         uint8_t algorithmType;  
 };
  
 typedef struct EMdgmMbody_def EMdgmMbody, *pEMdgmMbody;
  
 /************************************
     #MRZ - multibeam data for raw range,
     depth, reflectivity, seabed image(SI) etc.
  ************************************/
 struct EMdgmMRZ_pingInfo_def
 {
         uint16_t numBytesInfoData;   
         uint16_t padding0;   
         float pingRate_Hz;  
         uint8_t beamSpacing;  
         uint8_t depthMode;   
         uint8_t subDepthMode;  
         uint8_t distanceBtwSwath; 
         uint8_t detectionMode;   
         uint8_t pulseForm; 
         uint16_t padding1;
  
         float frequencyMode_Hz; 
         float freqRangeLowLim_Hz;   
         float freqRangeHighLim_Hz; 
         float maxTotalTxPulseLength_sec; 
         float maxEffTxPulseLength_sec; 
         float maxEffTxBandWidth_Hz; 
         float absCoeff_dBPerkm;  
         float portSectorEdge_deg;  
         float starbSectorEdge_deg; 
         float portMeanCov_deg;  
         float starbMeanCov_deg; 
         int16_t portMeanCov_m;  
         int16_t starbMeanCov_m; 
         uint8_t modeAndStabilisation; 
         uint8_t runtimeFilter1;  
         uint16_t runtimeFilter2; 
         uint32_t pipeTrackingStatus;  
         float transmitArraySizeUsed_deg; 
         float receiveArraySizeUsed_deg;  
         float transmitPower_dB; 
         uint16_t SLrampUpTimeRemaining; 
         uint16_t padding2;  
         float yawAngle_deg; 
         uint16_t numTxSectors;  
         uint16_t numBytesPerTxSector;  
         float headingVessel_deg;  
         float soundSpeedAtTxDepth_mPerSec; 
         float txTransducerDepth_m; 
         float z_waterLevelReRefPoint_m; 
         float x_kmallToall_m;   
         float y_kmallToall_m;   
         uint8_t latLongInfo; 
         uint8_t posSensorStatus; 
         uint8_t attitudeSensorStatus; 
         uint8_t padding3;  
         double latitude_deg; 
         double longitude_deg; 
         float ellipsoidHeightReRefPoint_m; 
		 float bsCorrectionOffset_dB; 
		 uint8_t lambertsLawApplied; 
		 uint8_t iceWindow; 
         uint16_t activeModes; 
 };
  
 typedef struct EMdgmMRZ_pingInfo_def EMdgmMRZ_pingInfo, *pEMdgmMRZ_pingInfo;
  
 struct EMdgmMRZ_txSectorInfo_def
 {
         uint8_t txSectorNumb;  
         uint8_t txArrNumber;  
         uint8_t txSubArray;  
         uint8_t padding0;    
         float sectorTransmitDelay_sec;  
         float tiltAngleReTx_deg; 
         float txNominalSourceLevel_dB;  
         float txFocusRange_m; 
         float centreFreq_Hz;  
         float signalBandWidth_Hz;     
         float totalSignalLength_sec;    
         uint8_t pulseShading;   
         uint8_t signalWaveForm; 
         uint16_t padding1;      
         float highVoltageLevel_dB; 
         float sectorTrackingCorr_dB; 
         float effectiveSignalLength_sec; 
 };
  
 typedef struct EMdgmMRZ_txSectorInfo_def EMdgmMRZ_txSectorInfo, *pEMdgmMRZ_txSectorInfo;
  
 struct EMdgmMRZ_rxInfo_def
 {
         uint16_t numBytesRxInfo;    
         uint16_t numSoundingsMaxMain; 
         uint16_t numSoundingsValidMain;  
         uint16_t numBytesPerSounding; 
         float WCSampleRate;   
         float seabedImageSampleRate; 
         float BSnormal_dB;  
         float BSoblique_dB; 
         uint16_t extraDetectionAlarmFlag;  
         uint16_t numExtraDetections; 
         uint16_t numExtraDetectionClasses; 
         uint16_t numBytesPerClass;  
 };
  
 typedef struct EMdgmMRZ_rxInfo_def EMdgmMRZ_rxInfo, *pEMdgmMRZ_rxInfo;
  
 struct EMdgmMRZ_extraDetClassInfo_def
 {
         uint16_t numExtraDetInClass;  
         int8_t padding;  
         uint8_t alarmFlag;  
 };
  
 typedef struct EMdgmMRZ_extraDetClassInfo_def EMdgmMRZ_extraDetClassInfo, *pEMdgmMRZ_extraDetClassInfo;
  
 struct EMdgmMRZ_sounding_def
 {
  
         uint16_t soundingIndex; 
         uint8_t txSectorNumb;  
         uint8_t detectionType;   
         uint8_t detectionMethod; 
         uint8_t rejectionInfo1;  
         uint8_t rejectionInfo2;  
         uint8_t postProcessingInfo;  
         uint8_t detectionClass; 
         uint8_t detectionConfidenceLevel;  
         uint16_t padding; 
         float rangeFactor; 
         float qualityFactor;  
         float detectionUncertaintyVer_m;  
         float detectionUncertaintyHor_m;  
         float detectionWindowLength_sec;  
         float echoLength_sec; 
         uint16_t WCBeamNumb;       
         uint16_t WCrange_samples;  
         float WCNomBeamAngleAcross_deg; 
         float meanAbsCoeff_dBPerkm; 
         float reflectivity1_dB;  
         float reflectivity2_dB;  
         float receiverSensitivityApplied_dB; 
         float sourceLevelApplied_dB; 
         float BScalibration_dB; 
         float TVG_dB; 
         float beamAngleReRx_deg;  
         float beamAngleCorrection_deg;  
         float twoWayTravelTime_sec;  
         float twoWayTravelTimeCorrection_sec; 
         float deltaLatitude_deg;   
         float deltaLongitude_deg;  
         float z_reRefPoint_m; 
         float y_reRefPoint_m; 
         float x_reRefPoint_m; 
         float beamIncAngleAdj_deg; 
         uint16_t realTimeCleanInfo;     
         uint16_t SIstartRange_samples; 
         uint16_t SIcentreSample;  
         uint16_t SInumSamples;    
 };
  
 typedef struct EMdgmMRZ_sounding_def EMdgmMRZ_sounding, *pEMdgmMRZ_sounding;
  
 struct EMdgmMRZ_extraSI_def
 {
  
         uint16_t portStartRange_samples;  
         uint16_t numPortSamples;           
         int16_t portSIsample_desidB[MAX_SIDESCAN_EXTRA_SAMP]; 
         uint16_t starbStartRange_samples; 
         uint16_t numStarbSamples;   
         int16_t starbSIsample_desidB[MAX_SIDESCAN_EXTRA_SAMP]; 
 };
  
 typedef struct EMdgmMRZ_extraSI_def EMdgmMRZ_extraSI, *pEMdgmMRZ_extraSI;
  
 struct EMdgmMRZ_def
 {
         struct EMdgmHeader_def header;
         struct EMdgmMpartition_def partition;
         struct EMdgmMbody_def cmnPart;
         struct EMdgmMRZ_pingInfo_def pingInfo;
         struct EMdgmMRZ_txSectorInfo_def sectorInfo[MAX_NUM_TX_PULSES];
         struct EMdgmMRZ_rxInfo_def rxInfo;
         struct EMdgmMRZ_extraDetClassInfo_def extraDetClassInfo[MAX_EXTRA_DET_CLASSES];
         struct EMdgmMRZ_sounding_def sounding[MAX_NUM_BEAMS+MAX_EXTRA_DET];
         int16_t SIsample_desidB[MAX_SIDESCAN_SAMP];               
 };
  
 #define MRZ_VERSION 3
 typedef struct EMdgmMRZ_def EMdgmMRZ, *pEMdgmMRZ;
  
 /************************************
     #MWC - water column datagram
  ************************************/
 struct EMdgmMWCtxInfo_def
 {
         uint16_t numBytesTxInfo;  
         uint16_t numTxSectors;  
         uint16_t numBytesPerTxSector;   
         int16_t padding;  
         float heave_m; 
 };
  
 typedef struct EMdgmMWCtxInfo_def EMdgmMWCtxInfo, *pEMdgmMWCtxInfo;
  
 struct EMdgmMWCtxSectorData_def
 {
         float tiltAngleReTx_deg;  
         float centreFreq_Hz;        
         float txBeamWidthAlong_deg; 
         uint16_t txSectorNum;    
         int16_t padding;  
 };
  
 typedef struct EMdgmMWCtxSectorData_def EMdgmMWCtxSectorData, *pEMdgmMWCtxSectorData;
  
 struct EMdgmMWCrxInfo_def
 {
         uint16_t numBytesRxInfo;  
         uint16_t numBeams;  
         uint8_t numBytesPerBeamEntry;      
         uint8_t phaseFlag;  
         uint8_t TVGfunctionApplied;  
         int8_t TVGoffset_dB; 
         float sampleFreq_Hz;   
         float soundVelocity_mPerSec;  
 };
  
 typedef struct EMdgmMWCrxInfo_def EMdgmMWCrxInfo, *pEMdgmMWCrxInfo;
  
 struct EMdgmMWCrxBeamData_def
 {
         float beamPointAngReVertical_deg;
         uint16_t startRangeSampleNum;
         uint16_t detectedRangeInSamples;    
         uint16_t beamTxSectorNum;
         uint16_t numSampleData;  
         float detectedRangeInSamplesHighResolution;    
         int8_t  *sampleAmplitude05dB_p;  
 };
  
 typedef struct EMdgmMWCrxBeamData_def EMdgmMWCrxBeamData, *pEMdgmMWCrxBeamData;
  
 struct EMdgmMWCrxBeamPhase1_def
 {
         int8_t rxBeamPhase; 
 };
  
 typedef struct EMdgmMWCrxBeamPhase1_def EMdgmMWCrxBeamPhase1, *pEMdgmMWCrxBeamPhase1;
  
 struct EMdgmMWCrxBeamPhase2_def
 {
         int16_t rxBeamPhase; 
 };
  
 typedef struct EMdgmMWCrxBeamPhase2_def EMdgmMWCrxBeamPhase2, *pEMdgmMWCrxBeamPhase2;
  
 struct EMdgmMWC_def
 {
         struct EMdgmHeader_def header;
         struct EMdgmMpartition_def partition;
         struct EMdgmMbody_def cmnPart;
         struct EMdgmMWCtxInfo_def txInfo;
         struct EMdgmMWCtxSectorData_def sectorData[MAX_NUM_TX_PULSES];
         struct EMdgmMWCrxInfo_def rxInfo;
         struct EMdgmMWCrxBeamData_def *beamData_p; 
 };
  
 #define MWC_VERSION 2
 typedef struct EMdgmMWC_def EMdgmMWC, *pEMdgmMWC;
  
  
 /*********************************************
  
    Compatibility datagrams for .all to .kmall conversion support
  
  *********************************************/
  
 /************************************
    #CPO - Compatibility position sensor data
  ************************************/
 struct EMdgmCPOdataBlock_def
 {
         uint32_t timeFromSensor_sec;            
         uint32_t timeFromSensor_nanosec;            
     float posFixQuality_m;  
     double correctedLat_deg;   
     double correctedLong_deg;   
     float speedOverGround_mPerSec;  
     float courseOverGround_deg;   
     float ellipsoidHeightReRefPoint_m;  
     int8_t posDataFromSensor[MAX_CPO_DATALENGTH]; 
 };
  
 typedef struct EMdgmCPOdataBlock_def EMdgmCPOdataBlock, *pEMdgmCPOdataBlock;
  
 struct EMdgmCPO_def
 {
     struct EMdgmHeader_def header;
     struct EMdgmScommon_def cmnPart;
     struct EMdgmCPOdataBlock_def sensorData;
 };
  
 #define CPO_VERSION 0
 typedef struct EMdgmCPO_def EMdgmCPO, *pEMdgmCPO;
  
  
 /************************************
     #CHE - Compatibility heave data
  ************************************/
 struct EMdgmCHEdata_def
 {
     float heave_m;  
 };
  
 typedef struct EMdgmCHEdata_def EMdgmCHEdata, *pEMdgmCHEdata;
  
 struct EMdgmCHE_def
 {
     struct EMdgmHeader_def header;
     struct EMdgmMbody_def cmnPart;
     struct EMdgmCHEdata_def data;
 };
  
 #define CHE_VERSION 0
 typedef struct EMdgmCHE_def EMdgmCHE, *pEMdgmCHE;
  
  
 /*********************************************
  
    File datagrams
  
  *********************************************/
  
 struct EMdgmFcommon_def
 {
         uint16_t numBytesCmnPart; 
         int8_t fileStatus; 
         uint8_t padding1; 
     uint32_t numBytesFile; 
         uint8_t fileName[MAX_F_FILENAME_LENGTH]; 
 };
  
 typedef struct EMdgmFcommon_def EMdgmFcommon, *pEMdgmFcommon;
  
 /********************************************
     #FCF - Backscatter calibration file datagram
  ********************************************/
 struct EMdgmFCF_def
 {
         struct EMdgmHeader_def header;
     struct EMdgmMpartition_def partition;
         struct EMdgmFcommon_def cmnPart;
         uint8_t bsCalibrationFile[MAX_F_FILE_SIZE]; 
 };
  
 typedef struct EMdgmFCF_def EMdgmFCF, *pEMdgmFCF;
 #define FCF_VERSION 0
  
  
 /*********************************************
  
    Installation and runtime datagrams
  
  *********************************************/
  
 /************************************
     #IIP - Info Installation PU
  ************************************/
 struct EMdgmIIP_def
 {
         EMdgmHeader header;
         uint16_t numBytesCmnPart;  
         uint16_t info;                          
         uint16_t status;                                
         uint8_t install_txt;            
 };
  
 #define IIP_VERSION 0
 typedef struct EMdgmIIP_def dgm_IIP, *pdgm_IIP;
  
  
 /************************************
     #IOP -  Runtime datagram
  ************************************/
 struct EMdgmIOP_def
 {
         EMdgmHeader header;
         uint16_t numBytesCmnPart;  
         uint16_t info;                          
         uint16_t status;                                
         uint8_t runtime_txt;            
 };
  
 #define IOP_VERSION 0
 typedef struct EMdgmIOP_def dgm_IOP, *pdgm_IOP;
  
  
 /************************************
     #IB - BIST Error Datagrams
  ************************************/
 struct EMdgmIB_def
 {
         EMdgmHeader header;
         uint16_t numBytesCmnPart;  
         uint8_t BISTInfo;         
         uint8_t BISTStyle;        
         uint8_t BISTNumber;       
         int8_t  BISTStatus;       
         uint8_t BISTText;         
 };
  
 #define BIST_VERSION 0
 typedef struct EMdgmIB_def dgm_IB, *pdgm_IB;
  
 #ifndef _VXW
 #pragma pack()
 #endif
 #endif


