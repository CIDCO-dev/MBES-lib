#ifndef KONGSBERGTYPES_HPP
#define KONGSBERGTYPES_HPP

#define STX 0x02
#define ETX 0x03

#pragma pack(1)
typedef struct{
    uint32_t            size; //Size is computed starting from STX, so it excludes this one
    unsigned char       stx;
    unsigned char       type;
    uint16_t            modelNumber;
    uint32_t            date;
    uint32_t            time;
    uint16_t            counter;
    uint16_t            serialNumber;
} KongsbergHeader;
#pragma pack()


#pragma pack(1)
typedef struct{
    uint16_t  deltaTime;         //time in milliseconds since record start
    int16_t   sensorStatus;
    int16_t   roll;              //in 0.01 degrees
    int16_t   pitch;             //in 0.01 degrees
    int16_t   heave;             //in cm
    uint16_t  heading;           //in 0.01 degrees
} KongsbergAttitudeEntry;
#pragma pack()

#pragma pack(1)
typedef struct{
    int32_t  lattitude;          // decimal degrees * 20,000,000
    int32_t  longitude;          // decimal degrees * 20,000,000
    uint16_t fixQuality;        // in cm
    uint16_t speedOverGround;   // in cm/s
    uint16_t courseOverGround;  // in 0.01deg
    uint16_t headingOverGround; // in 0.01deg
    uint8_t  positionSystemDescriptor;
    uint8_t  inputDatagramBytes; // Number of bytes in input datagram
    char     inputDatagram[];

    //(...)  input datagram etc. we don't need that much

} KongsbergPositionDatagram;
#pragma pack()

#pragma pack(1)
typedef struct{
    uint32_t            profileDate; //Date = year*10000 + month*100 + day
    uint32_t            profileTime; //Time since midnight in seconds
    uint16_t            nbEntries;
    uint16_t            depthResolution; //in cm
} KongsbergSoundSpeedProfile;
#pragma pack()

#pragma pack(1)
typedef struct{
    uint32_t            depth;
    uint32_t            soundSpeed; //in dm/s
} KongsbergSoundSpeedProfileEntry;
#pragma pack()

#endif // KONGSBERGTYPES_HPP
