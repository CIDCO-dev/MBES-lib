/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   S7kTypes.hpp
 * Author: glm,jordan
 *
 * Created on November 1, 2018, 11:59 AM
 */

#ifndef S7KTYPES_HPP
#define S7KTYPES_HPP

#include <stdint.h>
#include <iostream>
#include <iomanip>

#define SYNC_PATTERN 0x0000FFFF

#pragma pack(1)
typedef struct {
    uint16_t Year;
    uint16_t Day;
    float Seconds;
    uint8_t Hours;
    uint8_t Minutes;
} S7kTime;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kTime& time) {
    return os << std::setprecision(15)
            << "Year: " << time.Year << std::endl
            << "Day: " << time.Day << std::endl
            << "Seconds: " << time.Seconds << std::endl
            << "Hours: " << time.Hours << std::endl
            << "Minutes: " << time.Minutes; // no std::endl since it is contained in another record
}

#pragma pack(1)
typedef struct { // pp 13-14
    uint16_t ProtocolVersion;
    uint16_t Offset;
    uint32_t SyncPattern;
    uint32_t Size;
    uint32_t OptionalDataOffset;
    uint32_t OptionalDataIdentifier;
    S7kTime Timestamp;
    uint16_t RecordVersion;
    uint32_t RecordTypeIdentifier;
    uint32_t DeviceIdentifier;
    uint16_t Reserved;
    uint16_t SystemEnumerator;
    uint32_t Reserved2;
    uint16_t Flags;
    uint16_t Reserved3;
    uint32_t Reserved4;
    uint32_t TotalRecordsInFragmentedDataRecordSet; // if appropriate flag is set
    uint32_t FragmentNumber; // if appropriate flag is set
    // does not include variable sized data records
    // does not include u32 checksum after data section
} S7kDataRecordFrame;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kDataRecordFrame& drf) {
    return os << "ProtocolVersion: " << drf.ProtocolVersion << std::endl
            << "Offset: " << drf.Offset << std::endl
            << "SyncPattern: " << drf.SyncPattern << std::endl
            << "Size: " << drf.Size << std::endl
            << "OptionalDataOffset: " << drf.OptionalDataOffset << std::endl
            << "OptionalDataIdentifier: " << drf.OptionalDataIdentifier << std::endl
            << "Timestamp: " << drf.Timestamp << std::endl
            << "RecordVersion: " << drf.RecordVersion << std::endl
            << "RecordTypeIdentifier: " << drf.RecordTypeIdentifier << std::endl
            << "DeviceIdentifier: " << drf.DeviceIdentifier << std::endl
            << "Reserved: " << drf.Reserved << std::endl
            << "SystemEnumerator: " << drf.SystemEnumerator << std::endl
            << "Reserved2: " << drf.Reserved2 << std::endl
            << "Flags: " << drf.Flags << std::endl
            << "Reserved3: " << drf.Reserved3 << std::endl
            << "Reserved4: " << drf.Reserved4 << std::endl
            << "TotalRecordsInFragmentedDataRecordSet: " << drf.TotalRecordsInFragmentedDataRecordSet << std::endl
            << "FragmentNumber: " << drf.FragmentNumber << std::endl;
}

#pragma pack(1)
typedef struct { // pp 102-103
    uint64_t FileIdentifier[2];
    uint16_t VersionNumber;
    uint16_t Reserved;
    uint64_t SessionIdentifier[2];
    uint32_t RecordDataSize;
    uint32_t NumberOfDevices;
    char RecordingName[64];
    char RecordingProgramVersionNumber[16];
    char UserDefinedName[64];
    char Notes[128];
    // does not include variable sized data records
    // does not include variable sized optional records
} S7kFileHeader;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kFileHeader& fh) {
    return os << "FileIdentifier: " << fh.FileIdentifier << std::endl
            << "VersionNumber: " << fh.VersionNumber << std::endl
            << "Reserved: " << fh.Reserved << std::endl
            << "SessionIdentifier: " << fh.SessionIdentifier << std::endl
            << "RecordDataSize: " << fh.RecordDataSize << std::endl
            << "NumberOfDevices: " << fh.NumberOfDevices << std::endl
            << "RecordingName: " << fh.RecordingName << std::endl
            << "RecordingProgramVersionNumber: " << fh.RecordingProgramVersionNumber << std::endl
            << "UserDefinedName: " << fh.UserDefinedName << std::endl
            << "Notes: " << fh.Notes << std::endl;
}

#pragma pack(1)
typedef struct { // p. 103
    uint32_t DeviceIdentifier;
    uint16_t SystemEnumerator;
    // See NumberOfDevices in S7kFileHeader to determine array size
} S7kFileHeaderRecordDatum;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kFileHeaderRecordDatum& fhrd) {
    return os << "DeviceIdentifier: " << fhrd.DeviceIdentifier << std::endl
            << "SystemEnumerator: " << fhrd.SystemEnumerator << std::endl;
}

#pragma pack(1)
typedef struct { // p. 103
    uint32_t Size;
    uint64_t Offset;
} S7kFileHeaderOptionalData;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kFileHeaderOptionalData& fhod) {
    return os << "Size: " << fhod.Size << std::endl
            << "Offset: " << fhod.Offset << std::endl;
}

#pragma pack(1)
typedef struct { // pp 25-26
    uint32_t DatumIdentifier;
    float Latency;
    double LatitudeOrNorthing;
    double LongitudeOrEasting;
    double Height;
    uint8_t PositionTypeFlag; // 0 = geographical ; 1 = grid
    uint8_t UTMZone;
    uint8_t QualityFlag;
    uint8_t PositioningMethod; // see p.26
    uint8_t NumberOfSatellites;
} S7kPosition;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kPosition& pos) {
    return os << "DatumIdentifier: " << pos.DatumIdentifier << std::endl
            << "Latency: " << pos.Latency << std::endl
            << "LatitudeOrNorthing: " << pos.LatitudeOrNorthing << std::endl
            << "LongitudeOrEasting: " << pos.LongitudeOrEasting << std::endl
            << "Height: " << pos.Height << std::endl
            << "PositionTypeFlag: " << pos.PositionTypeFlag << std::endl
            << "UTMZone: " << pos.UTMZone << std::endl
            << "QualityFlag: " << pos.QualityFlag << std::endl
            << "PositioningMethod: " << pos.PositioningMethod << std::endl
            << "Latency: " << pos.Latency << std::endl;
}

#pragma pack(1)
typedef struct { // p. 30
    uint8_t DepthDescriptor;
    uint8_t CorrectionFlag;
    uint16_t Reserved;
    float Depth; // in meters positive towards greater depth
} S7kDepth;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kDepth& depth) {
    return os << "DepthDescriptor: " << depth.DepthDescriptor << std::endl
            << "CorrectionFlag: " << depth.CorrectionFlag << std::endl
            << "Reserved: " << depth.Reserved << std::endl
            << "Depth: " << depth.Depth << std::endl;
}

#pragma pack(1)
typedef struct { // pp 35-36
    uint8_t VerticalReference;
    double Latitude; // radians -west
    double Longitude; // radians
    float HorizontalPositionAccuracy; // meters
    float VesselHeight; // vessel reference point above vertical reference in meters
    float HeightAccuracy; // meters
    float SpeedOverGround; // m/s
    float CourseOverGround; // radians
    float Heading; // radians
} S7kNavigation;
#pragma pack()

std::ostream& operator<<(std::ostream& os, const S7kNavigation& nav) {
    return os << "VerticalReference: " << nav.VerticalReference << std::endl
            << "Latitude: " << nav.Latitude << std::endl
            << "Longitude: " << nav.Longitude << std::endl
            << "HorizontalPositionAccuracy: " << nav.HorizontalPositionAccuracy << std::endl
            << "VesselHeight: " << nav.VesselHeight << std::endl
            << "HeightAccuracy: " << nav.HeightAccuracy << std::endl
            << "SpeedOverGround: " << nav.SpeedOverGround << std::endl
            << "CourseOverGround: " << nav.CourseOverGround << std::endl
            << "Heading: " << nav.Heading << std::endl;
}

#pragma pack(1)
typedef struct { // pp 35-36
    uint8_t NumberOfAttitudeDataSets;
} S7kAttitudeRTH;
#pragma pack()

#pragma pack(1)
typedef struct { // pp 35-36
    uint16_t TimeDifferenceFromRecordTimeStamp; // in milliseconds
    float roll;
    float pitch;
    float heave;
    float heading;
} S7kAttitudeRD;
#pragma pack()




#endif /* S7KTYPES_HPP */

