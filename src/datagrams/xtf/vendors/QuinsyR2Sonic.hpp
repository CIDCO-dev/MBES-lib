#ifndef XTFTYPESQUINSYR2SONIC_HPP
#define XTFTYPESQUINSYR2SONIC_HPP
/**
 *  Description des datagrammes pour le format XTF
 *  Référence: Triton Imaging, Inc. eXtended Triton Format (XTF) Rev. 40 (9/22/2015)
 *  Auteurs: Guillaume Morissette
 *
 *  Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */


#include <stdint.h>
#include <string>

#pragma pack(1)
typedef struct {
        uint32_t PacketName; // 'BTH0'
        uint32_t PacketSize; // [bytes] size of this entire packet
        uint32_t DataStreamID; // reserved for future use
}XtfHeaderQuinsyR2SonicBathy;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section H0: header
    uint16_t SectionName; // 'H0'
    uint16_t SectionSize; // [bytes] size of this entire section
    uint8_t  ModelNumber[12]; // example "2024", unused chars are nulls
    uint8_t  SerialNumber[12]; // example "100017", unused chars are nulls
    uint32_t TimeSeconds; // [seconds] ping time relative to 0000 hours 1-Jan-1970, integer part
    uint32_t TimeNanoseconds; // [nanoseconds] ping time relative to 0000 hours 1-Jan-1970, fraction part
    uint32_t PingNumber; // pings since power-up or reboot
    float    PingPeriod; // [seconds] time between most recent two pings
    float    SoundSpeed; // [meters per second]
    float    Frequency; // [hertz] sonar center frequency
    float    TxPower; // [dB re 1 uPa at 1 meter]
    float    TxPulseWidth; // [seconds]
    float    TxBeamwidthVert; // [radians]
    float    TxBeamwidthHoriz; // [radians]
    float    TxSteeringVert; // [radians]
    float    TxSteeringHoriz; // [radians]
    uint16_t ProjTemp; // [hundredths of a degree Kelvin] 2026 projector temperature (divide value by 100, subtract 273.15 to get °C)
    int16_t  VTXOffset; // [hundredths of a dB] transmit voltage offset at time of ping (divide value by 100 to get dB)
    float    RxBandwidth; // [hertz]
    float    RxSampleRate; // [hertz] sample rate of data acquisition and signal processing
    float    RxRange; // [meters] sonar range setting
    float    RxGain; // [multiply by two for relative dB]
    float    RxSpreading; // [dB (times log range in meters)]
    float    RxAbsorption; // [dB per kilometer]
    float    RxMountTilt; // [radians]
    uint32_t RxMiscInfo; // reserved for future use
    uint16_t reserved; // reserved for future use
    uint16_t Points; // number of bathy points
}XtfHeaderQuinsyR2SonicBathy_H0;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section R0: 16-bit bathy point ranges
    uint16_t SectionName; // 'R0'
    uint16_t SectionSize; // [bytes] size of this entire section
    float    ScalingFactor;
    uint16_t RangeArray; // [seconds two-way] = R0_Range * R0_ScalingFactor
    //uint16_t R0_unused[H0_Points & 1]; // ensure 32-bit section size
}XtfHeaderQuinsyR2SonicBathy_R0;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section A0: bathy point angles, equally-spaced (present only during "equi-angle" spacing mode)
    uint16_t    SectionName; // 'A0'
    uint16_t    SectionSize; // [bytes] size of this entire section
    float       AngleFirst; // [radians] angle of first (port side) bathy point, relative to array centerline, AngleFirst < AngleLast
    float       AngleLast; // [radians] angle of last (starboard side) bathy point
    float       MoreInfo[6]; // reserved for future use
}XtfHeaderQuinsyR2SonicBathy_A0;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section A2: 16-bit bathy point angles, arbitrarily-spaced (present only during "equi-distant" spacing mode)
    uint16_t SectionName; // 'A2'
    uint16_t SectionSize; // [bytes] size of this entire section
    float    AngleFirst; // [radians] angle of first (port side) bathy point, relative to array centerline, AngleFirst < AngleLast
    float    ScalingFactor;
    float    MoreInfo[6]; // reserved for future use
    uint16_t AngleStepArray; // [radians] angle[n] = A2_AngleFirst + (32-bit sum of A2_AngleStep[0] through A2_AngleStep[n]) * A2_ScalingFactor
    // u16 A2_unused[H0_Points & 1]; // ensure 32-bit section size
}XtfHeaderQuinsyR2SonicBathy_A2;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section I1: 16-bit bathy intensity (present only if enabled)
    uint16_t SectionName; // 'I1'
    uint16_t SectionSize; // [bytes] size of this entire section
    float    ScalingFactor;
    uint16_t IntensityArray; // [micropascals] intensity[n] = I1_Intensity[n]) * I1_ScalingFactor
    // unused[H0_Points & 1]; // ensure 32-bit section size
}XtfHeaderQuinsyR2SonicBathy_I1;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section G0: simple straight-line depth gates
    uint16_t    SectionName; // 'G0'
    uint16_t    SectionSize; // [bytes] size of this entire section
    float       DepthGateMin; // [seconds two-way]
    float       DepthGateMax; // [seconds two-way]
    float       DepthGateSlope; // [radians]
}XtfHeaderQuinsyR2SonicBathy_G0;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section G1: 8-bit gate positions, arbitrary paths (present only during "verbose" gate description mode)
    uint16_t    SectionName; // 'G1'
    uint16_t    SectionSize; // [bytes] size of this entire section
    float       ScalingFactor;
    uint8_t     RangeMinArray; // [seconds two-way] = RangeMin * G1_ScalingFactor
    uint8_t     RangeMaxArray; // [seconds two-way] = RangeMax * G1_ScalingFactor
    // % u16 G1_unused[H0_Points & 1]; // ensure 32-bit section size
}XtfHeaderQuinsyR2SonicBathy_G1;
#pragma pack()

#pragma pack(1)
typedef struct {
    // section Q0: 4-bit quality flags
    uint16_t    SectionName; // 'Q0' quality, 4-bit
    uint16_t    SectionSize; // [bytes] size of this entire section
    uint32_t    QualityArray; //[(H0_Points+7)/8]; // 8 groups of 4 flags bits (phase detect, magnitude detect, reserved, reserved), packed left-to-right
}XtfHeaderQuinsyR2SonicBathy_Q0;
#pragma pack()



#endif
