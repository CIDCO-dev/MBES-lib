/*
* KONGSBERG retain full title to and ownership to the Licensed Software, 
* including all copies, derivatives or future versions thereof, and all 
* rights therein. Notwithstanding the foregoing, Customer may modify the 
* Licensed Software to the extent necessary in connection with Customer’s 
* creation of applications for the Licensed Software, however, provided 
* that such permitted modifications shall be for Customer’s internal use 
* only. Customer shall not sell, distribute or otherwise provide access to 
* such modifications to any third party, without the consent from KONGSBERG. 
* 
* Copyright (c) 2018 KONGSBERG - All rights reserved.
*/

//h+/////////////////////////////////////////////////////////////////////////////////////
/*!
// \file    KMALL_reader.cpp
//
// \brief   Demonstrate how to read a .kmall-file
*/

/*!
// \par     Copyright 2018 Kongsberg Maritime AS
//
// \author  Terje Haga Pedersen
//
// \date    27 APR 2018
//
//h-/////////////////////////////////////////////////////////////////////////////////////
*/

/////////////////////////////////////////////////////////////////////////////////////////
//
// Revision History:
//
// 00  27 APR 2018  THP  Original Version
// 01  19 OCT 2018 jornh Updated with helper functions
//
/////////////////////////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
#define _MDP_STANDALONE
#define _DATUM_DEFINED
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sys/stat.h>
using namespace std;

#include "EMdgmFormat.h"


//f+/////////////////////////////////////////////////////////////////////////
//
//  getMRZPingInfo
/*!
//  Get start ptr of MRZ ping info.
//
//  \param  void
//
//  \return Pointer to the start of rx info.
//
//  \author Ole-Jacob Enderud Jensen
//
//  \date   12.10.2018
*/
//f-/////////////////////////////////////////////////////////////////////////

char * getMRZPingInfo(void * tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      pData = pPI;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

//f+/////////////////////////////////////////////////////////////////////////
//
//  getMRZSectorInfo
/*!
//  Get start ptr of MRZ rx info.
//
//  \param  void
//
//  \return Pointer to the start of rx info.
//
//  \author Ole-Jacob Enderud Jensen
//
//  \date   12.10.2018
*/
//f-/////////////////////////////////////////////////////////////////////////

char * getMRZSectorInfo(void * tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      //Move to end of pingInfo
      char* pch = pPI + pPingInfo->numBytesInfoData; //At start of txsectors
      int nSkip = 0;
      pch += nSkip;

      //Get sector
      int nBytesTx = pPingInfo->numBytesPerTxSector;
      pEMdgmMRZ_txSectorInfo pSect = (pEMdgmMRZ_txSectorInfo)pch;

      pData = pch;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

//f+/////////////////////////////////////////////////////////////////////////
//
//  getMRZRxInfo
/*!
//  Get start ptr of MRZ rx info.
//
//  \param  void
//
//  \return Pointer to the start of rx info.
//
//  \author Ole-Jacob Enderud Jensen
//
//  \date   12.10.2018
*/
//f-/////////////////////////////////////////////////////////////////////////

char *getMRZRxInfo(void* tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      //Move to end of pingInfo
      char* pch = pPI + pPingInfo->numBytesInfoData; //At start of txsectors
      int nSkip = 0;
      pch += nSkip;

      //Get sector
      int nBytesTx = pPingInfo->numBytesPerTxSector;
      pEMdgmMRZ_txSectorInfo pSect = (pEMdgmMRZ_txSectorInfo)pch;

      //Move to start of rxInfo
      pch = (char*)pSect;
      pch += (nBytesTx * numTxSectors);
      EMdgmMRZ_rxInfo* pRxInfo = (EMdgmMRZ_rxInfo*)pch;

      pData = pch;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

//f+/////////////////////////////////////////////////////////////////////////
//
//  getMRZSoundings
/*!
//  Get start ptr of MRZ beam soundings.
//
//  \param  void
//
//  \return Pointer to the start of rx info.
//
//  \author Ole-Jacob Enderud Jensen
//
//  \date   12.10.2018
*/
//f-/////////////////////////////////////////////////////////////////////////

char *getMRZSoundings(void* tgm)
{
   char *pData = NULL;

   // if this record is not empty
   if (tgm != NULL)
   {
      pEMdgmMRZ dgm = (pEMdgmMRZ)tgm;
      int sizeCommon = dgm->cmnPart.numBytesCmnPart;

      // Find rxInfo. Must take account of number of TX sectors.
      int numTxSectors = dgm->pingInfo.numTxSectors;

      //Get PingInfo
      char* pPI = (char*)(&(dgm->partition.dgmNum));
      pPI += 2;
      pPI += sizeCommon;

      pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)pPI;

      //Move to end of pingInfo
      char* pch = pPI + pPingInfo->numBytesInfoData; //At start of txsectors
      int nSkip = 0;
      pch += nSkip;

      //Get sector
      int nBytesTx = pPingInfo->numBytesPerTxSector;
      pEMdgmMRZ_txSectorInfo pSect = (pEMdgmMRZ_txSectorInfo)pch;

      //Move to start of rxInfo
      pch = (char*)pSect;
      pch += (nBytesTx * numTxSectors);
      EMdgmMRZ_rxInfo* pRxInfo = (EMdgmMRZ_rxInfo*)pch;
      int nSizeRXInfo = pRxInfo->numBytesRxInfo;

      // Now, find soundings.
      int numExtraDet = pRxInfo->numExtraDetectionClasses;
      pch += nSizeRXInfo;
      pch += numExtraDet * sizeof(EMdgmMRZ_extraDetClassInfo);
      EMdgmMRZ_sounding* pSoundings = (EMdgmMRZ_sounding*)pch;

      pData = pch;

      // end of datagram for safety testing
      char *pSOD = (char*)dgm;
      char *pEOD = pSOD + dgm->header.numBytesDgm; // first byte past end of data

                                                   // safety check - verify pointer is valid
      if (pData >= pEOD)
      {
         pData = NULL;
      }
   }

   return pData;
}

//! Print help
static void help(char* progname)
{
	cout << "\n\nUsage: " << progname << "\n";
	cout << "          -h (gives this help)\n";
	cout << "          -f <filename>\n";
}

int main(int argc, char *argv[])
{
	string filename("");
	bool helpNeeded = false;
	EMdgmMRZ mrz;
	EMdgmHeader header;
	char *p;

	if (argc == 1) helpNeeded = true;
	string h = string("-h");
	string f = string("-f");
	for (int i = 1; i < argc; i++)
	{
		if (h.compare(string(argv[i])) == 0)
		{
			helpNeeded = true;
			break;
		}
		if (f.compare(argv[i]) == 0)
		{
			i++;
			if (i >= argc)
			{
				helpNeeded = true;
				break;
			}
			filename.assign(argv[i]);
			ifstream fexists(filename);
			if (fexists.good() == false)
			{
				cout << "Filename " << filename << " does not exist.\n";
				helpNeeded = true;
				break;
			}
		}
	}

	if (helpNeeded)
	{
		help(argv[0]);
		exit(0);
	}
	else {
		cout << "Filename: " << filename << "\n";
	}


	// Now do the thing. 
	ifstream kmall(filename, ios::in | ios::binary);
	// Then read datagrams
	long long tmptime;
	while (1 == 1) {
		memset(&mrz, 0, sizeof(mrz));
		memset(&header, 0, sizeof(header));
		// First read header
		kmall.read((char*)&header, sizeof(header));
		if (kmall.gcount() != sizeof(header))
		{
			break;
		}
		// Check datagram type
		if (header.dgmType[0] == '#' && header.dgmType[1] == 'M' && header.dgmType[2] == 'R' && header.dgmType[3] == 'Z')
		{
			p = (char*)&mrz;
			// Put the header into the datagram
			memcpy(p, (char*)&header, sizeof(header));
			p += sizeof(header);
			// Read the rest of the datagram
			kmall.read(p, header.numBytesDgm - sizeof(header));
			if (kmall.gcount() != header.numBytesDgm - sizeof(header))
			{
				break;
			}
			// Make time into millisec
			tmptime = header.time_sec;
			tmptime *= 1000;
			tmptime += (header.time_nanosec / 1000000);

			pEMdgmMRZ depths = &mrz;
         //Structs above rxinfo can grow in newer datagram version. Use getMRZRxInfo to move to the correct location
         pEMdgmMRZ_rxInfo rxInfo = (pEMdgmMRZ_rxInfo)getMRZRxInfo((char*)depths);
			char* pData = (char*)(&(depths->sectorInfo[depths->pingInfo.numTxSectors]));
			unsigned short numExtraDet = ((pEMdgmMRZ_rxInfo)(pData))->numExtraDetectionClasses;
			
         //Structs above sounding can grow in newer datagram version. Use getMRZSoundings to move to the correct location
			pEMdgmMRZ_sounding depthList = (pEMdgmMRZ_sounding)getMRZSoundings((char*)depths);

         //Structs above pinginfo can grow in newer datagram version. Use getMRZPingInfo to move to the correct location
         pEMdgmMRZ_pingInfo pPingInfo = (pEMdgmMRZ_pingInfo)getMRZPingInfo((char*)depths);
			double blat = pPingInfo->latitude_deg;
			double blon = pPingInfo->longitude_deg;

			for (int i = 0; i < rxInfo->numSoundingsMaxMain; i++) {
				double dlat = (depthList + i)->deltaLatitude_deg;
				double dlon = (depthList + i)->deltaLongitude_deg;
				double depthRefPoint = (depthList + i)->z_reRefPoint_m;
				double lat = blat + dlat;
				double lon = blon + dlon;
				double depthReWaterline = depthRefPoint - mrz.pingInfo.z_waterLevelReRefPoint_m;
				cout << std::fixed << std::setprecision(8) << lat << " ";
				cout << std::fixed << std::setprecision(8) << lon << " ";
				cout << std::fixed << std::setprecision(2) << depthReWaterline << " ";
				cout << tmptime << "\n";
			}
		}//if #MRZ
		else {
			// Move file pointer to next datagram
			kmall.seekg(header.numBytesDgm - sizeof(header), ios::cur);
		}
	}//while forever
	int i = 0;
}

