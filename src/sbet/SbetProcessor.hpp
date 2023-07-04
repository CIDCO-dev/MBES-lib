/**
 * @author Guillaume Labbe-Morissette
 */

#ifndef SBETPROCESSOR_HPP
#define SBETPROCESSOR_HPP

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string>
#include <iostream>

#ifdef __GNUC__
#include <unistd.h>
#endif

#ifdef _WIN32
#include <io.h>
#endif

#pragma pack(1)
typedef struct{
	double time;    	//GPS seconds of week
	double latitude;	//latitude in radians
	double longitude;	//longitude in radians
	double altitude;    	//altitude
	double xSpeed;    	//velocity in x direction
	double ySpeed;    	//velocity in y direction
	double zSpeed;    	//velocity in z direction
	double roll;    	//roll angle
	double pitch;    	//pitch angle
	double heading;    	//heading angle
	double wander;    	//wander
	double xForce;    	//force in x direction
	double yForce;    	//force in y direction
	double zForce;    	//force in z direction
	double xAngularRate;   	//angular rate in x direction
	double yAngularRate;   	//angular rate in y direction
	double zAngularRate;    //angular rate in z direction
} SbetEntry;
#pragma pack()

class SbetProcessor{
	public:
		SbetProcessor();
        virtual ~SbetProcessor();

		bool readFile(std::string & filename);
		virtual void processEntry(SbetEntry * entry)=0;

	private:
		int doRead(int fd,void* buf,unsigned int sz);
		int doOpen(const char * filename);
};


#endif
