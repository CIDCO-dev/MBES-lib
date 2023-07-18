#ifndef SBETPROCESSOR_CPP
#define SBETPROCESSOR_CPP

#include "SbetProcessor.hpp"

SbetProcessor::SbetProcessor(){

}

SbetProcessor::~SbetProcessor(){

}

int SbetProcessor::doRead(int fd,void * buffer,unsigned int sz){

#ifdef _WIN32
    return _read(fd,buffer,sz);
#endif

#ifdef __GNUC__
    return read(fd,buffer,sz);
#endif

}

int SbetProcessor::doOpen(const char * filename){
#ifdef _WIN32
        return _open(filename,_O_RDONLY|_O_BINARY);
#endif

#ifdef __GNUC__
        return open(filename,O_RDONLY);
#endif
}

/**
 * Read a SBET file and return true if the reading was successful
 * 
 * @param filename name of the SBET file
 */
bool SbetProcessor::readFile(std::string & filename){
    int fd;

    if((fd=doOpen(filename.c_str())) == -1){
        std::cerr << "Cannot open file " << filename << std::endl;
        return false;
    }

    SbetEntry entry;

    int bytesRead;

    do{
        bytesRead = doRead(fd,(void*)&entry,sizeof(SbetEntry));

        if(bytesRead == sizeof(SbetEntry)){
            processEntry(&entry);
        }
    }
    while(bytesRead > 0);

    if(bytesRead == -1)
        perror("Error while reading file");
    

    return true;
}

#endif
