#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#define PI M_PI
#define INF 1.e100
#define R2D ((double)180/(double)PI)
#define D2R ((double)PI/(double)180)

#ifdef _WIN32
#include <direct.h>
#define DIRECTORY_SEPARATOR "\\"
#endif

#ifdef __GNUC__
#include <unistd.h>
#define DIRECTORY_SEPARATOR "/"
#endif


#endif
