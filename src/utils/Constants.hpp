/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

/*!
* \author Guillaume Labbe-Morissette
*/
#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#ifdef _WIN32
#define M_PI 3.14159265358979323846
#endif
#define PI M_PI
#define INF 1.e100
#define R2D ((double)180/(double)PI)
#define D2R ((double)PI/(double)180)
#define a_wgs84 6378137.0
#define e2_wgs84 (0.081819190842622 * 0.081819190842622)

#ifdef _WIN32
#include <direct.h>
#define DIRECTORY_SEPARATOR "\\"
#elif __GNUC__
#include <unistd.h>
#define DIRECTORY_SEPARATOR "/"
#endif


#endif
