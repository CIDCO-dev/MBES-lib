/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef STRINGUTILS_HPP
#define STRINGUTILS_HPP

#include <cstring>
#include <string>
#include <algorithm>

#ifdef _WIN32
#include <cctype>
#endif

/*!
* \author Guillaume Labbe-Morissette
*/


class StringUtils{
public:    
    /**
    * Returns if the first text value ends with the second text value is true or false
    *
    * @param str first text value
    * @param suffix second text value
    */
    static bool ends_with(const char * str, const char * suffix);
    

    /**
    * Returns a text value without space
    *
    * @param s text that needs to be trimmed
    */
    static std::string trim(const std::string &s);
    
};

#endif
