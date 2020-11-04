/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef STRINGUTILS_CPP
#define STRINGUTILS_CPP

#include "StringUtils.hpp"
#include <sstream>

/**
* Returns if the first text value ends with the second text value is true or false
*
* @param str first text value
* @param suffix second text value
*/
bool StringUtils::ends_with(const char * str, const char * suffix) {

  if( str == NULL || suffix == NULL )
  return 0;

  size_t str_len = strlen(str);
  size_t suffix_len = strlen(suffix);

  if(suffix_len > str_len)
  return 0;

  return 0 == strncmp( str + str_len - suffix_len, suffix, suffix_len );
}


/**
* Returns if the first text value ends with the second text value is true or false. Case independant
*
* @param str first text value
* @param suffix second text value
*/
bool StringUtils::ends_with_ci(const char * str, const char * suffix) {

  if( str == NULL || suffix == NULL )
  return 0;

  size_t str_len = strlen(str);
  size_t suffix_len = strlen(suffix);

  if(suffix_len > str_len)
  return 0;

  return 0 == StringUtils::strcmpi( str + str_len - suffix_len, suffix );
}

/* Returns true if strings are equal. Case independant
 * 
 */
bool StringUtils::strcmpi(const std::string& a, const std::string& b)
{
    if (b.size() != a.size()){
        return false;
    }
    else{
	for (unsigned int i = 0; i < a.size(); ++i){
	        if (tolower(a[i]) != tolower(b[i])){
	            return false;
		}
	}
    }

    return true;
}

/**
* Returns a text value without space
*
* @param s text that needs to be trimmed
*/
std::string StringUtils::trim(const std::string &s)
{
  auto wsfront=std::find_if_not(s.begin(),s.end(),[](int c){return std::isspace(c);});
  auto wsback=std::find_if_not(s.rbegin(),s.rend(),[](int c){return std::isspace(c);}).base();
  return (wsback<=wsfront ? std::string() : std::string(wsfront,wsback));
}

/**
 * Returns a string with the requested decimals
 */
std::string StringUtils::to_string_with_precision(const double a_value, const int n)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

#endif
