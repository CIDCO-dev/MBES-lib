#ifndef STRINGUTILS_HPP
#define STRINGUTILS_HPP

#include <cstring>
#include <string>
#include <algorithm>

#ifdef _WIN32
#include <cctype>
#endif

/**
 * Return 1 if the first text value ends with the second text value and return 0 if not
 * 
 * @param str first text value
 * @param suffix second text value
 */
int ends_with(const char * str, const char * suffix) {

        if( str == NULL || suffix == NULL )
                return 0;

        size_t str_len = strlen(str);
        size_t suffix_len = strlen(suffix);

        if(suffix_len > str_len)
                return 0;

        return 0 == strncmp( str + str_len - suffix_len, suffix, suffix_len );
}

/**
 * Return a text value without space
 * 
 * @param s text who needs to be trim
 */
std::string trim(const std::string &s)
{
	auto wsfront=std::find_if_not(s.begin(),s.end(),[](int c){return std::isspace(c);});
	auto wsback=std::find_if_not(s.rbegin(),s.rend(),[](int c){return std::isspace(c);}).base();
	return (wsback<=wsfront ? std::string() : std::string(wsfront,wsback));
}

#endif
