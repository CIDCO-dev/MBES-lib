#ifndef STRINGUTILS_HPP
#define STRINGUTILS_HPP

#include <cstring>

int ends_with(const char * str, const char * suffix) {

        if( str == NULL || suffix == NULL )
                return 0;

        size_t str_len = strlen(str);
        size_t suffix_len = strlen(suffix);

        if(suffix_len > str_len)
                return 0;

        return 0 == strncmp( str + str_len - suffix_len, suffix, suffix_len );
}

#endif
