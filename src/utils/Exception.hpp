
/* 
 * File:   Exception.hpp
 * Author: jordan
 *
 * Created on March 28, 2018, 12:45 PM
 */

#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <exception>

class Exception : public std::exception {
private:
    std::string message;

public:

    Exception(std::string msg) : message(msg) {
    }

    const char* what() const throw () {
        return message.c_str();
    }
};


#endif /* EXCEPTION_HPP */

