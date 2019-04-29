
/* 
 * File:   Exception.hpp
 * Author: jordan
 *
 * Created on March 28, 2018, 12:45 PM
 */

#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <exception>

/*!
 * \brief Exception class extend of std::exception
 */
class Exception : public std::exception {
private:
    
    /**Value text of the Exception message*/
    std::string message;

public:

    /**
     * Create a Exception
     * 
     * @param msg value of the Exception message
     */
    Exception(std::string msg) : message(msg) {
    }

    /**Return the message in character line*/
    std::string & getMessage() {
        return message;
    }
};


#endif /* EXCEPTION_HPP */

