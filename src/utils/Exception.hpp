
#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <exception>

/*!
* \brief Exception class.
*
* Extends from std::exception
* \author Jordan MacManus
* \date March 28, 2018, 12:45 PM
*/
class Exception : public std::exception {
private:

  /**Text value of the Exception message*/
  std::string message;

public:

  /**
  * Creates an Exception
  *
  * @param msg Text value of the Exception message
  */
  Exception(std::string msg) : message(msg) {
  }

  /**Returns the message in character line*/
  std::string & getMessage() {
    return message;
  }
};


#endif /* EXCEPTION_HPP */
