/*
* Copyright 2019-2023 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <exception>

/*!
* \brief Exception class.
*
* Extends from std::exception
* \author https://stackoverflow.com/questions/8152720/correct-way-to-inherit-from-stdexception
*/
class Exception : public std::exception {
protected:

  /**Text value of the Exception message*/
  std::string message;

public:


  explicit Exception(const std::string msg) : message(msg) {}
  
  explicit Exception(const char* msg): message(msg) {}
  
  virtual ~Exception() noexcept {}

  /**Returns the message in character line*/
  virtual const char* what() const noexcept{
    return message.c_str();
  }
};


#endif /* EXCEPTION_HPP */
