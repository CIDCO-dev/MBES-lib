/*
* Copyright 2019 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
*/

#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <stdexcept>
#include <cmath>

#include "../Position.hpp"
#include "../Attitude.hpp"
#include "../utils/Exception.hpp"

/*!
* \brief Interpolator class
* \author Guillaume Labbe-Morissette, Jordan McManus
* \date October 2, 2018, 3:04 PM
*/
class Interpolator {
public:

  /**
  * Returns an interpolated position between two position(position)
  *
  * @param p1 first position
  * @param p2 second position
  * @param timestamp time in microsecond since 1st January 1970
  */
  static Position* interpolatePosition(Position & p1, Position & p2, uint64_t timestamp) {
    double interpLat = linearInterpolationByTime(p1.getLatitude(), p2.getLatitude(), timestamp, p1.getTimestamp(), p2.getTimestamp());
    double interpLon = linearInterpolationByTime(p1.getLongitude(), p2.getLongitude(), timestamp, p1.getTimestamp(), p2.getTimestamp());
    double interpAlt = linearInterpolationByTime(p1.getEllipsoidalHeight(), p2.getEllipsoidalHeight(), timestamp, p1.getTimestamp(), p2.getTimestamp());
    return new Position(timestamp,interpLat, interpLon, interpAlt);
  }

  /**
  * Returns an interpolated attitude between two attitude(attitude)
  *
  * @param a1 first attitude
  * @param a2 second attitude
  * @param timestamp time in microsecond since 1st January 1970
  */
  static Attitude* interpolateAttitude(Attitude & a1, Attitude & a2,uint64_t timestamp) {
    double interpRoll = linearAngleInterpolationByTime(a1.getRoll(), a2.getRoll(), timestamp, a1.getTimestamp(), a2.getTimestamp());
    double interpPitch = linearAngleInterpolationByTime(a1.getPitch(), a2.getPitch(), timestamp, a1.getTimestamp(), a2.getTimestamp());
    double interpHeading = linearAngleInterpolationByTime(a1.getHeading(), a2.getHeading(), timestamp, a1.getTimestamp(), a2.getTimestamp());
    return new Attitude(timestamp,interpRoll, interpPitch, interpHeading);
  }

  /**
  * Returns a linear interpolation between two meter
  *
  * @param y1 first meter
  * @param y2 second meter
  * @param x number of microsecond since 1st January 1970
  * @param x1 timestamp link y1
  * @param x2 timestamp link to y2
  */
  static double linearInterpolationByTime(double y1, double y2, uint64_t x, uint64_t x1, uint64_t x2) {
      if (x1 == x2)
      {
          throw new Exception("The two positions timestamp are the same");
      }
      if (x1 > x)
      {
          throw new Exception("The first position timestamp is higher than interpolation timestamp");
      }
      if (x1 > x2)
      {
          throw new Exception("The first position timestamp is higher than the second position timestamp");
      }
    double result = (y1 + (y2 - y1)*(x - x1) / (x2 - x1));
    return result;
  }

  /**
  * Returns a linear interpolation between two angle
  *
  * @param psi1 first angle
  * @param psi2 second angle
  * @param t number of microsecond since 1st January 1970
  * @param t1 timestamp link to psi1
  * @param t2 timestamp link to psi2
  */
  static double linearAngleInterpolationByTime(double psi1, double psi2, uint64_t t, uint64_t t1, uint64_t t2) {

    if (psi1 < 0 || psi1 >= 360 || psi2 < 0 || psi2 >= 360) {
      throw new Exception("Angles need to be between 0 (inclusive) and 360 (exclusive) degrees");
    }
    
    if (t1 == t2)
      {
          throw new Exception("The two positions timestamp are the same");
      }
      if (t1 > t)
      {
          throw new Exception("The first position timestamp is higher than interpolation timestamp");
      }
      if (t1 > t2)
      {
          throw new Exception("The first position timestamp is higher than the second position timestamp");
      }
    
    bool multiAnswer = false;
    std::stringstream ss;
    if (std::abs(psi2 - psi1)==180)
    {
        ss << "The angles " << psi1 << " and " << psi2
                << " have a difference of 180 degrees witch mean there is two possible answer at the timestamp " << t << ": ";
        multiAnswer = true;
    }

    if (psi1 == psi2) {
      return psi1;
    }

    double x1 = t-t1;
    double x2 = t2-t1;
    double delta = (x1 / x2);
    double dpsi = std::fmod((std::fmod(psi2 - psi1, 360) + 540), 360) - 180;
    double interpolation = psi1 + dpsi*delta;

    if (multiAnswer)
    {
        ss << std::abs(std::fmod(interpolation,360)) << " and " << -std::abs(std::fmod(interpolation,360))+360 << "\n";
        throw new Exception(ss.str());
    }
        
    if(interpolation >= 0 && interpolation < 360) {
      return interpolation;
    }

    double moduloInterpolation = std::fmod(interpolation, 360);

    if(moduloInterpolation < 0) {
       return moduloInterpolation + 360;
    }
    
    return moduloInterpolation;
  }

  /**
  * Returns a linear interpolation between two radian angle
  *
  * @param psi1 first angle
  * @param psi2 second angle
  * @param t number of microsecond since 1st January 1970
  * @param t1 timestamp link psi1
  * @param t2 timestamp link psi2
  */
  static double linearAngleRadiansInterpolationByTime(double psi1, double psi2, uint64_t t, uint64_t t1, uint64_t t2) {
    double interpDegrees = linearAngleInterpolationByTime(psi1*180.0/M_PI, psi2*180.0/M_PI, t, t1, t2);
    return interpDegrees*M_PI/180.0;
  }
};

#endif /* INTERPOLATOR_HPP */
