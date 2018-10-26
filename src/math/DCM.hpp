/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   DCM.hpp
 * Author: jordan
 *
 * Created on September 13, 2018, 10:34 AM
 */

#ifndef DCM_HPP
#define DCM_HPP

#include "../Attitude.hpp"
#include <Eigen/Dense>

class DCM {
public:

    static void getDcmRollAtReceptionPitchHeadingAtEmission(Eigen::Matrix3d & outputMatrix,const Attitude & receptionAttitude, const Attitude & transmissionAttitude) {
        return getDcm(outputMatrix,receptionAttitude.sr, receptionAttitude.cr, transmissionAttitude.sp, transmissionAttitude.cp, transmissionAttitude.sh, transmissionAttitude.ch);
    };

    static void getDcm(Eigen::Matrix3d & outputMatrix,const Attitude & attitude) {
        return getDcm(outputMatrix,attitude.sr, attitude.cr, attitude.sp, attitude.cp, attitude.sh, attitude.ch);
    };

    static void getDcm(Eigen::Matrix3d & outputMatrix,double sr, double cr, double sp, double cp, double sh, double ch) {
        double m00 = ch*cp;
        double m01 = ch * sp * sr - sh*cr;
        double m02 = ch * sp * cr + sh*sr;

        double m10 = sh*cp;
        double m11 = sh * sp * sr + ch*cr;
        double m12 = sh * sp * cr - ch*sr;

        double m20 = -sp;
        double m21 = cp*sr;
        double m22 = cp*cr;

        outputMatrix << m00, m01, m02,
            m10, m11, m12,
            m20, m21, m22;
    };
};


#endif /* DCM_HPP */

