/*
 * Copyright 2020 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   PlaneFitter.hpp
 * Author: Jordan McManus
 */

#ifndef PLANEFITTER_HPP
#define PLANEFITTER_HPP

#include <Eigen/Dense>

class PlaneFitter {
public:

    static void convertPlaneZform2GeneralForm(Eigen::Vector3d & zForm, Eigen::Vector4d & generalForm) {
        /*
         * Z-form is z = Ax + By + C
         * General form is ax + by + cz + d = 0 with (a*a + b*b + c*c = 1 i.e. unit normal vector)
         */
        
        double c = 1.0 / zForm.norm();
        double a = -zForm(0) * c;
        double b = -zForm(1) * c;
        double d = -zForm(2) * c;
        double f = sqrt(a * a + b * b + c * c);
        a = a / f;
        b = b / f;
        c = c / f;
        d = d / f;

        generalForm << a, b, c, d;
    }

    static void convertPlaneGeneralForm2Zform(Eigen::Vector4d & generalForm, Eigen::Vector3d & zForm) {
        /*
         * Z-form is z = Ax + By + C
         * General form is ax + by + cz + d = 0 with (a*a + b*b + c*c = 1 i.e. unit normal vector)
         */
        double f = 1.0 / generalForm(2);
        double A = -generalForm(0) * f;
        double B = -generalForm(1) * f;
        double C = -generalForm(3) * f;
        
        zForm << A, B, C;
    }

    static void calculatePlaneResidualsFromMatrix(Eigen::VectorXd & residuals, Eigen::MatrixXd & cloud, Eigen::Vector4d & planarGeneralForm) {
        /*
         * General form is ax + by + cz + d = 0 with (a*a + b*b + c*c = 1 i.e. unit normal vector)
         */
        
        Eigen::MatrixXd augmentedCloud(cloud.rows(), 4);
        augmentedCloud.col(0) = cloud.col(0);
        augmentedCloud.col(1) = cloud.col(1);
        augmentedCloud.col(2) = cloud.col(2);
        augmentedCloud.col(3) = Eigen::VectorXd::Ones(cloud.rows());

        residuals = augmentedCloud*planarGeneralForm;
    }
    
    static void fitPlane(Eigen::MatrixXd & xyz, Eigen::Vector4d & planeGeneralFormParams) {
        
        Eigen::MatrixXd A(xyz.rows(), 3);
        A.col(0) = xyz.col(0);
        A.col(1) = xyz.col(1);
        A.col(2) = Eigen::VectorXd::Ones(xyz.rows());
        
        Eigen::VectorXd b = xyz.col(2);
        
        Eigen::Vector3d planeParameterEstimation = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        
        convertPlaneZform2GeneralForm(planeParameterEstimation, planeGeneralFormParams);
    }

    static void fitPlane(Eigen::MatrixXd & A, Eigen::VectorXd & b, Eigen::Vector4d & planeGeneralFormParams) {
        Eigen::Vector3d planeParameterEstimation = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        convertPlaneZform2GeneralForm(planeParameterEstimation, planeGeneralFormParams);
    }

};

#endif /* PLANEFITTER_HPP */

