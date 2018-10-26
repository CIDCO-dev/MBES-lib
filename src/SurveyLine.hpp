/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

/* 
 * File:   SurveyLine.hpp
 * Author: jordan
 *
 * Created on September 14, 2018, 2:38 PM
 */

#ifndef SURVEYLINE_HPP
#define SURVEYLINE_HPP

#include "Swath.hpp"

class SurveyLine {
public:
    const std::vector<Swath *>* swaths;
    const unsigned int swathCount;
    
    SurveyLine(std::vector<Swath *>* swaths) :
    swaths(swaths), swathCount(swaths->size()) {
    }
    
    ~SurveyLine() {
        for(unsigned int i=0; i<swathCount; i++) {
            delete (*swaths)[i];
        }

        delete swaths;
    };
    
    friend std::ostream& operator<<(std::ostream& os, const SurveyLine& obj) {
        for(unsigned int i=0; i<obj.swathCount; i++) {
            os << *((*obj.swaths)[i]) << std::endl;
        }
        return os;
    }
};


#endif /* SURVEYLINE_HPP */

