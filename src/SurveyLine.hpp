/*
 * Copyright 2017 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
 */

#ifndef SURVEYLINE_HPP
#define SURVEYLINE_HPP

#include "Swath.hpp"

/*!
 * \brief Survey Line class
 * \author Jordan McManus
 * \date September 14, 2018, 2:38 PM
 */
class SurveyLine {
public:

    /**Vector that contains the swaths of the survey line*/
    const std::vector<Swath *>* swaths;

    /**Value of the number of the survey line swaths*/
    const unsigned int swathCount;

    /**
     * Creates a survey line
     *
     * @param swaths Vector of the survey line swaths
     */
    SurveyLine(std::vector<Swath *>* swaths) :
    swaths(swaths), swathCount(swaths->size()) {
    }

    /**
     * Destroys the survey line and delete all its swaths
     */
    ~SurveyLine() {
        for(unsigned int i=0; i<swathCount; i++) {
            delete (*swaths)[i];
        }

        delete swaths;
    };

    /**
     * Returns a line text for each SurveyLine swath
     *
     * @param os value where the lines will be put in
     * @param obj Survey Line that we need to get the swaths
     */
    friend std::ostream& operator<<(std::ostream& os, const SurveyLine& obj) {
        for(unsigned int i=0; i<obj.swathCount; i++) {
            os << *((*obj.swaths)[i]) << std::endl;
        }
        return os;
    }
};


#endif /* SURVEYLINE_HPP */
