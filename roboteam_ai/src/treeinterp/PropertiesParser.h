//
// Created by baris on 31/10/18.
//

#ifndef ROBOTEAM_AI_PROPERTIESPARSER_H
#define ROBOTEAM_AI_PROPERTIESPARSER_H

#include <string>
#include "roboteam_utils/Vector2.h"
#include "../bt/Blackboard.hpp"
#include "json.h"

class PropertiesParser {
        using json = nlohmann::json;

    private:

        enum type {
          Int,
          String,
          Double,
          Vector,
          Bool_True,
          Bool_False
        };

        char vectorStartChar = '{', vectorEndChar = '}', dot = '.', comma = ',', space = ' ';

        type checkVarTypeOfString(std::string keyName, json someJson, std::vector<double> &vec);

        type getNumberFromString(std::string strKey, char charKey[100], int &it, double &number);




    public:
        bt::Blackboard::Ptr parse(json someJson);


};

#endif //ROBOTEAM_AI_PROPERTIESPARSER_H
