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

        char vectorStartChar = '{', vectorEndChar = '}', dot = '.', comma = ',', space = ' ';

        int checkVarTypeOfString(std::string keyName, json someJson, std::vector<double> &vec);

        int getNumberFromString(std::string strKey, char charKey[100], int &it, double &number);



        enum class Type {
                Int,
                String,
                Double,
                Vector2,
                Bool
        };

    public:
        bt::Blackboard::Ptr parse(json someJson);


};

#endif //ROBOTEAM_AI_PROPERTIESPARSER_H
