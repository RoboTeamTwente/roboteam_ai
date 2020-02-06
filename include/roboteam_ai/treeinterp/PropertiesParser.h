//
// Created by baris on 31/10/18.
//

#ifndef ROBOTEAM_AI_PROPERTIESPARSER_H
#define ROBOTEAM_AI_PROPERTIESPARSER_H

#include <math.h>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "JsonReader.h"
#include "bt/Blackboard.h"
#include "roboteam_utils/Vector2.h"

class PropertiesParser {
    using json = nlohmann::json;

   private:
    JsonReader jsonReader;

    enum type { Int, String, Double, Vector, Bool_True, Bool_False };

    char vectorStartChar = '{', vectorEndChar = '}', dot = '.', comma = ',', space = ' ', minus = '-';

    type checkVarTypeOfString(std::string keyName, json someJson, std::vector<double> &vec);

    type getNumberFromString(std::string strKey, std::vector<char> charKey, int &it, double &number);

   public:
    bt::Blackboard::Ptr parse(json someJson);
};

#endif  // ROBOTEAM_AI_PROPERTIESPARSER_H
