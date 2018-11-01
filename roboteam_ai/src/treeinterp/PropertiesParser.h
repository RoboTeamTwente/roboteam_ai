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
        bool isInt(std::string keyName);
        enum class Type {
                Int,
                String,
                Double,
                Vector,
                Bool
        };

    public:
        bt::Blackboard::Ptr parse(json someJson);


};

#endif //ROBOTEAM_AI_PROPERTIESPARSER_H
