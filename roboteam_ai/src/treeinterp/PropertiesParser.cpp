//
// Created by baris on 31/10/18.
//

#include <utility>
#include "PropertiesParser.h"
#include "../bt/Blackboard.hpp"


bt::Blackboard::Ptr PropertiesParser::parse(PropertiesParser::json jsonLeaf) {
    bt::Blackboard::Ptr BB = std::make_shared<bt::Blackboard>();
    for ( auto &property : jsonLeaf["properties"].get<json::object_t>() ) {
        if (isInt(property.first)) {
           // BB->setInt()
        }

    }
    return BB;
}
bool PropertiesParser::isInt(std::string keyName) {
    return false;
}


