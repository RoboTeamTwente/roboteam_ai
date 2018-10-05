//
// Created by baris on 01/10/18.
//

#ifndef ROBOTEAM_AI_TREEINTERPRETER_H
#define ROBOTEAM_AI_TREEINTERPRETER_H

#include "json.h"
#include "../bt/BehaviorTree.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include "vector"
#include <map>


using json = nlohmann::json;

class TreeInterpreter {


    private:
        FRIEND_TEST(Tree, JsonTest);

        bt::BehaviorTree buildTreeFromJSON(json json);
        json readJSON(std::string fileName);
        std::vector<json> parseSmallJSONs(json json);

    protected:

    public:
        std::map<std::string, bt::BehaviorTree> getTrees(std::string name);
        static TreeInterpreter& getInstance();

};


#endif //ROBOTEAM_AI_TREEINTERPRETER_H
