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

using json = nlohmann::json;

class TreeInterpreter {


    private:
        FRIEND_TEST(Tree, JsonTest);

        bt::BehaviorTree buildTreeFromJson(json json);
        json readJson(std::string fileName);


    protected:

    public:
        bt::BehaviorTree getTree(std::string name);
        static TreeInterpreter& getInstance();



};


#endif //ROBOTEAM_AI_TREEINTERPRETER_H
