//
// Created by baris on 01/10/18.
//

#ifndef ROBOTEAM_AI_TREEINTERPRETER_H
#define ROBOTEAM_AI_TREEINTERPRETER_H

#include "json.h"
#include "../bt/BehaviorTree.hpp"
#include "JsonReader.h"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include "vector"
#include "../bt/composites/MemSequence.hpp"
#include "../bt/Leaf.hpp"
#include <map>
#include <unistd.h>

#define GetCurrentDir getcwd

using json = nlohmann::json;

class TreeInterpreter {

private:
    JsonReader jsonReader;

    FRIEND_TEST(Tree, JsonTest);

    bt::BehaviorTree buildTreeFromJSON(json jsonTree);

    bt::Node::Ptr buildNode(json json);

    std::vector<json> parseSmallJSONs(json json);

    bool isLeaf(json json);

    bt::MemSequence::Ptr makeNonLeafNode(std::string name);

protected:

public:
    std::map<std::string, bt::BehaviorTree> getProject(std::string name);

    bt::BehaviorTree getTreeWithID(std::string projectName, std::string ID);

    static TreeInterpreter& getInstance();

};

#endif //ROBOTEAM_AI_TREEINTERPRETER_H
