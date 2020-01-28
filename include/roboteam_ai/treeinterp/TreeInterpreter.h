//
// Created by baris on 01/10/18.
//

#ifndef ROBOTEAM_AI_TREEINTERPRETER_H
#define ROBOTEAM_AI_TREEINTERPRETER_H

#include <gtest/gtest_prod.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "BTImport.h"
#include "JsonReader.h"
#include "PropertiesParser.h"
#include "Switches.h"
#include "bt/Role.h"
#include <nlohmann/json.hpp>
#include "skills/Skill.h"
#include "vector"

#define GetCurrentDir getcwd

using json = nlohmann::json;

class TreeInterpreter {
   private:
    JsonReader jsonReader;

    PropertiesParser propertyParser;

    std::map<std::string, bt::Node::Ptr> tactics;

    FRIEND_TEST(JsonBasics, JsonTest);

    FRIEND_TEST(TreeTest, JsonTest);

    bt::BehaviorTree::Ptr buildTreeFromJSON(json jsonTree);

    bt::Node::Ptr buildNode(json node, json tree, bt::Blackboard::Ptr globalBlackBoard);

    bool isLeaf(json json);

    bt::Node::Ptr makeNonLeafNode(std::string name);

    bt::Leaf::Ptr makeLeafNode(json jsonLeaf);

    bt::Node::Ptr tacticSwitch(std::string, bt::Blackboard::Ptr properties);

   protected:
   public:
    std::map<std::string, bt::BehaviorTree::Ptr> getTrees(std::string name);

    std::map<std::string, bt::Node::Ptr> makeTactics(std::string fileName, bt::Blackboard::Ptr globalBB);

    static TreeInterpreter &getInstance();
};

#endif  // ROBOTEAM_AI_TREEINTERPRETER_H
