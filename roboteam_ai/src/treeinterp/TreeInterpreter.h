//
// Created by baris on 01/10/18.
//

#ifndef ROBOTEAM_AI_TREEINTERPRETER_H
#define ROBOTEAM_AI_TREEINTERPRETER_H

#include "json.h"
#include "JsonReader.h"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include "vector"
#include <map>
#include <unistd.h>
#include "BTImport.h"
#include "PropertiesParser.h"

#define GetCurrentDir getcwd

using json = nlohmann::json;

class TreeInterpreter {

    private:
        JsonReader jsonReader;

        PropertiesParser propertyParser;

        FRIEND_TEST(JsonBasics, JsonTest);

        FRIEND_TEST(TreeTest, JsonTest);

        bt::BehaviorTree::Ptr buildTreeFromJSON(json jsonTree);

        bt::Node::Ptr buildNode(json node, json tree, bt::Blackboard::Ptr globalBlackBoard);

        std::vector<json> parseSmallJSONs(json json);

        bool isLeaf(json json);

        bt::Node::Ptr makeNonLeafNode(std::string name);

        bt::Leaf::Ptr makeLeafNode(json jsonLeaf);

        std::map<std::string, bt::Node::Ptr> tactics;

        bt::Node::Ptr tacticSwitch(std::string, bt::Blackboard::Ptr properties);


    protected:

    public:
        std::map<std::string, bt::BehaviorTree::Ptr> getTrees(std::string name);

        std::map<std::string, bt::Node::Ptr> makeTactics(std::string fileName, bt::Blackboard::Ptr globalBB);

        static TreeInterpreter &getInstance();


};

#endif //ROBOTEAM_AI_TREEINTERPRETER_H
