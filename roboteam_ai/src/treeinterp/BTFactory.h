//
// Created by baris on 04/10/18.
//

#ifndef ROBOTEAM_AI_BTFACTORY_H
#define ROBOTEAM_AI_BTFACTORY_H

#include "json.h"
#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include <map>
#include "TreeInterpreter.h"
#include "BTImport.h"
#include "roboteam_ai/src/Switches.h"

class BTFactory {

        // TODO: have the names of all the project before here
        static std::string currentTree;
        static std::string keeperTree;

    public:
        static void makeTrees();

        static bt::BehaviorTree::Ptr getTree(std::string treeName);

        static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

        static std::map<std::string, bt::Node::Ptr> tacticsRepo;

        static std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo;

        static std::string getCurrentTree();

        static bt::BehaviorTree::Ptr getKeeperTree();

        static void setCurrentTree(const std::string &currentTree);

        static void setKeeperTree(const std::string &keeperTree);

        static void halt();
};

#endif //ROBOTEAM_AI_BTFACTORY_H
