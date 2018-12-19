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
#include "Switches.h"

class BTFactory {

        // TODO: have the names of all the project before here
        TreeInterpreter interpreter;

        static std::string currentTree;

        static std::string keeperTree;

        static int keeperID;

        static bool initialized;

    public:
        void init();

        static BTFactory &getFactory();

        bt::BehaviorTree::Ptr getTree(std::string treeName);

        static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

        static std::map<std::string, bt::Node::Ptr> tacticsRepo;

        static std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo;

        static std::string getCurrentTree();

        static bt::BehaviorTree::Ptr getKeeperTree();

        static void setCurrentTree(const std::string &currentTree);

        static void setKeeperTree(const std::string &keeperTree);

        static void setKeeper(int newID);

        static bool isInitialized();
};

#endif //ROBOTEAM_AI_BTFACTORY_H
