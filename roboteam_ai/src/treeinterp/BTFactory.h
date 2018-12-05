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

public:
        void init();

        static BTFactory &getFactory();

        bt::BehaviorTree::Ptr getTree(std::string treeName);

        static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

        static std::map<std::string, bt::Node::Ptr> tacticsRepo;

        static std::string getCurrentTree();

        static void setCurrentTree(const std::string &currentTree);
};

#endif //ROBOTEAM_AI_BTFACTORY_H
