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

    public:
        void init();

        static BTFactory &getFactory();

        bt::BehaviorTree::Ptr getTree(std::string treeName);

        static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

        static std::map<std::string, bt::Node::Ptr> tacticsRepo;


};

#endif //ROBOTEAM_AI_BTFACTORY_H
