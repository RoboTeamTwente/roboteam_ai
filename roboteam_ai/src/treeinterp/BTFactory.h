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

class BTFactory {

        // TODO: have the names of all the project before here
        TreeInterpreter interpreter;

    public:
        void init();

        JsonReader jsonReader;

        static BTFactory &getFactory();

        void updateProject(std::string projectName);

        bt::BehaviorTree::Ptr getTree(std::string treeName);

        void updateTree(std::string projectName, std::string treeName);

        static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

        static std::map<std::string, bt::Node::Ptr> tacticsRepo;

        bool isIsInitiated() const;

        void setIsInitiated(bool isInitiated);

    protected:
//       static bool isInitiated;


};

#endif //ROBOTEAM_AI_BTFACTORY_H
