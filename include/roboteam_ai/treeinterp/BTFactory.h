//
// Created by baris on 04/10/18.
//

#ifndef ROBOTEAM_AI_BTFACTORY_H
#define ROBOTEAM_AI_BTFACTORY_H

#include <gtest/gtest_prod.h>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include "BTImport.h"
#include "TreeInterpreter.h"
#include "stp/play-utilities/Play.h"

class BTFactory {
    // TODO: have the names of all the project before here

    static std::mutex keeperTreeMutex;

   public:
    static void makeTrees();

    static bt::BehaviorTree::Ptr getTree(std::string treeName);

    static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

    static std::map<std::string, bt::Node::Ptr> tacticsRepo;

    static std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo;

    static std::map<std::string, bt::BehaviorTree::Ptr> codeTrees;

    static std::string getCurrentTree();

    static bt::BehaviorTree::Ptr getKeeperTree();

    static void setCurrentTree(rtt::ai::analysis::Play *play);

    static void setCurrentTree(const std::string &newTree);

    static void setKeeperTree(const std::string &keeperTree);

    static void halt();

    static std::string getKeeperTreeName();

    static bool hasMadeTrees();

   private:
    static std::string currentTree;
    static std::string keeperTree;
    static bool weMadeTrees;
    static rtt::ai::analysis::Play *play;
};

#endif  // ROBOTEAM_AI_BTFACTORY_H
