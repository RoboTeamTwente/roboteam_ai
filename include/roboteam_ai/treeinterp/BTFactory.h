

//
// Created by baris on 04/10/18.
//

#ifndef ROBOTEAM_AI_BTFACTORY_H
#define ROBOTEAM_AI_BTFACTORY_H

#include <gtest/gtest_prod.h>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include "BTImport.h"
#include "TreeInterpreter.h"
#include "json.h"

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

    static void setCurrentTree(const std::string &currentTree);

    static void setKeeperTree(const std::string &keeperTree);

    static void halt();

    static std::string getKeeperTreeName();

    static bool hasMadeTrees();

   private:
    static std::string currentTree;
    static std::string keeperTree;
    static bool weMadeTrees;

    static std::shared_ptr<bt::BehaviorTree> testing_tree;
};

#endif  // ROBOTEAM_AI_BTFACTORY_H
