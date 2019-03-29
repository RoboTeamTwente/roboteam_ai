//
// Created by baris on 04/10/18.
//

#ifndef ROBOTEAM_AI_BTFACTORY_H
#define ROBOTEAM_AI_BTFACTORY_H

#include "json.h"
#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include "TreeInterpreter.h"
#include "BTImport.h"
#include "roboteam_ai/src/Switches.h"

namespace rtt {
namespace ai {
namespace treeinterp {

class BTFactory {
private:
    TreeInterpreter interpreter;
    std::string currentTree = "NaN";
    std::string keeperTree;
    int keeperID;
    bool initialized = false;
public:
    void init();
    bt::BehaviorTree::Ptr getTree(std::string treeName);
    std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;
    std::map<std::string, bt::Node::Ptr> tacticsRepo;
    std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo;
    std::string getCurrentTree();
    bt::BehaviorTree::Ptr getKeeperTree();
    void setCurrentTree(const std::string& currentTree);
    void setKeeperTree(const std::string& keeperTree);
    void setKeeper(int newID);
    bool isInitialized();
    void halt();
};

extern BTFactory g_btfactory;

} // treeinterp
} // ai
} // rtt

#endif //ROBOTEAM_AI_BTFACTORY_H
