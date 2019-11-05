//
// Created by jesse on 14.10.19.
//

#include <include/roboteam_ai/bt/tactics/DefaultTactic.h>
#include "bt/BehaviorTree.hpp"
#include "bt/Role.h"
#ifndef RTT_MAKE_TREE_WITHOUT_JSONS_H
#define RTT_MAKE_TREE_WITHOUT_JSONS_H

#endif //RTT_MAKE_TREE_WITHOUT_JSONS_H


namespace bt {

class TreeProtoType {
 public:
    TreeProtoType();
    std::shared_ptr<bt::DefaultTactic> createDefensiveTactic(std::shared_ptr<Blackboard> bb);
    std::shared_ptr<bt::BehaviorTree> createNormalPlayStrategy();
    std::shared_ptr<bt::Role> createDefenderRole(std::string name);

private:
    std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots;
    
};
}