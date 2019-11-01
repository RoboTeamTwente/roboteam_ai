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
    std::shared_ptr<bt::DefaultTactic> createDefensiveTactic(std::shared_ptr<Blackboard> bb);
    std::shared_ptr<bt::BehaviorTree> createNormalPlayStrategy();
    std::shared_ptr<bt::Role> createDefenderRole(std::string name);

private:
    // Set the robottypes for the robot so the robotdealer can decide which robot should do what
    std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots = {
            {"o1", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o2", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o3", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o4", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o5", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o6", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o7", rtt::ai::robotDealer::RobotType::RANDOM},
            {"o8", rtt::ai::robotDealer::RobotType::RANDOM}
    };
};
}