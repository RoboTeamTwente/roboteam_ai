//
// Created by jesse on 14.10.19.
//
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {

std::shared_ptr<BehaviorTree> TreeProtoType::createNormalPlayStrategy() {
    std::shared_ptr<RoleDivider> roleDivider = std::make_shared<RoleDivider>();
    std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();
    bb->setString("TacticType", "General");

    std::shared_ptr<DefaultTactic> defensiveTactic = createDefensiveTactic(bb);

    //roleDivider->addChild(defensiveTactic);
    std::shared_ptr<rtt::ai::Attack> attackskill = std::make_shared<rtt::ai::Attack>("attack", std::make_shared<Blackboard>());
    defensiveTactic->setProperties(bb);
    auto tree = std::make_shared<BehaviorTree>("defendertree");
    tree->SetRoot(defensiveTactic);
    return tree;
}

std::shared_ptr<DefaultTactic> TreeProtoType::createDefensiveTactic(std::shared_ptr<Blackboard> bb) {

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



    // create a default tactic which will be used to build the defensive tactic
    std::shared_ptr<DefaultTactic> defensiveTactic = std::make_shared<DefaultTactic>("defensiveTactic", bb, robots);

    // TODO: change the 8 to a "number of robots"
    // Creating the roles for all the robots in the tactic:
    for (int i = 1; i < robots.size(); i++) {
        std::string name = "o" + std::to_string(i);
        auto temp = createDefenderRole(name, bb);
        std::shared_ptr<Role> temprole = createDefenderRole(name, bb);
        temp->giveProperty("ROLE", name);
        defensiveTactic->addChild(temprole);
    }

    return defensiveTactic;
}

/// This function creates a defender role. This role consists of a repeater with as child an attack skill
std::shared_ptr<Role> TreeProtoType::createDefenderRole(std::string rolename, std::shared_ptr<Blackboard> bb) {

    // set the rolename for the current role. This is important because the robotdealer decides how to deal the robots based on their rolenames
    bb->setString("ROLE", rolename);
    std::shared_ptr<Role> role = std::make_shared<Role>(rolename);

    // Give the role the blackboard
    role->setProperties(bb);

    // Make a repeater with the child "attack"
    std::shared_ptr<Repeater> repeater = std::make_shared<Repeater>();
    std::shared_ptr<rtt::ai::Attack> attack = std::make_shared<rtt::ai::Attack>("attack",  bb);

    repeater->addChild(attack);
    role->addChild(repeater);
    std::cout << "creating defender role" << rolename << std::endl;
    return role;

}



}
