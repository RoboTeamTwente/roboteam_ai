//
// Created by jesse on 14.10.19.
//

#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreeProtoType.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {
/**
 * This class makes a tree for a defensive strategy. It is a proof of concept
 *
 * The structure of a behaviour tree is as follows:
 * The tree has a root node
 *
 * Each tactic should have a roledivider. This roledivider uses a "robots" vector to determine which robots
 * the robotdealer should match to which physical robot ids.
 *
 *
 * @return
 */

    // Set the robottypes for the robot so the robotdealer can decide which robot should do what
    TreeProtoType::TreeProtoType(){
        this->robots = {
                {"o1", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o2", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o3", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o4", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o5", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o6", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o7", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o8", rtt::ai::robotDealer::RobotType::RANDOM}
        };
    }
    
    

std::shared_ptr<BehaviorTree> TreeProtoType::createNormalPlayStrategy() {
    std::shared_ptr<RoleDivider> roleDivider = std::make_shared<RoleDivider>();
    std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();

    // Set the tactictype so the roledivider can divide the robots
    // Currently there are 2 options: General and (Offensive/Defensive/Midfield).
    // All roledividers must support all robots, so the number of children of a roledivider = number of robots.
    bb->setString("TacticType", "General");

    std::shared_ptr<DefaultTactic> defensiveTactic = createDefensiveTactic(bb);
    defensiveTactic->setProperties(bb);

    auto tree = std::make_shared<BehaviorTree>("defendertree");
    tree->SetRoot(defensiveTactic);
    return tree;
}

std::shared_ptr<DefaultTactic> TreeProtoType::createDefensiveTactic(std::shared_ptr<Blackboard> bb) {
    // create a default tactic which will be used to build the defensive tactic
    std::shared_ptr<DefaultTactic> defensiveTactic = std::make_shared<DefaultTactic>("defensiveTactic", bb, robots);

    // Creating the roles for all the robots in the tactic:
    for (int i = 1; i < robots.size(); i++) {
        std::string name = "o" + std::to_string(i);
        auto temp = createDefenderRole(name);
        std::shared_ptr<Role> temprole = createDefenderRole(name);
        temp->giveProperty("ROLE", name);
        defensiveTactic->addChild(temprole);

    }

    return defensiveTactic;
}

/// This function creates a defender role. This role consists of a repeater with as child an attack skill
std::shared_ptr<Role> TreeProtoType::createDefenderRole(std::string rolename) {
    // set the rolename for the current role. This is important because the robotdealer decides how to deal the robots based on their rolenames
    // localbb is the blackboard that is given to the attack skill. The property "ROLE" should be set to the name of the role, which should correspond to the rolename
    // found in the robotdealer robots vector
    auto localbb = std::make_shared<Blackboard>();
    localbb->setString("ROLE", rolename);
    std::shared_ptr<Role> role = std::make_shared<Role>(rolename);

    // Give the role the blackboard. This is used by the robotdealer to find the robot
    // TODO: figure out the exact difference between giving ROLE property to Role blackboard vs giving ROLE property to Skill blackboard
    role->setProperties(localbb);

    // Make a repeater with the child "attack"
    std::shared_ptr<Repeater> repeater = std::make_shared<Repeater>();
    std::shared_ptr<rtt::ai::Attack> attack = std::make_shared<rtt::ai::Attack>("attack",  localbb);

    repeater->addChild(attack);
    role->addChild(repeater);
    std::cout << "creating defender role" << rolename << std::endl;
    return role;

}



}
