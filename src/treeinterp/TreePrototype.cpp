//
// Created by jesse on 14.10.19.
//
#include <include/roboteam_ai/bt/composites/Selector.hpp>
#include "bt/RoleDivider.h"
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/TreePrototype.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {

std::shared_ptr<BehaviorTree> TreeProtoType::createNormalPlayStrategy() {
    std::shared_ptr<RoleDivider> roleDivider = std::make_shared<RoleDivider>();
    std::shared_ptr<DefaultTactic> defensiveTactic = createDefensiveTactic();
    std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();
    bb->setString("TacticType", "General");

    //roleDivider->addChild(defensiveTactic);
    std::shared_ptr<rtt::ai::Attack> attackskill = std::make_shared<rtt::ai::Attack>("attack", std::make_shared<Blackboard>());
    defensiveTactic->setProperties(bb);
    auto tree = std::make_shared<BehaviorTree>("defendertree");
    tree->SetRoot(defensiveTactic);
    return tree;
}

std::shared_ptr<DefaultTactic> TreeProtoType::createDefensiveTactic() {

  std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots = {
        {"o1", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o2", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o3", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o4", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o5", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o6", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o7", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL},
        {"o8", rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL}
  };



  auto bb = std::make_shared<Blackboard>();

    std::shared_ptr<DefaultTactic> defensiveTactic = std::make_shared<DefaultTactic>("defensiveTactic", bb, robots);
    for (int i = 0; i < 11; i++) {
      std::string name = "o" + i;
        auto temp = createDefenderRole(name);
        temp->giveProperty("ROLE", name);
        defensiveTactic->addChild(temp);
    }
    return defensiveTactic;
}

std::shared_ptr<Role> TreeProtoType::createDefenderRole(std::string rolename) {
    std::shared_ptr<Role> role = std::make_shared<Role>(rolename);
    std::shared_ptr<Repeater> repeaterino = std::make_shared<Repeater>();
    //std::shared_ptr<Selector> selectorino = std::make_shared<Selector>();

    std::shared_ptr<rtt::ai::Attack> attackio = std::make_shared<rtt::ai::Attack>("attack",  std::make_shared<Blackboard>());

    // Make a repeater with the child "attack"
    repeaterino->addChild(attackio);
//    role->addChild(selectorino);
    role->addChild(repeaterino);
    //selectorino->addChild(repeaterino);
    std::cout << "creating defender role" << std::endl;
    return role;

}



}
