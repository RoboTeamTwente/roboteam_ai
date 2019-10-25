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

using namespace rtt::ai;


std::shared_ptr<bt::BehaviorTree> bt::createNormalPlayStrategy() {
    std::shared_ptr<bt::RoleDivider> roleDivider = std::make_shared<bt::RoleDivider>();
    std::shared_ptr<bt::DefaultTactic> defensiveTactic = bt::createDefensiveTactic();


    //defensiveTactic->

    std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();
    bb->setString("TacticType", "General");

    //roleDivider->addChild(defensiveTactic);
    std::shared_ptr<Attack> attackskill = std::make_shared<Attack>("attack", std::make_shared<bt::Blackboard>());
    defensiveTactic->setProperties(bb);
    auto tree = std::make_shared<bt::BehaviorTree>("defendertree");
    tree->SetRoot(defensiveTactic);
    return tree;
}

std::shared_ptr<bt::DefaultTactic> bt::createDefensiveTactic() {
    std::shared_ptr<bt::DefaultTactic> defensiveTactic = std::make_shared<bt::DefaultTactic>("defensiveTactic");
    for (int i = 0; i < 11; i++) {
      std::string name = "o" + i;
        auto temp = bt::createDefenderRole(name);
        temp->giveProperty("ROLE", name);
        defensiveTactic->addChild(temp);
    }
    return defensiveTactic;
}

std::shared_ptr<bt::Role> bt::createDefenderRole(string rolename) {
    std::shared_ptr<bt::Role> role = std::make_shared<bt::Role>(rolename);
    std::shared_ptr<bt::Repeater> repeaterino = std::make_shared<bt::Repeater>();
    //std::shared_ptr<bt::Selector> selectorino = std::make_shared<bt::Selector>();

    std::shared_ptr<Attack> attackio = std::make_shared<Attack>("attack",  std::make_shared<bt::Blackboard>());

    // Make a repeater with the child "attack"
    repeaterino->addChild(attackio);
//    role->addChild(selectorino);
    role->addChild(repeaterino);
    //selectorino->addChild(repeaterino);
    std::cout << "creating defender role" << std::endl;
    return role;

}