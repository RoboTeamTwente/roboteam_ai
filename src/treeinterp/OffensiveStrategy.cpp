//
// Created by jesse on 14.10.19.
//
#include <include/roboteam_ai/treeinterp/PassRole.h>
#include <include/roboteam_ai/treeinterp/SideAttackerRole.h>
#include "bt/BehaviorTree.hpp"
#include "bt/decorators/Repeater.hpp"
#include "treeinterp/OffensiveStrategy.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/Role.h"
#include "skills/gotopos/GoToPos.h"
#include "skills/Attack.h"

namespace bt {

    OffensiveStrategy::OffensiveStrategy() {
        // Set the robottypes for the robot so the robotdealer can decide which robot should do what
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

    std::shared_ptr<BehaviorTree> OffensiveStrategy::createOffensiveStrategy() {
        std::shared_ptr<Blackboard> bb = std::make_shared<Blackboard>();
        // Set the tactictype so the robotdealer can divide the robots
        // Currently there are 2 options: General and (Offensive/Defensive/Midfield).
        bb->setString("TacticType", "General");

        std::shared_ptr<DefaultTactic> offensiveTactic = createOffensiveTactic(bb);
        offensiveTactic->setProperties(bb);

        auto tree = std::make_shared<BehaviorTree>("defendertree");
        tree->SetRoot(offensiveTactic);
        this->tree = tree;
        return tree;
    }


    std::shared_ptr<DefaultTactic> OffensiveStrategy::createOffensiveTactic(std::shared_ptr<Blackboard> bb) {
        // create a default tactic which will be used to build the offensive tactic
        std::shared_ptr<DefaultTactic> offensiveTactic = std::make_shared<DefaultTactic>("offensiveTactic", bb, robots);

        // Creating the roles for all the robots in the tactic:
        for (int i = 1; i < robots.size(); i++) {
            std::string name = "o" + std::to_string(i);
            auto temp = createOffenderRole(name);

            /// For testing purposes, this is changed here:
            auto temphelper = bt::SideAttackerRole();
            std::shared_ptr<Role> temprole = temphelper.createSideAttackerRole(name);
            auto ch = temprole->getChildren();

            offensiveTactic->addChild(temprole);
        }

        return offensiveTactic;
    }

    std::shared_ptr<Role> OffensiveStrategy::createOffenderRole(std::string name) {
        // set the rolename for the current role. This is important because the robotdealer decides how to deal the robots based on their rolenames
        // The property "ROLE" should be set to the name of the role, which should correspond to the rolename
        // found in the robotdealer robots vector

        auto localbb = std::make_shared<Blackboard>();
        localbb->setString("ROLE", name);
        std::shared_ptr<Role> role = std::make_shared<Role>(name);
        role->setProperties(localbb);

        // Make a repeater with the child "attack"
        std::shared_ptr<Repeater> repeater = std::make_shared<Repeater>();
        std::shared_ptr<rtt::ai::Attack> attack = std::make_shared<rtt::ai::Attack>("attack", localbb);

        repeater->addChild(attack);
        role->addChild(repeater);
        return role;

    }



}
