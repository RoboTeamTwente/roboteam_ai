//
// Created by jessevw on 14.01.20.
//

#include <include/roboteam_ai/skills/CoachDefend.h>
#include <include/roboteam_ai/bt/Role.h>
#include <include/roboteam_ai/skills/Halt.h>
#include <include/roboteam_ai/skills/Attack.h>
#include <include/roboteam_ai/bt/Node.hpp>
#include "analysis/PlaysObjects/PassAndPlayPlay.h"
#include "skills/Pass2.h"
#include "bt/Blackboard.hpp"
#include "bt/tactics/DefaultTactic.h"


namespace rtt::ai::analysis {
    /// make tactic and execute this
    /**
     * The play will consist of multiple strategies which in turn consist of multiple roles. We build the tree in the constructor for now
     */
    PassAndPlayPlay::PassAndPlayPlay() {
        tree1 = std::make_shared<bt::BehaviorTree>();
        auto bb = std::make_shared<bt::Blackboard>();
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
        std::shared_ptr<bt::DefaultTactic> offensiveTactic = std::make_shared<bt::DefaultTactic>("offensiveTactic", bb, robots);

        // Creating the roles for all the robots in the tactic:
        for (int i = 1; i < robots.size(); i++) {
            auto localb = std::make_shared<bt::Blackboard>();
            std::string rolename = "o" + std::to_string(i);
            localb->setString("ROLE", rolename);
            std::shared_ptr<bt::Role> temprole = std::make_shared<bt::Role>(rolename);

            if (i == 5) {
                auto pass = std::make_shared<rtt::ai::Pass2>("pass", localb);
                pass->properties->setInt("PassTo", 2);
                temprole->addChild(pass);
            }
            else {
                auto halt = std::make_shared<rtt::ai::Halt>("pass", localb);
                temprole->addChild(halt);
            }

            temprole->setRoleString(rolename);
            offensiveTactic->addChild(temprole);
        }
        tree1->SetRoot(offensiveTactic);

        /// TREE 2
        tree2 = std::make_shared<bt::BehaviorTree>();
        auto bb2 = std::make_shared<bt::Blackboard>();
        std::vector<std::pair<std::string, rtt::ai::robotDealer::RobotType>> robots2 = {
                {"o1", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o2", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o3", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o4", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o5", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o6", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o7", rtt::ai::robotDealer::RobotType::RANDOM},
                {"o8", rtt::ai::robotDealer::RobotType::RANDOM}
        };
        std::shared_ptr<bt::DefaultTactic> offensiveTactic2 = std::make_shared<bt::DefaultTactic>("offensiveTactic", bb, robots);

        // Creating the roles for all the robots in the tactic:
        for (int i = 1; i < robots.size(); i++) {
            auto localb = std::make_shared<bt::Blackboard>();
            std::string rolename = "o" + std::to_string(i);
            localb->setString("ROLE", rolename);
            std::shared_ptr<bt::Role> temprole2 = std::make_shared<bt::Role>(rolename);

            if (i == 5) {
                auto pass = std::make_shared<rtt::ai::Attack>("Attack2", localb);
                pass->properties->setInt("PassTo", 2);
                temprole2->addChild(pass);
            }
            else {
                auto halt = std::make_shared<rtt::ai::Halt>("halt2", localb);
                temprole2->addChild(halt);
            }

            temprole2->setRoleString(rolename);
            offensiveTactic2->addChild(temprole2);
        }
        tree2->SetRoot(offensiveTactic2);
        tree = tree1;
        tree1->properties->setString("NAME", "FIRST");
        tree2->properties->setString("NAME", "SECOND");

        tree1->setNext(tree2);

    }

    void PassAndPlayPlay::executePlay(world::World *world, world::Field *field) {

    }


    std::shared_ptr<bt::BehaviorTree> PassAndPlayPlay::getTreeForWorld() {
        moveToNextTactic();
        return tree;
    }

    void PassAndPlayPlay::moveToNextTactic() {
        if (tree->getStatus() == bt::BehaviorTree::Status::Success) {
            std::cout << "moving to next tactic!" << std::endl;
            tree = tree->getNext();
        }
    }


}