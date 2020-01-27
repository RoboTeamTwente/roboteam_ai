//
// Created by jessevw on 14.01.20.
//

#include <include/roboteam_ai/bt/Role.h>
#include <include/roboteam_ai/skills/Halt.h>
#include <include/roboteam_ai/skills/Attack.h>
#include <include/roboteam_ai/bt/Node.hpp>
#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h>
#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h>
#include <include/roboteam_ai/bt/composites/Sequence.hpp>
#include "analysis/PlaysObjects/PassAndPlayPlay.h"
#include "skills/NewPass.h"
#include "bt/Blackboard.hpp"
#include "bt/tactics/DefaultTactic.h"
#include "skills/Receive.h"
#include "skills/KickTo.h"
#include "skills/gotopos/SkillGoToPos.h"

namespace rtt::ai::analysis {
    /// make tactic and execute this
    /**
     * The play will consist of multiple strategies which in turn consist of multiple roles. We build the tree in the constructor for now
     */
    PassAndPlayPlay::PassAndPlayPlay(std::string name) : Play(name) {
        tree1 = std::make_shared<bt::BehaviorTree>();
        tree2 = std::make_shared<bt::BehaviorTree>();
        makeTree1();
        makeTree2();
        tree = tree1;
        tree1->properties->setString("NAME", "FIRST");
//        tree2->properties->setString("NAME", "SECOND");
//        tree1->setNext(tree2);

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
            if (tree->getNext()) {
                tree = tree->getNext();
            }
        }
    }

    int PassAndPlayPlay::scorePlay(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        int score = 0;
        if (BallBelongsToUsInvariant::isValid(world, field)) {
            score = 10;
        }
        return score;
    }

    bool PassAndPlayPlay::isValidPlay(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        return BallOnOurSideInvariant::isValid(world, field)  || true;
    }

    void PassAndPlayPlay::makeTree2() {
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
        std::shared_ptr<bt::DefaultTactic> tactic = std::make_shared<bt::DefaultTactic>("tree2", bb, robots);

        // Creating the roles for all the robots in the tactic:
        for (int i = 1; i < robots.size(); i++) {
            auto localbb = std::make_shared<bt::Blackboard>();
            std::string rolename = "o" + std::to_string(i);
            localbb->setString("ROLE", rolename);
            std::shared_ptr<bt::Role> temprole2 = std::make_shared<bt::Role>(rolename);

            if (i == 5) {
                auto pass = std::make_shared<rtt::ai::Attack>("Attack2", localbb);
                pass->properties->setInt("PassTo", 2);
                temprole2->addChild(pass);
            }
            else {
                auto halt = std::make_shared<rtt::ai::Halt>("halt2", localbb);
                temprole2->addChild(halt);
            }

            temprole2->setRoleString(rolename);
            tactic->addChild(temprole2);
        }
        tree2->SetRoot(tactic);
    }

    void PassAndPlayPlay::makeTree1() {
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
        std::shared_ptr<bt::DefaultTactic> tactic = std::make_shared<bt::DefaultTactic>("tactic", bb, robots);

        // Creating the roles for all the robots in the tactic:
        for (int i = 1; i < robots.size(); i++) {
            auto localb = std::make_shared<bt::Blackboard>();
            std::string rolename = "o" + std::to_string(i);
            localb->setString("ROLE", rolename);
            std::shared_ptr<bt::Role> temprole = std::make_shared<bt::Role>(rolename);

            if (i == 5) {
                auto pass = std::make_shared<rtt::ai::NewPass>("pass", localb);
                pass->properties->setInt("PassTo", 2);
                auto s = std::make_shared<bt::Sequence>();
                s->addChild(pass);
                auto posbb = std::make_shared<bt::Blackboard>();
                posbb->setString("goToType", "numTrees");
                posbb->setVector2("targetPos", Vector2(2,2));
                auto gtp = std::make_shared<rtt::ai::SkillGoToPos>("go to pos 2,2", posbb);
                s->addChild(gtp);
                temprole->addChild(s);
            }
            else if (i == 2) {
                auto receive = std::make_shared<rtt::ai::Receive>("receive", localb);
                localb->setVector2("where", Vector2(2, 2));
                auto shootAtPoint = std::make_shared<rtt::ai::KickTo>("shoot at point", localb);
                auto s = std::make_shared<bt::Sequence>();
                temprole->addChild(s);
                s->addChild(receive);
                s->addChild(shootAtPoint);
            }
            else {
                auto halt = std::make_shared<rtt::ai::Halt>("halt", localb);
                temprole->addChild(halt);
            }

            temprole->setRoleString(rolename);
            tactic->addChild(temprole);
        }
        tree1->SetRoot(tactic);

    }


}