//
// Created by jessevw on 14.01.20.
//

#include "skills/Halt.h"
#include "skills/Receive.h"
#include "skills/KickTo.h"
#include "skills/gotopos/SkillGoToPos.h"
#include "skills/Pass.h"
#include "analysis/play-utilities/invariants/BallBelongsToUsInvariant.h"
#include "analysis/play-utilities/invariants/BallOnOurSideInvariant.h"
#include "analysis/play-utilities/Plays/PassAndPlayPlay.h"
#include "bt/Role.h"
#include "bt/tactics/DefaultTactic.h"
#include "bt/composites/Sequence.hpp"
#include "bt/Blackboard.hpp"
#include "bt/BehaviorTree.hpp"

namespace rtt::ai::analysis {
    PassAndPlayPlay::PassAndPlayPlay(std::string name) : Play(name) {
        tree = std::make_shared<bt::BehaviorTree>();
        makeTree();
    }

    void PassAndPlayPlay::executePlay(world::World *world, world::Field *field) {

    }

    bool PassAndPlayPlay::isValidPlayToKeep(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        return true;
    }

    bool PassAndPlayPlay::isValidPlayToStart(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        return BallOnOurSideInvariant::isValid(world, field);
    }


    uint8_t PassAndPlayPlay::scorePlay(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        int score = 0;
        if (BallBelongsToUsInvariant::isValid(world, field)) {
            score = 10;
        }
        return score;
    }

    /**
     * The numbers in this function are magic by design :D
     */
    void PassAndPlayPlay::makeTree() {
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
                auto pass = std::make_shared<rtt::ai::Pass>("pass", localb);
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
        tree->SetRoot(tactic);

    }


}