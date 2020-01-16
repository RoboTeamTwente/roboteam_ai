//
// Created by jessevw on 14.01.20.
//

#include <include/roboteam_ai/skills/CoachDefend.h>
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




    }

    // Select which robot to pass to
    bt::Node::Status PassAndPlayPlay::executePlay(world::World* world, world::Field* field) {
        tree = std::make_shared<bt::BehaviorTree>();
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

        auto localbb = std::make_shared<bt::Blackboard>();

        auto tactic = std::make_shared<bt::DefaultTactic>("Tactic trial", localbb, robots);

        localbb->setInt("PassTo", 3);
        auto passto = std::make_shared<rtt::ai::Pass2>("Pass to robot 3 node", localbb);
        tactic->addChild(passto);

        std::shared_ptr<bt::Blackboard> bb = std::make_shared<bt::Blackboard>();
        // Set the tactictype so the robotdealer can divide the robots
        bb->setString("TacticType", "General");
        tactic->setProperties(bb);

        tree->SetRoot(tactic);
        return tree->tick(world, field);
    }
}