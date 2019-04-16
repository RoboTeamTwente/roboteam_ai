//
// Created by mrlukasbos on 11-1-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/bt/tactics/DefaultTactic.h>
#include <roboteam_ai/src/Switches.h>
#include <roboteam_ai/src/treeinterp/BTFactory.h>
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/src/analysis/GameAnalyzer.h>

namespace w = rtt::ai::world;
namespace rd = rtt::ai::robotDealer;

namespace bt {

TEST(DefaultTacticTest, default_general_tactic_works) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 9;
    field.field_length = 12;

    // set the world with 8 of our robots.
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(8, 0, false, field));

    // make sure game analyzer has data
    rtt::ai::analysis::GameAnalyzer::getInstance().start();
    rtt::ai::analysis::GameAnalyzer::getInstance().generateReportNow();

    rd::RobotDealer::refresh();

    // fake robots input as if it were from switches.cpp
    std::vector<std::pair<std::string, DefaultTactic::RobotType>> robots = {
            {"general1", DefaultTactic::RobotType::RANDOM},
            {"general2", DefaultTactic::RobotType::RANDOM},
            {"general3", DefaultTactic::RobotType::RANDOM},
            {"general4", DefaultTactic::RobotType::RANDOM},
            {"general5", DefaultTactic::RobotType::RANDOM},
            {"general6", DefaultTactic::RobotType::RANDOM},
            {"general7", DefaultTactic::RobotType::RANDOM},
            {"general8", DefaultTactic::RobotType::RANDOM}
    };

    auto properties = std::make_shared<bt::Blackboard>();
    properties->setString("TacticType", "General");

    DefaultTactic tactic("GeneralTactic", properties, robots);


    tactic.updateStyle();
    rd::RobotDealer::setUseSeparateKeeper(false);
    EXPECT_EQ(tactic.amountToTick, rtt::ai::world::world->getUs().size());

    rd::RobotDealer::setUseSeparateKeeper(true);
    tactic.updateStyle();
    EXPECT_EQ(tactic.amountToTick, rtt::ai::world::world->getUs().size() - 1);

    EXPECT_EQ(tactic.robots.size(), robots.size());

    // the first robot it should claim is general1
    EXPECT_EQ(tactic.getNextClaim().first, "general1");
    tactic.claimRobots(1); // claim 1 robot

    // the second robot should be general2, the last claim was general1
    EXPECT_EQ(tactic.getNextClaim().first, "general2");
    EXPECT_EQ(tactic.getLastClaim().first, "general1");
    tactic.disClaimRobots(1);
    EXPECT_EQ(tactic.getNextClaim().first, "general1");



    // after initialize it should have claimed all robots
    // NOTE that there is a keeper because  rd::RobotDealer::setUseSeparateKeeper(true);

    tactic.initialize();
    EXPECT_EQ(tactic.getLastClaim().first, "general7");

    // lets disclaim 3
    tactic.disClaimRobots(3);
    EXPECT_EQ(tactic.getLastClaim().first, "general4");
    EXPECT_EQ(tactic.getNextClaim().first, "general5");

    tactic.initialize();
    EXPECT_EQ(tactic.getLastClaim().first, "general7");


    }
}
