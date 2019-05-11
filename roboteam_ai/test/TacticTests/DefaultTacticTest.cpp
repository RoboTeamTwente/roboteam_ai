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

    // the type should be defaulted to 'general'

    tactic.updateStyle();
    rd::RobotDealer::setUseSeparateKeeper(false);
    EXPECT_EQ(tactic.amountToTick, static_cast<int>(rtt::ai::world::world->getUs().size()));

    rd::RobotDealer::setUseSeparateKeeper(true);
    rd::RobotDealer::setKeeperID(-1); // set the keeper id to -1 to mimick a keeper that is not seen by cameras
    tactic.updateStyle();
    EXPECT_EQ(tactic.amountToTick, static_cast<int>(rtt::ai::world::world->getUs().size()));

    // with a visible keeper we should subtract one robot to tick
    // the first robot in our world is always visible so we always make that one the keeper
    rd::RobotDealer::setKeeperID(rtt::ai::world::world->getUs().at(0).id);
    tactic.updateStyle();
    EXPECT_EQ(tactic.amountToTick, static_cast<int>(rtt::ai::world::world->getUs().size() - 1));

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

    // now without a keeper: it should also claim the last robot
    rd::RobotDealer::setKeeperID(-1);
    rd::RobotDealer::setUseSeparateKeeper(false);

    tactic.updateStyle();
    tactic.initialize();
    EXPECT_EQ(tactic.getLastClaim().first, "general8");
}

TEST(DefaultTacticTest, offensive_defensive_midfield_tactics_work) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 9;
    field.field_length = 12;

    // set the world with 8 of our robots.
    w::world->updateWorld(testhelpers::WorldHelper::getWorldMsg(8, 0, false, field));

    // make sure game analyzer has data
    rtt::ai::analysis::GameAnalyzer::getInstance().start();
    auto report = rtt::ai::analysis::GameAnalyzer::getInstance().generateReportNow();
    rd::RobotDealer::refresh();

    // fake robots input as if it were from switches.cpp
    std::vector<std::pair<std::string, DefaultTactic::RobotType>> defensiveRobots = {
            {"def1", DefaultTactic::RobotType::RANDOM},
            {"def2", DefaultTactic::RobotType::RANDOM},
            {"def3", DefaultTactic::RobotType::RANDOM},
            {"def4", DefaultTactic::RobotType::RANDOM},
            {"def5", DefaultTactic::RobotType::RANDOM},
            {"def6", DefaultTactic::RobotType::RANDOM},
            {"def7", DefaultTactic::RobotType::RANDOM},
            {"def8", DefaultTactic::RobotType::RANDOM}
    };

    // fake robots input as if it were from switches.cpp
    std::vector<std::pair<std::string, DefaultTactic::RobotType>> midfieldRobots = {
            {"mid1", DefaultTactic::RobotType::RANDOM},
            {"mid2", DefaultTactic::RobotType::RANDOM},
            {"mid3", DefaultTactic::RobotType::RANDOM},
            {"mid4", DefaultTactic::RobotType::RANDOM},
            {"mid5", DefaultTactic::RobotType::RANDOM},
            {"mid6", DefaultTactic::RobotType::RANDOM},
            {"mid7", DefaultTactic::RobotType::RANDOM},
            {"mid8", DefaultTactic::RobotType::RANDOM}
    };

    // fake robots input as if it were from switches.cpp
    std::vector<std::pair<std::string, DefaultTactic::RobotType>> offensiveRobots = {
            {"att1", DefaultTactic::RobotType::RANDOM},
            {"att2", DefaultTactic::RobotType::RANDOM},
            {"att3", DefaultTactic::RobotType::RANDOM},
            {"att4", DefaultTactic::RobotType::RANDOM},
            {"att5", DefaultTactic::RobotType::RANDOM},
            {"att6", DefaultTactic::RobotType::RANDOM},
            {"att7", DefaultTactic::RobotType::RANDOM},
            {"att8", DefaultTactic::RobotType::RANDOM}
    };

    auto defensiveProperties = std::make_shared<bt::Blackboard>();
    defensiveProperties->setString("TacticType", "Defensive");
    DefaultTactic defensiveTactic("Defensivetactic", defensiveProperties, defensiveRobots);

    auto midfieldProperties = std::make_shared<bt::Blackboard>();
    midfieldProperties->setString("TacticType", "Middle");
    DefaultTactic midfieldTactic("MidfieldTactic", midfieldProperties, midfieldRobots);

    auto offensiveProperties = std::make_shared<bt::Blackboard>();
    offensiveProperties->setString("TacticType", "Offensive");
    DefaultTactic offensiveTactic("OffensiveTactic", offensiveProperties, offensiveRobots);

    // Check the names
    EXPECT_EQ(defensiveTactic.node_name(), "Defensivetactic");
    EXPECT_EQ(midfieldTactic.node_name(), "MidfieldTactic");
    EXPECT_EQ(offensiveTactic.node_name(), "OffensiveTactic");

    // get the playstyle from the decision maker
    rtt::ai::analysis::DecisionMaker maker;
    auto style = maker.getRecommendedPlayStyle(report->ballPossession);

    defensiveTactic.initialize();
    midfieldTactic.initialize();
    offensiveTactic.initialize();

    // the amounts to tick should be correct
    EXPECT_EQ(defensiveTactic.amountToTick, style.amountOfDefenders);
    EXPECT_EQ(midfieldTactic.amountToTick, style.amountOfMidfielders);
    EXPECT_EQ(offensiveTactic.amountToTick, style.amountOfAttackers);

    // the amounts of robotIds after initialize should be correct
    EXPECT_EQ(static_cast<int>(defensiveTactic.robotIDs.size()), style.amountOfDefenders);
    EXPECT_EQ(static_cast<int>(midfieldTactic.robotIDs.size()), style.amountOfMidfielders);
    EXPECT_EQ(static_cast<int>(offensiveTactic.robotIDs.size()), style.amountOfAttackers);

    rtt::ai::analysis::GameAnalyzer::getInstance().stop();

    }

} // bt
