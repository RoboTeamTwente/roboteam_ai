//
// Created by rolf on 10-10-18.
// Edited by baris on 23/10/18.
//

#include <gtest/gtest.h>
#include "../src/utilities/RobotDealer.h"
#include <string>

namespace ai = rtt::ai;

TEST(RobotDealerTest, RobotDealerTest) {
    // Create a RobotDealer instance and test names;
    std::string tacticA = "Tactic A";
    std::string tacticB = "Tactic B";
    std::string keeperTactic = "Keeper Tactic";
    // Initially no Robot should be claimed
    ASSERT_EQ(0, ai::RobotDealer::getClaimedRobots().size());

    //claim bot 1 for tactic A. NOTE that the keeper id is initialized as bot 0, so if you try to claim bot 0 it FAILS.
    //getClaimedRobots() does NOT count the keeper.
    ASSERT_TRUE(ai::RobotDealer::claimRobotForTactic(1, tacticA));
    std::vector<int> claimedBots = ai::RobotDealer::getClaimedRobots();
    ASSERT_EQ(1, ai::RobotDealer::getClaimedRobots().size());

    // bots 2 and 3 for tactic B
    std::vector<int> robot_ids;
    robot_ids.push_back(2);
    robot_ids.push_back(3);
    ASSERT_TRUE(ai::RobotDealer::claimRobotForTactic(robot_ids, tacticB));

    claimedBots = ai::RobotDealer::getClaimedRobots();
    ASSERT_EQ(3, claimedBots.size());

    //set Keeper ID to 4
    ai::RobotDealer::setKeeper(4);

    // Claim the keeper and check if it is correct.
    ai::RobotDealer::claimRobotForTactic(4, keeperTactic);
    claimedBots = ai::RobotDealer::getClaimedRobots();
    ASSERT_FALSE(ai::RobotDealer::getKeeperAvailable());
    ASSERT_EQ(4, ai::RobotDealer::getKeeper());

    ASSERT_TRUE(ai::RobotDealer::releaseRobot(3));

    std::map<std::string, std::set<int>> ownerList = ai::RobotDealer::getRobotOwnerList();
    ASSERT_EQ(ownerList[tacticB].find(3), ownerList[tacticB].end());
    ASSERT_NE(ownerList[tacticB].find(2), ownerList[tacticB].end());

    ASSERT_TRUE(ai::RobotDealer::claimRobotForTactic(3, tacticB));

    // a robot with an invalid id cannot be claimed for a tactic
    ASSERT_FALSE(ai::RobotDealer::claimRobotForTactic(30000, tacticB));

    // robot 3 cannot be claimed because it was already claimed
    ASSERT_FALSE(ai::RobotDealer::claimRobot(3));

    //release robots 2 and 3
    ASSERT_TRUE(ai::RobotDealer::releaseRobots(robot_ids));
    ASSERT_EQ(1, ai::RobotDealer::getClaimedRobots().size());

    // the keeper could not be taken because it was already claimed.
    ASSERT_FALSE(ai::RobotDealer::claimRobot(4));

    // a robot with an unknown id cannot be claimed
    ASSERT_FALSE(ai::RobotDealer::claimRobot(400000));

    // release keeper
    ai::RobotDealer::releaseRobot(4);
    ASSERT_TRUE(ai::RobotDealer::getKeeperAvailable());

    std::string newTactic = "aha a tactic";
    int newRobot = 5;

    ai::RobotDealer::claimRobotForTactic(newRobot, newTactic);
    ownerList = ai::RobotDealer::getRobotOwnerList();
    claimedBots = ai::RobotDealer::getClaimedRobots();
    ai::RobotDealer::releaseRobot(newRobot);

    ai::RobotDealer::haltOverride();

    ownerList = ai::RobotDealer::getRobotOwnerList();
    claimedBots = ai::RobotDealer::getClaimedRobots();

    ASSERT_TRUE(ownerList.empty());

    ASSERT_TRUE(ai::RobotDealer::claimRobots(robot_ids));
    ASSERT_EQ(ai::RobotDealer::getClaimedRobots().size(), 2);
}