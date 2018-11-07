//
// Created by rolf on 10-10-18.
// Edited by baris on 23/10/18.
//

#include <gtest/gtest.h>
#include "../src/utilities/RobotDealer.h"
#include "../src/utilities/World.h"

namespace ai = rtt::ai;

roboteam_msgs::WorldRobot getRobot(int x, int y, float angle, int id = 0) {
    roboteam_msgs::WorldRobot robot;
    robot.pos = rtt::Vector2(x, y);
    robot.id = (unsigned int) id;
    robot.angle = angle;
    return robot;
}

TEST(RobotDealerTest, RobotDealerTest) {

    roboteam_msgs::World worldMsg;
    int amountOfRobots = 11;
    for (int id = 1; id < amountOfRobots; id++) {
        worldMsg.us.push_back(getRobot(10*id, 0, 0, id));
    }
    ai::World::set_world(worldMsg);
    ros::Rate rate(1);

    // Create a RobotDealer instance and test names;
    std::string tacticA = "Tactic A";
    std::string tacticB = "Tactic B";
    std::string keeperTactic = "Keeper Tactic";
    std::string roleA = "attack!";
    std::string roleB = "defend.";
    std::string roleK = "holdTheDoor";

    // Initially no Robot should be claimed
    ASSERT_EQ(0, ai::RobotDealer::getClaimedRobots().size());

    //claim bot 1 for tactic A. NOTE that the keeper id is initialized as bot 0, so if you try to claim bot 0 it FAILS.
    //getClaimedRobots() does NOT count the keeper.
    ASSERT_TRUE(ai::RobotDealer::claimRobotForTactic(1, tacticA, roleA));
    std::set<int> claimedBots = ai::RobotDealer::getClaimedRobots();
    ASSERT_EQ(1, ai::RobotDealer::getClaimedRobots().size());

    // bots 2 and 3 for tactic B
    std::set<int> robot_ids;
    robot_ids.insert(2);
    robot_ids.insert(3);
    ASSERT_TRUE(ai::RobotDealer::claimRobotForTactic(robot_ids, tacticB, roleB));

    claimedBots = ai::RobotDealer::getClaimedRobots();
    ASSERT_EQ(3, claimedBots.size());

    //set Keeper ID to 4
    ASSERT_FALSE(ai::RobotDealer::releaseKeeper());
    ai::RobotDealer::claimKeeper(4);

    // Claim the keeper and check if it is correct.
    ai::RobotDealer::claimRobotForTactic(4, keeperTactic, roleK);
    claimedBots = ai::RobotDealer::getClaimedRobots();
    ASSERT_FALSE(ai::RobotDealer::getKeeperAvailable());
    ASSERT_EQ(4, ai::RobotDealer::getKeeper());
    ASSERT_FALSE(ai::RobotDealer::claimRobot(4));
    ASSERT_TRUE(ai::RobotDealer::releaseRobot(3));

    ASSERT_TRUE(ai::RobotDealer::releaseRobot(4));

    std::map<std::string, std::set<std::pair<int,std::string>>> ownerList = ai::RobotDealer::getRobotOwnerList();
    std::pair<int,std::string> B3 = {3, roleB}, B2 = {2, roleB};
    ASSERT_EQ(ownerList[tacticB].find(B3), ownerList[tacticB].end());
    ASSERT_NE(ownerList[tacticB].find(B2), ownerList[tacticB].end());

    ASSERT_TRUE(ai::RobotDealer::claimRobotForTactic(3, tacticB, roleB));

    // a robot with an invalid id cannot be claimed for a tactic
    ASSERT_FALSE(ai::RobotDealer::claimRobotForTactic(30000, tacticB, roleB));

    // robot 3 cannot be claimed because it was already claimed
    ASSERT_FALSE(ai::RobotDealer::claimRobot(3));

    //release robots 2 and 3
    ASSERT_TRUE(ai::RobotDealer::releaseRobot(robot_ids));
    ASSERT_EQ(1, ai::RobotDealer::getClaimedRobots().size());
    // the keeper could not be taken because it was already claimed.
    ASSERT_TRUE(ai::RobotDealer::claimRobot(4));

    // a robot with an unknown id cannot be claimed
    ASSERT_FALSE(ai::RobotDealer::claimRobot(400000));

    // release keeper
    ASSERT_TRUE(!ai::RobotDealer::getKeeperAvailable());
    ai::RobotDealer::releaseRobot(4);
    ASSERT_TRUE(ai::RobotDealer::getKeeperAvailable());

    std::string newTactic = "aha a tactic";
    int newRobot = 5;

    ai::RobotDealer::claimRobotForTactic(newRobot, newTactic, roleA);
    ownerList = ai::RobotDealer::getRobotOwnerList();
    claimedBots = ai::RobotDealer::getClaimedRobots();

    ASSERT_FALSE(ai::RobotDealer::claimRobotForTactic(newRobot, newTactic, roleB));

    ai::RobotDealer::releaseRobot(newRobot);

    ai::RobotDealer::haltOverride();

    ownerList = ai::RobotDealer::getRobotOwnerList();
    claimedBots = ai::RobotDealer::getClaimedRobots();

    ASSERT_TRUE(ownerList.empty());

    // test for invalid robot ids
    ASSERT_FALSE(ai::RobotDealer::claimRobotForTactic(-1, newTactic, roleA));
    ASSERT_FALSE(ai::RobotDealer::releaseRobot(-1));

    // test for unclaimed robots
    ASSERT_FALSE(ai::RobotDealer::releaseRobot(6));

    // test for multiple robots
    std::set<int> robots{7, 8, 9};
    ASSERT_TRUE(ai::RobotDealer::claimRobot(robots));
    ASSERT_FALSE(ai::RobotDealer::claimRobot(robots));
    robots = {6, 7};
    ASSERT_FALSE(ai::RobotDealer::claimRobot(robots));

}