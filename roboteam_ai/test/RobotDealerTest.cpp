//
// Created by rolf on 10-10-18.
// Edited by baris on 23/10/18.
//

#include <gtest/gtest.h>
#include "../src/utilities/OldRobotDealer.h"
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
    for (int id = 1; id < amountOfRobots+1; id++) {
        worldMsg.us.push_back(getRobot(10*id, 0, 0, id));
    }
    ai::World::set_world(worldMsg);
    ros::Rate rate(1);

    // Create a OldRobotDealer instance and test names;
    std::string tacticA = "Tactic A";
    std::string tacticB = "Tactic B";
    std::string keeperTactic = "Keeper Tactic";
    std::string roleA = "attack!";
    std::string roleB = "defend.";
    std::string roleK = "holdTheDoor";

    // Initially no Robot should be claimed
    ASSERT_EQ(0, ai::OldRobotDealer::getClaimedRobots().size());
    ASSERT_EQ(amountOfRobots, ai::OldRobotDealer::getAvailableRobots().size());

    int id = ai::OldRobotDealer::claimRandomRobot();
    ASSERT_EQ(id, *ai::OldRobotDealer::getClaimedRobots().begin());
    ai::OldRobotDealer::releaseRobot(id);

    ASSERT_EQ(ai::OldRobotDealer::findRobotForRole(roleA), -1);
    std::pair<int, std::string> idNameA = {1, roleA};
    ASSERT_TRUE(ai::OldRobotDealer::claimRobotForTactic(idNameA, tacticA));
    ASSERT_EQ(ai::OldRobotDealer::findRobotForRole(roleA), 1);

    std::set<int> claimedBots = ai::OldRobotDealer::getClaimedRobots();
    ASSERT_EQ(1, ai::OldRobotDealer::getClaimedRobots().size());

    // bots 2 and 3 for tactic B
    std::pair<int, std::string> idNameB2 = {2, roleA};
    std::pair<int, std::string> idNameB3 = {3, roleB};
    std::set<std::pair<int, std::string>> idNamePairsB = {idNameB3,idNameB2};

    ASSERT_TRUE(ai::OldRobotDealer::claimRobotForTactic(idNamePairsB, tacticB));

    claimedBots = ai::OldRobotDealer::getClaimedRobots();
    ASSERT_EQ(3, claimedBots.size());

    //set Keeper ID to 4


    // Claim the keeper and check if it is correct.
    ASSERT_FALSE(ai::OldRobotDealer::releaseKeeper());
    ai::OldRobotDealer::claimKeeper(4);
    std::pair<int, std::string> idNameKeeper = {4, roleK};
    ai::OldRobotDealer::claimRobotForTactic(idNameKeeper, keeperTactic);

    claimedBots = ai::OldRobotDealer::getClaimedRobots();
    ASSERT_FALSE(ai::OldRobotDealer::getKeeperAvailable());
    ASSERT_EQ(4, ai::OldRobotDealer::getKeeper());
    ASSERT_FALSE(ai::OldRobotDealer::claimRobot(4));
    ASSERT_TRUE(ai::OldRobotDealer::releaseRobot(3));

    ASSERT_TRUE(ai::OldRobotDealer::releaseRobot(4));

    auto ownerList = ai::OldRobotDealer::getRobotOwnerList();
    ASSERT_EQ(ownerList[tacticB].find(idNameB3), ownerList[tacticB].end());
    ASSERT_NE(ownerList[tacticB].find(idNameB2), ownerList[tacticB].end());

    std::pair<int, std::string> idNameC = {3, roleB};
    ASSERT_TRUE(ai::OldRobotDealer::claimRobotForTactic(idNameC, tacticB));

    // a robot with an invalid id cannot be claimed for a tactic
    std::pair<int, std::string> idNameFaulty = {30000, roleB};

    ASSERT_FALSE(ai::OldRobotDealer::claimRobotForTactic(idNameFaulty, tacticB));

    // robot 3 cannot be claimed because it was already claimed
    ASSERT_FALSE(ai::OldRobotDealer::claimRobot(3));

    //release robots 2 and 3
    ASSERT_EQ(3, ai::OldRobotDealer::getClaimedRobots().size());
    std::set<int> robot_ids = {2,3};
    ASSERT_TRUE(ai::OldRobotDealer::releaseRobot(robot_ids));
    ASSERT_EQ(1, ai::OldRobotDealer::getClaimedRobots().size());
    ASSERT_EQ(amountOfRobots-1, ai::OldRobotDealer::getAvailableRobots().size());

    // the keeper could not be taken because it was already claimed.
    ASSERT_TRUE(ai::OldRobotDealer::claimRobot(4));

    // a robot with an unknown id cannot be claimed
    ASSERT_FALSE(ai::OldRobotDealer::claimRobot(400000));

    // release keeper
    ASSERT_TRUE(ai::OldRobotDealer::getKeeperAvailable());
    ai::OldRobotDealer::releaseRobot(4);
    ASSERT_TRUE(ai::OldRobotDealer::getKeeperAvailable());

    std::string newTactic = "aha a tactic!";
    std::string newRole = "wow a role!";
    int newRobot = 5;

    std::pair<int, std::string> newPair = {newRobot, newRole};
    ai::OldRobotDealer::claimRobotForTactic(newPair, newTactic);
    ownerList = ai::OldRobotDealer::getRobotOwnerList();
    claimedBots = ai::OldRobotDealer::getClaimedRobots();

    ASSERT_FALSE(ai::OldRobotDealer::claimRobotForTactic(newPair, newTactic));

    ai::OldRobotDealer::releaseRobot(newRobot);

    ai::OldRobotDealer::haltOverride();

    ownerList = ai::OldRobotDealer::getRobotOwnerList();
    claimedBots = ai::OldRobotDealer::getClaimedRobots();

    ASSERT_TRUE(ownerList.empty());

    // test for invalid robot ids

    std::pair<int, std::string> pairWithBadID = {-1, roleA};
    ASSERT_FALSE(ai::OldRobotDealer::claimRobotForTactic(pairWithBadID, newTactic));
    ASSERT_FALSE(ai::OldRobotDealer::releaseRobot(-1));

    // test for unclaimed robots
    ASSERT_FALSE(ai::OldRobotDealer::releaseRobot(6));

    // test for multiple robots
    std::set<int> robots{7, 8, 9};
    ASSERT_TRUE(ai::OldRobotDealer::claimRobot(robots));
    ASSERT_FALSE(ai::OldRobotDealer::claimRobot(robots));
    robots = {6, 7};
    ASSERT_FALSE(ai::OldRobotDealer::claimRobot(robots));

    ai::OldRobotDealer::haltOverride();    //nuke

    //TODO: make these functions/make these tests
    ASSERT_EQ(-1, ai::OldRobotDealer::claimRobotClosestToPoint({0,0}));

    std::string tac42 = "yes";
    std::string role42 = "42?";
    std::string role43 = "43!";

    std::pair<int, std::string> idRole42 = {3, role42};

    ai::OldRobotDealer::claimRobotForTactic(idRole42, tac42);

    ASSERT_EQ(-1, ai::OldRobotDealer::findRobotForRole(tac42, role43));

    // claim all robots except 1..
    for (int i = 0; i < amountOfRobots-2; i++) {
        ai::OldRobotDealer::claimRandomRobot();
    }

    // try to claim the last robot + 1
    ASSERT_NE(-1, ai::OldRobotDealer::claimRandomRobot());
    ASSERT_EQ(-1, ai::OldRobotDealer::claimRandomRobot());

}