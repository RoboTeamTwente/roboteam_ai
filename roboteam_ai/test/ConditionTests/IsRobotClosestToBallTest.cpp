//
// Created by robzelluf on 10/22/18.
//

#include <gtest/gtest.h>
#include "../../src/bt/bt.hpp"
#include "roboteam_msgs/World.h"
#include "../../src/world/World.h"
#include "../../src/utilities/RobotDealer.h"
#include "../../src/conditions/IsRobotClosestToBall.h"

TEST(NoSecondsAhead, IsRobotClosestToBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 0);
    BB->setString("ROLE","test");
    rtt::ai::IsRobotClosestToBall Node("Test", BB);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(Node.node_name(), "IsRobotClosestToBall");

    // First test should fail since robot is not set in world state yet
    ASSERT_EQ(Node.update(), bt::Node::Status::Waiting);



    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);

    worldMsg.ball.pos.x=1.0;
    worldMsg.ball.pos.y=1.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);

    rtt::ai::robotDealer::robotDealer->claimRobotForTactic(rtt::ai::robotDealer::RobotType::RANDOM,"IsRobotClosestToBallTestTactic","test");
    // Test should succeed because one robot is always closest to the ball
    ASSERT_EQ(Node.update(), bt::Node::Status::Success);

    roboteam_msgs::WorldRobot robot2;

    robot2.id = 1;
    robot2.pos.x=0.5;
    robot2.pos.y=0.5;
    worldMsg.us.push_back(robot2);
    rtt::ai::world::world->updateWorld(worldMsg);

    // Test should fail since robot 2 is no longer closest to the ball
    ASSERT_EQ(Node.update(), bt::Node::Status::Failure);
    rtt::ai::robotDealer::robotDealer->removeTactic("IsRobotClosestToBallTestTactic");
}

TEST(secondsAhead, IsRobotClosestToBallTest) {
    bt::Blackboard BB;
    BB.setInt("ROBOT_ID", 0);
    BB.setString("ROLE","test");
    BB.setDouble("secondsAhead", 3.0);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsRobotClosestToBall Node("Test", BBpointer);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    roboteam_msgs::WorldRobot robot2;

    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);

    robot2.id=1;
    robot2.pos.x=1;
    robot2.pos.y=1;
    worldMsg.us.push_back(robot2);

    worldMsg.ball.pos.x=2.0;
    worldMsg.ball.pos.y=2.0;
    worldMsg.ball.vel.x = -1;
    worldMsg.ball.vel.y = -1;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::robotDealer::robotDealer->claimRobotForTactic(rtt::ai::robotDealer::RobotType::RANDOM,"IsRobotClosestToBallTestTactic","test");
    ASSERT_EQ(Node.update(), bt::Node::Status::Success);
    rtt::ai::robotDealer::robotDealer->removeTactic("IsRobotClosestToBallTestTactic");
}