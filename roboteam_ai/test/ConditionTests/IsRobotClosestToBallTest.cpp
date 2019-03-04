//
// Created by robzelluf on 10/22/18.
//

#include <gtest/gtest.h>
#include "../../src/bt/bt.hpp"
#include "roboteam_msgs/World.h"
#include "../../src/utilities/World.h"
#include "../../src/utilities/RobotDealer.h"
#include "../../src/conditions/IsRobotClosestToBall.h"

TEST(NoSecondsAhead, IsRobotClosestToBallTest) {
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 0);
    BB->setString("ROLE","test");
    rtt::ai::IsRobotClosestToBall Node("Test", BB);

    EXPECT_EQ(Node.node_name(), "IsRobotClosestToBall");

    // First test should fail since robot is not set in world state yet
    ASSERT_EQ(Node.update(), bt::Node::Status::Failure);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);

    worldMsg.ball.pos.x=1.0;
    worldMsg.ball.pos.y=1.0;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);

    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::random,"IsRobotClosestToBallTestTactic","test");
    // Test should succeed because one robot is always closest to the ball
    ASSERT_EQ(Node.update(), bt::Node::Status::Success);

    roboteam_msgs::WorldRobot robot2;

    robot2.id = 1;
    robot2.pos.x=0.5;
    robot2.pos.y=0.5;
    worldMsg.us.push_back(robot2);
    rtt::ai::World::set_world(worldMsg);

    // Test should fail since robot 2 is no longer closest to the ball
    ASSERT_EQ(Node.update(), bt::Node::Status::Failure);
    robotDealer::RobotDealer::removeTactic("IsRobotClosestToBallTestTactic");
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

    rtt::ai::World::set_world(worldMsg);
    robotDealer::RobotDealer::claimRobotForTactic(robotDealer::RobotType::random,"IsRobotClosestToBallTestTactic","test");
    ASSERT_EQ(Node.update(), bt::Node::Status::Success);
    robotDealer::RobotDealer::removeTactic("IsRobotClosestToBallTestTactic");
}