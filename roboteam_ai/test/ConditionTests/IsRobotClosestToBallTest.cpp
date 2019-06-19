//
// Created by robzelluf on 10/22/18.
//

#include <gtest/gtest.h>
#include <roboteam_msgs/World.h>
#include "../../src/world/World.h"
#include "../../src/utilities/RobotDealer.h"
#include "../../src/conditions/IsRobotClosestToBall.h"
#include "../../src/world/Ball.h"

TEST(IsRobotClosestToBallTest, NoSecondsAhead) {

    rtt::ai::world::Ball::hasBeenSeen = false;
    auto BB = std::make_shared<bt::Blackboard>();
    BB->setInt("ROBOT_ID", 0);
    BB->setString("ROLE","test");
    rtt::ai::IsRobotClosestToBall node("IsRobotClosestToBall", BB);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(node.node_name(), "IsRobotClosestToBall");

    // First test should fail since robot is not set in world state yet
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);

    worldMsg.ball.pos.x=1.0;
    worldMsg.ball.pos.y=1.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.area = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);

    rtt::ai::robotDealer::RobotDealer::halt();

    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(
            rtt::ai::robotDealer::RobotType::RANDOM, "test", "IsRobotClosestToBallTestTactic");

    // Test should succeed because one robot is always closest to the ball
    node.initialize();
    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    roboteam_msgs::WorldRobot robot2;

    robot2.id = 1;
    robot2.pos.x=0.5;
    robot2.pos.y=0.5;
    worldMsg.us.push_back(robot2);
    rtt::ai::world::world->updateWorld(worldMsg);

    // Test should fail since robot 2 is no longer closest to the ball
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);
    rtt::ai::robotDealer::RobotDealer::removeTactic("IsRobotClosestToBallTestTactic");
}

TEST(IsRobotClosestToBallTest, secondsAhead) {
    bt::Blackboard BB;
    BB.setInt("ROBOT_ID", 0);
    BB.setString("ROLE","test");
    BB.setDouble("secondsAhead", 3.0);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsRobotClosestToBall node("Test", BBpointer);

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
    worldMsg.ball.area = 99999;
    rtt::ai::world::world->updateWorld(worldMsg, false);
    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(
            rtt::ai::robotDealer::RobotType::RANDOM, "test", "IsRobotClosestToBallTestTactic");

    node.initialize();
    ASSERT_EQ(node.update(), bt::Node::Status::Success);
    rtt::ai::robotDealer::RobotDealer::removeTactic("IsRobotClosestToBallTestTactic");
}