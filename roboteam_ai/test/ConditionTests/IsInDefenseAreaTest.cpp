//
// Created by rolf on 17-10-18.
//
#include <gtest/gtest.h>
#include "../../src/conditions/IsInDefenseArea.hpp"
#include "../../src/world/World.h"
#include "../../src/world/Ball.h"
#include "roboteam_ai/src/world/Field.h"
#include "../../src/utilities/RobotDealer.h"

TEST(DetectsInOurDefenseArea, IsInDefenseAreaTest)
{
    rtt::ai::robotDealer::RobotDealer::halt();

    bt::Blackboard BB;
    BB.setBool("useRobot", true);
    BB.setInt("ROBOT_ID", 0);
    BB.setString("ROLE", "test");
    BB.setDouble("margin", 0.2);
    BB.setBool("ourDefenseArea", true);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInDefenseArea node("IsInDefenseArea", BBpointer);

    EXPECT_EQ(node.node_name(), "IsInDefenseArea");
    rtt::ai::world::Ball::exists = false;

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    rtt::ai::world::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 8;
    field.field_length = 12;

    field.left_penalty_line.begin.x = -1.0f;
    field.left_penalty_line.end.x = -1.0f;

    field.left_penalty_line.begin.y = -1.0f;
    field.left_penalty_line.end.y = 1.0;

    field.right_penalty_line.begin.x = 1.0;
    field.right_penalty_line.end.x = 1.0;

    field.right_penalty_line.begin.y = -1.0f;
    field.right_penalty_line.end.y = 1.0;

    rtt::ai::world::field->set_field(field);

    robot.id = 0;
    robot.pos.x = -1.2;
    robot.pos.y = 0;

    worldMsg.us.push_back(robot);
    worldMsg.ball.existence = 99999;
    worldMsg.ball.pos.x = 0;
    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(rtt::ai::robotDealer::RobotType::RANDOM, "test", "IsInDefenseAreaTest");
    node.initialize();

    // Should succeed since robot is in our defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.us[0].pos.x = -0.7;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should fail since robot is not in out defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.us[0].pos.x = -0.9;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should succeed since robot is within the margin of our defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Success);
    rtt::ai::robotDealer::RobotDealer::removeTactic("IsInDefenseAreaTest");
}

TEST(DetectsInTheirDefenseArea, IsInDefenseAreaTest)
{
    rtt::ai::world::Ball::exists = false;
    bt::Blackboard BB;
    BB.setBool("useRobot", true);
    BB.setInt("ROBOT_ID", 0);
    BB.setString("ROLE", "test");
    BB.setDouble("margin", 0.2);
    BB.setBool("ourDefenseArea", false);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInDefenseArea node("IsInDefenseArea", BBpointer);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    robot.id = 0;
    robot.pos.x = 1.2;
    robot.pos.y = 0;

    worldMsg.us.push_back(robot);

    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 12;
    field.field_length = 8;
    field.left_penalty_line.begin.x = -1.0f;
    field.left_penalty_line.end.x = -1.0f;

    field.left_penalty_line.begin.y = -1.0f;
    field.left_penalty_line.end.y = 1.0;

    field.right_penalty_line.begin.x = 1.0;
    field.right_penalty_line.end.x = 1.0;

    field.right_penalty_line.begin.y = -1.0f;
    field.right_penalty_line.end.y = 1.0;

    rtt::ai::world::field->set_field(field);
    worldMsg.ball.existence = 99999;
    worldMsg.ball.visible = 1;

    rtt::ai::world::world->updateWorld(worldMsg);
    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(rtt::ai::robotDealer::RobotType::RANDOM, "test", "IsInDefenseAreaTest");
    node.initialize();

    // Should succeed since robot is in their defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.us[0].pos.x = 0.7;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should fail since robot is not in their defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.us[0].pos.x = 0.9;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should succeed since robot is within the margin of their defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Success);
    rtt::ai::robotDealer::RobotDealer::removeTactic("IsInDefenseAreaTest");
}

TEST(DetectsBallInOurDefenceArea, IsInDefenceAreaTest)
{
    rtt::ai::world::Ball::exists = false;
    bt::Blackboard BB;
    BB.setBool("useRobot", false);
    BB.setDouble("margin", 0.2);
    BB.setBool("ourDefenseArea", true);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInDefenseArea node("IsInDefenseArea", BBpointer);

    roboteam_msgs::World worldMsg;

    rtt::ai::world::world->updateWorld(worldMsg);

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;
    field.left_penalty_line.begin.x = -1.0f;
    field.left_penalty_line.end.x = -1.0f;

    field.left_penalty_line.begin.y = -1.0f;
    field.left_penalty_line.end.y = 1.0;

    field.right_penalty_line.begin.x = 1.0;
    field.right_penalty_line.end.x = 1.0;

    field.right_penalty_line.begin.y = -1.0f;
    field.right_penalty_line.end.y = 1.0;
    rtt::ai::world::field->set_field(field);

    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = -1.1;
    worldMsg.ball.pos.y = 0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should succeed since ball is in our defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = -0.7;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should fail since ball is not in out defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = -0.9;
    rtt::ai::world::world->updateWorld(worldMsg);

    // Should succeed since ball is within the margin of our defence area
    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}
