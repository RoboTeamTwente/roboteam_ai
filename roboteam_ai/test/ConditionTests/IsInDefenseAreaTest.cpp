//
// Created by rolf on 17-10-18.
//
#include <gtest/gtest.h>
#include "../../src/conditions/IsInDefenseArea.hpp"

#include "../../src/utilities/Field.h"

TEST(DetectsInOurDefenseArea,IsInDefenseAreaTest){
    bt::Blackboard BB;
    BB.setBool("robot", true);
    BB.setInt("ROBOT_ID", 2);
    BB.setDouble("margin", 0.2);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInDefenseArea node("Test", BBpointer);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::GeometryFieldSize field;
    field.left_penalty_line.begin.x = -1.0;
    field.left_penalty_line.begin.x = -1.0;

    field.left_penalty_line.end.y = -1.0;
    field.left_penalty_line.end.y = 1.0;

    rtt::ai::Field::set_field(field);

    robot.id = 2;
    robot.pos.x = -1.2;
    robot.pos.y = 0;

    worldMsg.us.push_back(robot);
    rtt::ai::World::set_world(worldMsg);

    // Should succeed since robot is in our defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.us[0].pos.x = -0.7;
    rtt::ai::World::set_world(worldMsg);

    // Should fail since robot is not in out defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.us[0].pos.x = -0.9;
    rtt::ai::World::set_world(worldMsg);

    // Should succeed since robot is within the margin of our defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Success);
}

TEST(DetectsInTheirDefenseArea,IsInDefenseAreaTest){
    bt::Blackboard BB;
    BB.setBool("robot", true);
    BB.setInt("ROBOT_ID", 2);
    BB.setDouble("margin", 0.2);
    BB.setBool("ourDefenseArea", false);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInDefenseArea node("Test", BBpointer);

    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;

    rtt::ai::World::set_world(worldMsg);

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::GeometryFieldSize field;
    field.right_penalty_line.begin.x = 1.0;
    field.right_penalty_line.begin.x = 1.0;

    field.right_penalty_line.end.y = -1.0;
    field.right_penalty_line.end.y = 1.0;

    rtt::ai::Field::set_field(field);

    robot.id = 2;
    robot.pos.x = 1.2;
    robot.pos.y = 0;

    worldMsg.us.push_back(robot);
    rtt::ai::World::set_world(worldMsg);

    // Should succeed since robot is in their defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.us[0].pos.x = 0.7;
    rtt::ai::World::set_world(worldMsg);

    // Should fail since robot is not in their defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.us[0].pos.x = 0.9;
    rtt::ai::World::set_world(worldMsg);

    // Should succeed since robot is within the margin of their defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Success);
}

TEST(DetectsBallInOurDefenceArea, IsInDefenceAreaTest) {
    bt::Blackboard BB;
    BB.setBool("robot", false);
    BB.setDouble("margin", 0.2);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsInDefenseArea node("Test", BBpointer);

    roboteam_msgs::World worldMsg;

    rtt::ai::World::set_world(worldMsg);

    //ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    roboteam_msgs::GeometryFieldSize field;
    field.left_penalty_line.begin.x = -1.0;
    field.left_penalty_line.begin.x = -1.0;

    field.left_penalty_line.end.y = -1.0;
    field.left_penalty_line.end.y = 1.0;

    rtt::ai::Field::set_field(field);

    worldMsg.ball.pos.x = -1.1;
    worldMsg.ball.pos.y = 0;

    rtt::ai::World::set_world(worldMsg);

    // Should succeed since robot is in our defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.x = -0.7;
    rtt::ai::World::set_world(worldMsg);

    // Should fail since robot is not in out defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.x = -0.9;
    rtt::ai::World::set_world(worldMsg);

    // Should succeed since robot is within the margin of our defence area
    ASSERT_EQ(node.update(), bt::Node::Status::Success);
}