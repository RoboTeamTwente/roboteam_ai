//
// Created by rolf on 28-1-19.
//

#include <gtest/gtest.h>
#include "../../src/conditions/BallInDefenseAreaAndStill.h"
#include "../../src/utilities/World.h"
#include "../../src/utilities/Field.h"

TEST(DetectsDefenseArea,BallInDefenseAreaAndStill){
    bt::Blackboard BB;
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::BallInDefenseAreaAndStill node("Test",BBpointer);
    roboteam_msgs::GeometryFieldSize field;
    field.left_penalty_line.begin.x = -1.0f;
    field.left_penalty_line.end.x = -1.0f;

    field.left_penalty_line.begin.y = -1.0f;
    field.left_penalty_line.end.y = 1.0;
    field.right_penalty_line.begin.x = 1.0;
    field.right_penalty_line.end.x = 1.0;

    field.right_penalty_line.begin.y = -1.0f;
    field.right_penalty_line.end.y = 1.0;

    rtt::ai::Field::set_field(field);
    roboteam_msgs::World worldMsg;
    worldMsg.ball.pos.x=0;
    worldMsg.ball.pos.y=0;
    worldMsg.ball.visible=0;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);

    worldMsg.ball.pos.x=-1.5;
    worldMsg.ball.pos.y=0.0;
    worldMsg.ball.visible=1;

    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Success);
    worldMsg.ball.pos.y=-1.1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);
    worldMsg.ball.pos.y=1.1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);
    worldMsg.ball.pos.y=0.0;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Success);
    worldMsg.ball.vel.x=0.11;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);
}