//
// Created by rolf on 28-1-19.
//

#include <gtest/gtest.h>
#include "../../src/conditions/BallKickedToOurGoal.h"
#include "../../src/utilities/World.h"
#include "../../src/utilities/Field.h"

TEST(BallKickedToOurGoal,BallKickedToOurGoal){
    roboteam_msgs::GeometryFieldSize field;
    field.field_length=4.0;
    field.goal_width=0.8;
    rtt::ai::Field::set_field(field);

    roboteam_msgs::World worldMsg;
    worldMsg.ball.visible=1;
    worldMsg.ball.pos.x=0.0;
    worldMsg.ball.pos.y=0.0;
    worldMsg.ball.vel.x=0.0;
    worldMsg.ball.vel.y=0.0;
    worldMsg.ball.existence=99999;
    rtt::ai::World::set_world(worldMsg);

    bt::Blackboard BB;
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::BallKickedToOurGoal node("Test",BBpointer);

    EXPECT_EQ(node.node_name(), "BallKickedToOurGoal");

    EXPECT_EQ(node.update(),bt::Node::Status::Failure);
    worldMsg.ball.vel.x=-4.0f;
    worldMsg.ball.vel.y=0.0;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Success);
    worldMsg.ball.vel.x=4.0f;
    worldMsg.ball.vel.y=0.0;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);
    worldMsg.ball.vel.x=-2.0f;
    worldMsg.ball.vel.y=-0.3f;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Success);
    worldMsg.ball.vel.x=-4.0f;
    worldMsg.ball.vel.y=-1.0f;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(),bt::Node::Status::Failure);

}