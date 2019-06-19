#include <gtest/gtest.h>
#include <roboteam_ai/src/conditions/BallOutOfField.h>
#include "../../src/world/World.h"
#include "roboteam_ai/src/world/Field.h"
#include "../../src/utilities/RobotDealer.h"

namespace rd = rtt::ai::robotDealer;
namespace w = rtt::ai::world;

namespace rtt {
namespace ai {

TEST(BallOutOfFieldTest, it_detects_ball_out_of_field) {
    auto BBpointer = std::make_shared<bt::Blackboard>();

    rtt::ai::BallOutOfField node("BallOutOfField", BBpointer);
    EXPECT_EQ(node.node_name(), "BallOutOfField");

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;

    w::field->set_field(field);
    roboteam_msgs::World worldMsg;

    worldMsg.ball.pos.x = 0.0;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.area = 99999;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    worldMsg.ball.pos.y = 5.1;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.y = -4.1;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.pos.x = 3.9;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

}
}
}
