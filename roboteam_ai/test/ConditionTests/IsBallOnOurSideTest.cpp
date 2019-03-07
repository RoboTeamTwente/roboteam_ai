
#include <gtest/gtest.h>
#include <roboteam_ai/src/conditions/IsBallOnOurSide.h>
#include "../../src/conditions/BallInDefenseAreaAndStill.h"
#include "../../src/utilities/World.h"
#include "../../src/utilities/Field.h"
namespace rtt {
namespace ai {

TEST(IsBallOnOurSideTest, it_detects_ball_on_our_side)
{
    bt::Blackboard BB;
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    BBpointer->setBool("inField", true);
    rtt::ai::IsBallOnOurSide node("Test", BBpointer);
    node.initialize();
    EXPECT_TRUE(node.inField); // check if the property is handled properly
    EXPECT_EQ(node.node_name(), "IsBallOnOurSide");

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;

    rtt::ai::Field::set_field(field);
    roboteam_msgs::World worldMsg;

    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure); // return failure because no ball

    worldMsg.ball.pos.x = -1;
    worldMsg.ball.pos.y = 0;
    worldMsg.ball.visible = 0;
    worldMsg.ball.area = 99999;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure); // return failure because no ball visible

    // our side
    worldMsg.ball.pos.x = -1.5;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    // our side
    worldMsg.ball.pos.y = -1.1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    // our side
    worldMsg.ball.pos.y = 1.1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    // not our side
    worldMsg.ball.pos.x = 0.1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    // out of field
    worldMsg.ball.pos.x = -6;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);
}
}
}