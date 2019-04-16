
#include <gtest/gtest.h>
#include <roboteam_ai/src/conditions/IsBallOnOurSide.h>
#include "../../src/conditions/BallInDefenseAreaAndStill.h"
#include "../../src/world/World.h"
#include "../../src/utilities/RobotDealer.h"
#include "roboteam_ai/src/world/Field.h"

namespace rd = rtt::ai::robotDealer;
namespace w = rtt::ai::world;

namespace rtt {
namespace ai {

TEST(IsBallOnOurSideTest, it_detects_ball_on_our_side)
{
    bt::Blackboard BB;
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    BBpointer->setBool("inField", true);
    rtt::ai::IsBallOnOurSide node("IsBallOnOurSide", BBpointer);

    // initialize, but because there is no ball it does not succeed initializing. infield stays false.
    node.initialize();
    EXPECT_EQ(node.node_name(), "IsBallOnOurSide");

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 8;

    w::field->set_field(field);
    roboteam_msgs::World worldMsg;

    worldMsg.ball.pos.x = -1;
    worldMsg.ball.pos.y = 0;
    worldMsg.ball.visible = 0;
    worldMsg.ball.existence = 0;

    w::world->updateWorld(worldMsg);
    node.initialize();
    EXPECT_EQ(node.update(), bt::Node::Status::Waiting); // return failure because no ball visible

    // our side
    worldMsg.ball.pos.x = -1.5;
    worldMsg.ball.pos.y = 0.0;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 9999;

    w::world->updateWorld(worldMsg);
    node.initialize();
    EXPECT_TRUE(node.inField); // check if the property is handled properly

    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    // our side
    worldMsg.ball.pos.y = -1.1;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    // our side
    worldMsg.ball.pos.y = 1.1;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);

    // not our side
    worldMsg.ball.pos.x = 0.1;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    // out of field
    worldMsg.ball.pos.x = -10;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    // out of field
    worldMsg.ball.pos.x = -6;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    // out of field
    worldMsg.ball.pos.x = -5.999;
    w::world->updateWorld(worldMsg);
    EXPECT_EQ(node.update(), bt::Node::Status::Success);
}
}
}