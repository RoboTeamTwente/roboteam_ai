//
// Created by robzelluf on 2/18/19.
//

#include <gtest/gtest.h>
#include "../../src/conditions/IsBallCloseToBorder.h"
#include "../../src/utilities/Field.h"
#include "../../src/utilities/Coach.h"

namespace rtt{
namespace ai {

TEST(IsBallCloseToBorderTest, is_not_close_to_border) {
    robotDealer::RobotDealer::halt();
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 9;
    field.field_length = 12;
    Field::set_field(field);

    roboteam_msgs::World world;
    world.ball.pos = Vector2{0, 0};
    world.ball.visible = static_cast<unsigned char>(true);
    world.ball.existence = 99999;
    World::set_world(world);
    
    bt::Blackboard properties;
    auto propertiesPointer = std::make_shared<bt::Blackboard>(properties);
    
    rtt::ai::IsBallCloseToBorder node("Test", propertiesPointer);
    node.initialize();
    
    ASSERT_EQ(node.update(), bt::Node::Status::Failure);
}

TEST(IsBallCloseToBorderTest, is_close_to_border) {
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 9;
    field.field_length = 12;
    Field::set_field(field);

    roboteam_msgs::World world;
    world.ball.visible = 1;
    world.ball.pos = Vector2{5.90, 4.40};
    world.ball.existence = 99999;
    World::set_world(world);

    bt::Blackboard properties;
    auto propertiesPointer = std::make_shared<bt::Blackboard>(properties);

    rtt::ai::IsBallCloseToBorder node("Test", propertiesPointer);
    node.initialize();

    ASSERT_EQ(node.update(), bt::Node::Status::Success);

    propertiesPointer->setDouble("margin", 0.05);
    node.initialize();

    ASSERT_EQ(node.update(), bt::Node::Status::Failure);
}

} //ai
} //rtt

