//
// Created by robzelluf on 5/22/19.
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/conditions/IsBeingPassedTo.h"
#include "roboteam_ai/src/coach/PassCoach.h"
#include "../helpers/FieldHelper.h"

namespace rd = rtt::ai::robotDealer;
namespace w = rtt::ai::world;

TEST(IsBeingPassedTo, IsBeingPassedToTest) {
    rd::RobotDealer::halt();

    roboteam_msgs::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    rtt::ai::world::field->set_field(field);
    roboteam_msgs::World world;

    roboteam_msgs::WorldRobot robot0;
    robot0.id = 0;
    robot0.pos = Vector2{0, 0};
    world.us.push_back(robot0);

    roboteam_msgs::WorldRobot robot1;
    robot1.id = 1;
    robot1.pos.x = 2;
    robot1.pos.y = 0;
    roboteam_msgs::WorldBall ball;
    ball.existence = 11;
    ball.visible = static_cast<unsigned char>(true);
    ball.pos = Vector2(1.0,1.0);
    world.us.push_back(robot1);
    world.ball = ball;
    w::world->updateWorld(world);

    rtt::ai::robotDealer::RobotDealer::claimRobotForTactic(rtt::ai::robotDealer::RobotType::CLOSE_TO_THEIR_GOAL, "test2", "IsBeingPassedToTest");

    bt::Blackboard BB;
    BB.setString("ROLE", "test2");
    BB.setInt("ROBOT_ID", 1);
    auto BBpointer = std::make_shared<bt::Blackboard>(BB);
    rtt::ai::IsBeingPassedTo node("IsBeingPassedTo", BBpointer);

    EXPECT_EQ(node.update(), bt::Node::Status::Failure);

    rtt::ai::coach::g_pass.initiatePass(0);
    node.initialize();

    EXPECT_EQ(node.update(), bt::Node::Status::Success);

}
