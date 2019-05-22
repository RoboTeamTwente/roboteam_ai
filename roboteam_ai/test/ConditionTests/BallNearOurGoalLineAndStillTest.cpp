//
// Created by mrlukasbos on 22-5-19.
//


#include <gtest/gtest.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/test/helpers/FieldHelper.h>
#include "../../src/conditions/BallNearOurGoalLineAndStill.h"
#include "../../src/world/World.h"
#include "roboteam_ai/src/world/Field.h"
namespace rtt {
namespace ai {

TEST(BallNearOurGoalLineAndStillTest, BallNearOurGoalLineAndStill) {
        roboteam_msgs::World worldMsg;
        roboteam_msgs::WorldRobot robot;

        world::field->set_field(testhelpers::FieldHelper::generateField());
        rtt::ai::world::world->updateWorld(worldMsg);
        rtt::ai::world::Ball::exists = false;
        rtt::ai::robotDealer::RobotDealer::halt();
        auto BB = std::make_shared<bt::Blackboard>();

        BB->setString("ROLE","test");
        rtt::ai::BallNearOurGoalLineAndStill node("BallNearOurGoalLineAndStill", BB);
        EXPECT_EQ(node.node_name(), "BallNearOurGoalLineAndStill");

        robot.id = 0;
        robot.pos.x = 0;
        robot.pos.y = 0;
        worldMsg.us.push_back(robot);

        worldMsg.ball.pos.x = 0.1;
        worldMsg.ball.pos.y = 0.0;
        worldMsg.ball.vel.x = 3.0;
        worldMsg.ball.vel.y = 3.0;
        worldMsg.ball.visible = 1;
        worldMsg.ball.existence = 99999;

        rtt::ai::world::world->updateWorld(worldMsg);
        rtt::ai::robotDealer::RobotDealer::claimRobotForTactic( rtt::ai::robotDealer::RobotType::RANDOM, "test", "BallNearOurGoalLineAndStillTactic");
        node.initialize();
        EXPECT_EQ(node.margin, 1.2*Constants::ROBOT_RADIUS());


        // the ball is moving and not near the goal line
        EXPECT_EQ(node.update(), bt::Node::Status::Failure);


        // the ball is not moving but not near the goal line
        worldMsg.ball.vel.x = 0.0;
        worldMsg.ball.vel.y = 0.0;
        rtt::ai::world::world->updateWorld(worldMsg);
        EXPECT_EQ(node.update(), bt::Node::Status::Failure);

        // the ball is not moving and  near the goal line
        worldMsg.ball.pos.x = -world::field->get_field().field_length/2;
        rtt::ai::world::world->updateWorld(worldMsg);
        EXPECT_EQ(node.update(), bt::Node::Status::Success);

        // the ball is not moving but near the wrong (their) goal line
        worldMsg.ball.pos.x = world::field->get_field().field_length/2;
        rtt::ai::world::world->updateWorld(worldMsg);
        EXPECT_EQ(node.update(), bt::Node::Status::Failure);


        BB->setDouble("margin", 0.1);
        node.setProperties(BB);
        node.initialize();
        EXPECT_EQ(node.margin, 0.1);


}
}
}