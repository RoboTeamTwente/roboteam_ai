//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "../src/utilities/World.h"

TEST(WorldTest, it_sets_and_gets_the_world) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldBall ball;

    ball.z = 42;
    ball.z_vel = 100;
    worldMsg.ball = ball;

    rtt::ai::World::set_world(worldMsg);
    ASSERT_EQ(rtt::ai::World::get_world().ball.z, 42);
    ASSERT_EQ(rtt::ai::World::get_world().ball.z_vel, 100);
}

TEST(WorldTest, it_gets_the_ball) {
    roboteam_msgs::World worldMsg;

    worldMsg.ball.z = 42;
    rtt::ai::World::set_world(worldMsg);
    roboteam_msgs::WorldBall ball = rtt::ai::World::getBall();
    ASSERT_EQ(ball.z, 42);
}

TEST(WorldTest, it_gets_the_robot_ID) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2;

    std::vector<roboteam_msgs::WorldRobot> robots;
    robot1.id = 0;
    robot1.angle = 0.3;

    robot2.id = 2;
    robot2.angle = 0.4;
    robots.push_back(robot1);
    robots.push_back(robot2);
    worldMsg.us = robots;
    rtt::ai::World::set_world(worldMsg);
    ASSERT_TRUE(! rtt::ai::World::getRobotForId(1, true));

    auto robot1return = rtt::ai::World::getRobotForId(0, true).get();
    auto robot2return = rtt::ai::World::getRobotForId(2, true).get();
    ASSERT_FLOAT_EQ(robot1return->angle, 0.3);
    ASSERT_FLOAT_EQ(robot2return->angle, 0.4);
    ASSERT_EQ(rtt::ai::World::getRobotForId(0, false), nullptr);

}
