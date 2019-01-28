//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "../src/utilities/World.h"

TEST(WorldTest, it_sets_and_gets_the_world) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldBall ball;
    EXPECT_FALSE(rtt::ai::World::didReceiveFirstWorld);

    ball.z = 42;
    ball.z_vel = 100;
    ball.visible = 1;
    worldMsg.ball = ball;
    
    rtt::ai::World::set_world(worldMsg);
    EXPECT_EQ(rtt::ai::World::get_world().ball.z, 42);
    EXPECT_EQ(rtt::ai::World::get_world().ball.z_vel, 100);
}

TEST(WorldTest, it_gets_the_ball) {
    roboteam_msgs::World worldMsg;

    worldMsg.ball.z = 42;
    worldMsg.ball.visible = 1;
    rtt::ai::World::set_world(worldMsg);
    auto ball = rtt::ai::World::getBall();
    EXPECT_EQ(ball->z, 42);
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
    EXPECT_TRUE(rtt::ai::World::didReceiveFirstWorld);

    EXPECT_FALSE(rtt::ai::World::getRobotForId(1, true));

    auto robot1return = rtt::ai::World::getRobotForId(0, true);
    EXPECT_FLOAT_EQ(robot1return->angle, 0.3);
    auto robot2return = rtt::ai::World::getRobotForId(2, true);
    EXPECT_FLOAT_EQ(robot2return->angle, 0.4);
    EXPECT_EQ(rtt::ai::World::getRobotForId(0, false), nullptr);

}

TEST(WorldTest, it_gets_multiple_robot_ids) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2, robot3;

    std::vector<roboteam_msgs::WorldRobot> robotsUs;
    std::vector<roboteam_msgs::WorldRobot> robotsThem;

    robot1.id = 0;
    robot1.angle = 0.3;

    robot2.id = 2;
    robot2.angle = 0.4;

    robot3.id = 2;
    robot3.angle = 0.2;

    robotsUs.push_back(robot1);
    robotsUs.push_back(robot2);
    robotsThem.push_back(robot3);
    worldMsg.us = robotsUs;
    worldMsg.them = robotsThem;
    rtt::ai::World::set_world(worldMsg);
    
    // robot with id 1 should not exist
    EXPECT_EQ(rtt::ai::World::getRobotsForId({1}, true).size(), 0);
    
    // robot with id 0 should exit
    EXPECT_EQ(rtt::ai::World::getRobotsForId({0}, true).size(), 1);
    
    // sending the same id should work because it is a set
    EXPECT_EQ(rtt::ai::World::getRobotsForId({0, 0}, true).size(), 1);
    
    // two existing robots should be returned
    EXPECT_EQ(rtt::ai::World::getRobotsForId({0, 2}, true).size(), 2);

    // robot 1 does not exist but the two other robots still need to be returned
    EXPECT_EQ(rtt::ai::World::getRobotsForId({0, 1, 2}, true).size(), 2);

    //  the team should matter
    EXPECT_EQ(rtt::ai::World::getRobotForId(0, false), nullptr);

    // the data should be passed properly
    auto robot1return = rtt::ai::World::getRobotsForId({0}, true).front();
    EXPECT_FLOAT_EQ(robot1return.angle, 0.3);
    auto robot2return = rtt::ai::World::getRobotsForId({2}, true).front();
    EXPECT_FLOAT_EQ(robot2return.angle, 0.4);

    // robot 3 is from 'them'
    auto robot3return = rtt::ai::World::getRobotsForId({2}, false).front();
    EXPECT_FLOAT_EQ(robot3return.angle, 0.2);

    // the getAllRobots functions should return bot us and them
    EXPECT_EQ(rtt::ai::World::getAllRobots().size(), 3);
}
