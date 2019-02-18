//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/utilities/World.h"
#include "../helpers/WorldHelper.h"

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

TEST(WorldTest, bot_has_ball){
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2;
    roboteam_msgs::WorldBall ball1;
    robot1.id=0;
    robot1.pos.x=0.05;
    robot1.pos.y=0;
    robot1.angle=0;

    ball1.pos.x=0.15;
    ball1.pos.y=0;
    ball1.visible=true;
    worldMsg.us.push_back(robot1);
    worldMsg.ball=ball1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_TRUE(rtt::ai::World::ourBotHasBall(0));
    EXPECT_FALSE(rtt::ai::World::ourBotHasBall(0,0.01));
    EXPECT_EQ(rtt::ai::World::whichBotHasBall(true),0);
    EXPECT_EQ(rtt::ai::World::whichBotHasBall(false),-1);
    EXPECT_TRUE(rtt::ai::World::BotHasBall(3,false));
    EXPECT_FALSe(rtt::ai::World::BotHasBall(0,true));
    robot2.id=3;
    robot2.pos.x=0.25;
    robot2.pos.y=0;
    robot2.angle=M_PI;
    worldMsg.them.push_back(robot2);
    rtt::ai::World::set_world(worldMsg);
    EXPECT_TRUE(rtt::ai::World::theirBotHasBall(3));
    EXPECT_FALSE(rtt::ai::World::theirBotHasBall(3,0.01));
    EXPECT_EQ(rtt::ai::World::whichBotHasBall(true),0);
    EXPECT_EQ(rtt::ai::World::whichBotHasBall(false),3);
    EXPECT_TRUE(rtt::ai::World::BotHasBall(3,false));
    EXPECT_TRUE(rtt::ai::World::BotHasBall(0,true));
}

TEST(WorldTest,bot_has_ball_us_repeated){
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;
    for (int j = 0; j < 1000; ++ j) {
        std::pair<roboteam_msgs::World,int> worldWithRobot=testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(8,8,true,field);
        rtt::ai::World::set_world(worldWithRobot.first);
        EXPECT_TRUE(rtt::ai::World::ourBotHasBall(worldWithRobot.second));
        EXPECT_EQ(rtt::ai::World::whichBotHasBall(true),worldWithRobot.second);
    }
}
TEST(WorldTest,bot_has_ball_them_repeated){
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;
    for (int j = 0; j < 1000; ++ j) {
        std::pair<roboteam_msgs::World,int> worldWithRobot=testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(8,8,false,field);
        rtt::ai::World::set_world(worldWithRobot.first);
        EXPECT_TRUE(rtt::ai::World::theirBotHasBall(worldWithRobot.second));
        EXPECT_EQ(rtt::ai::World::whichBotHasBall(false),worldWithRobot.second);
    }
}
TEST(WorldTest,ball_visibility){

    //First normal worldstate where 2 bots ' have the ball'. Robot 3 is closer to it so the ball should move with robot 3
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot1, robot2;
    roboteam_msgs::WorldBall ball1;
    robot1.id=0;
    robot1.pos.x=0.05;
    robot1.pos.y=0;
    robot1.angle=0;

    robot2.id=3;
    robot2.pos.x=0.24;
    robot2.pos.y=0;
    robot2.angle=M_PI;

    ball1.pos.x=0.15;
    ball1.pos.y=0;
    ball1.visible=true;
    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    worldMsg.ball=ball1;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_TRUE(rtt::ai::World::ourBotHasBall(0));
    EXPECT_TRUE(rtt::ai::World::ourBotHasBall(3));

    worldMsg.us.clear();
    ball1.visible=false;
    worldMsg.ball=ball1;
    robot1.pos.x=3.0;
    robot1.pos.y=3.0;
    robot2.pos.x=1.0;
    robot2.pos.y=2.0;
    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    rtt::ai::World::set_world(worldMsg);
    EXPECT_TRUE(rtt::ai::World::ourBotHasBall(3));
    EXPECT_FALSE(rtt::ai::World::ourBotHasBall(0));

    worldMsg.ball.visible=true;
    rtt::ai::World::set_world(worldMsg);
    EXPECT_FALSE(rtt::ai::World::ourBotHasBall(3));
    EXPECT_FALSE(rtt::ai::World::ourBotHasBall(0));
}
