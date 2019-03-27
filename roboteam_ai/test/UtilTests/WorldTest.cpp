//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "roboteam_ai/src/world/World.h"
#include "../helpers/WorldHelper.h"
#include "roboteam_ai/src/world/WorldData.h"

namespace w = rtt::ai::world;

TEST(WorldTest, it_sets_and_gets_the_world) {
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldBall ball;
    EXPECT_FALSE(w::world->weHaveRobots());

    ball.pos.x = 42;
    ball.pos.y = 1;
    ball.vel.x = 100;
    ball.vel.y = -1;
    ball.visible = 1;
    worldMsg.ball = ball;
    
    w::world->setWorld(worldMsg);
    auto w = w::world->getWorld();
    EXPECT_EQ(w.ball.pos.x, 42);
    EXPECT_EQ(w.ball.pos.y, 1);
    EXPECT_EQ(w.ball.vel.x, 100);
    EXPECT_EQ(w.ball.vel.y, -1);
}

TEST(WorldTest, it_gets_the_ball) {
    roboteam_msgs::World worldMsg;

    worldMsg.ball.pos.x = 42;
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 99999;

    w::world->setWorld(worldMsg);

    auto ball = w::world->getBall();
    EXPECT_EQ(ball->pos.x, 42);
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
    w::world->setWorld(worldMsg);
    EXPECT_TRUE(w::world->weHaveRobots());

    EXPECT_FALSE(w::world->getRobotForId(1, true));

    auto robot1return = w::world->getRobotForId(0, true);
    EXPECT_FLOAT_EQ(robot1return->angle, 0.3);
    auto robot2return = w::world->getRobotForId(2, true);
    EXPECT_FLOAT_EQ(robot2return->angle, 0.4);
    EXPECT_EQ(w::world->getRobotForId(0, false), nullptr);

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
    w::world->setWorld(worldMsg);
    EXPECT_TRUE(w::world->ourRobotHasBall(0));
    EXPECT_FALSE(w::world->ourRobotHasBall(0,0.01));
    EXPECT_EQ(w::world->whichRobotHasBall(),0);
    EXPECT_EQ(w::world->whichRobotHasBall(),-1);
    EXPECT_TRUE(w::world->robotHasBall(0, true));
    robot2.id=3;
    robot2.pos.x=0.25;
    robot2.pos.y=0;
    robot2.angle=M_PI;
    worldMsg.them.push_back(robot2);
    w::world->setWorld(worldMsg);
    EXPECT_TRUE(w::world->theirRobotHasBall(3));
    EXPECT_FALSE(w::world->theirRobotHasBall(3,0.01));
    EXPECT_EQ(w::world->whichRobotHasBall(),0);
    EXPECT_EQ(w::world->whichRobotHasBall(w::WhichRobots::THEIR_ROBOTS),3);
    EXPECT_TRUE(w::world->robotHasBall(3, false));
    EXPECT_TRUE(w::world->robotHasBall(0, true));
}

TEST(WorldTest,bot_has_ball_us_repeated){
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;
    for (int j = 0; j < 1000; ++ j) {
        std::pair<roboteam_msgs::World,int> worldWithRobot=testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(8,8,true,field);
        w::world->setWorld(worldWithRobot.first);
        EXPECT_TRUE(w::world->ourRobotHasBall(worldWithRobot.second));
        EXPECT_EQ(w::world->whichRobotHasBall(),worldWithRobot.second);
    }
}
TEST(WorldTest,bot_has_ball_them_repeated){
    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;
    for (int j = 0; j < 1000; ++ j) {
        std::pair<roboteam_msgs::World,int> worldWithRobot=testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(8,8,false,field);
        w::world->setWorld(worldWithRobot.first);
        EXPECT_TRUE(w::world->theirRobotHasBall(worldWithRobot.second));
        EXPECT_EQ(w::world->whichRobotHasBall(w::WhichRobots::THEIR_ROBOTS),worldWithRobot.second);
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
    w::world->setWorld(worldMsg);
    EXPECT_TRUE(w::world->ourRobotHasBall(0));
    EXPECT_TRUE(w::world->ourRobotHasBall(3));

    worldMsg.us.clear();
    ball1.visible=false;
    worldMsg.ball=ball1;
    robot1.pos.x=3.0;
    robot1.pos.y=3.0;
    robot2.pos.x=1.0;
    robot2.pos.y=2.0;
    worldMsg.us.push_back(robot1);
    worldMsg.us.push_back(robot2);
    w::world->setWorld(worldMsg);
    EXPECT_TRUE(w::world->ourRobotHasBall(3));
    EXPECT_FALSE(w::world->ourRobotHasBall(0));

    worldMsg.ball.visible=true;
    w::world->setWorld(worldMsg);
    EXPECT_FALSE(w::world->ourRobotHasBall(3));
    EXPECT_FALSE(w::world->ourRobotHasBall(0));
}
