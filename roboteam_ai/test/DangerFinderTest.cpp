//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "../src/DangerFinder/DangerFinder.h"
#include "../src/utilities/World.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include <random>
#include <bitset>

namespace df = rtt::ai::dangerfinder;

// fieldMsg is the same everywhere
roboteam_msgs::GeometryFieldSize fieldMsg;

void setFieldtoWorld() {
  // set the field parameters
  fieldMsg.field_length = 1000;
  fieldMsg.field_width = 500;
  fieldMsg.goal_width = 200;
  fieldMsg.goal_depth = 20;

  // set the field to the world
  rtt::ai::World::set_field(fieldMsg);
}



TEST(DangerFinderTest, it_has_correct_danger_flags) {
  roboteam_msgs::World worldMsg;
  setFieldtoWorld();

  roboteam_msgs::WorldRobot robotUs;
  roboteam_msgs::WorldRobot robotThem;
  roboteam_msgs::WorldBall ball;

  // there is nothing between the opponent and the ball
  robotUs.pos = rtt::Vector2(0, 0);
  robotThem.pos = rtt::Vector2(0, 200);
  ball.pos = rtt::Vector2(0, 100);

  worldMsg.ball = ball;
  worldMsg.them.push_back(robotThem);
  worldMsg.us.push_back(robotUs);

  // set the global world object again
  rtt::ai::World::set_world(worldMsg);

  // force calculate dangerfinder
  df::DangerData danger = df::DangerFinder::instance().calculateDataNow();

  ASSERT_EQ(danger.flags.size(), 1);
  // the robotflag ISFREE should be triggered
  ASSERT_EQ(std::bitset<8>(danger.flags.at(0)), std::bitset<8>(0b00000001));

  // we are between the opponent and the ball
  robotUs.pos = rtt::Vector2(0, 200);
  robotThem.pos = rtt::Vector2(0, 400);
  ball.pos = rtt::Vector2(0, 0);
  worldMsg.ball = ball;
  worldMsg.them.at(0) = robotThem;
  worldMsg.us.at(0) = robotUs;

  // set the global world object again
  rtt::ai::World::set_world(worldMsg);

  // force calculate dangerfinder
  danger = df::DangerFinder::instance().calculateDataNow();

  ASSERT_EQ(danger.flags.size(), 1);
  // the robotflag ISFREE should NOT be triggered,
  // but the robot is closing in on our goal
  ASSERT_EQ(std::bitset<8>(danger.flags.at(0)), std::bitset<8>(0b00000010));


  // force calculate dangerfinde again without moving the robot (so it is standing still)
  danger = df::DangerFinder::instance().calculateDataNow();

  ASSERT_EQ(danger.flags.size(), 1);
  // the robotflag ISFREE should NOT be triggered,
  // the robot is not anymore closing in on our goal
  ASSERT_EQ(std::bitset<8>(danger.flags.at(0)), std::bitset<8>(0b00000000));
}

// Create an amount of robots in the world and start the dangerfinder
// Dangerfinder should contain dangerData of the robots in the world.
TEST(DangerFinderTest, it_logs_dangerdata) {
    setFieldtoWorld();
   roboteam_msgs::World worldMsg;

   int amountOfRobots = 11;
    // set some robots
    // set them all at exactly the same location for testing purposes
    for (int i = 0; i < amountOfRobots; i++) {
      roboteam_msgs::WorldRobot robot;

      robot.pos= rtt::Vector2(100, 100);

      robot.id = (unsigned int) i; // this is important for now. TODO let the dangerfinder use push_back (?)
      worldMsg.us.push_back(robot);
      worldMsg.them.push_back(robot);
    }

    // set the global world object.
    rtt::ai::World::set_world(worldMsg);

    // start the dangerfinder with the worldMsg and receive dangerdata
    df::DangerData danger = df::DangerFinder::instance().calculateDataNow();

    // All robots of the enemy should have dangerscores
    ASSERT_EQ(danger.dangerList.size(), amountOfRobots);
    ASSERT_EQ(danger.scores.size(), amountOfRobots);
    ASSERT_EQ(danger.flags.size(), amountOfRobots);

    // there is no data set for the robots so their danger scores should be equal
    for (int i = 1; i < danger.scores.size(); i++) {
      ASSERT_EQ(danger.scores.at(i-1), danger.scores.at(i));
    }
    int oldDangerScore = (int) danger.scores.at(0);

    // add a ball to the middle of the field
    worldMsg.ball.pos = rtt::Vector2(0, 0);

    // set the global world object again
    rtt::ai::World::set_world(worldMsg);

    // force recalculate
    danger = df::DangerFinder::instance().calculateDataNow();

    // there is still no different data set for the robots so their danger scores should be equal
    for (int i = 1; i < danger.scores.size(); i++) {
      ASSERT_EQ(danger.scores.at(i-1), danger.scores.at(i));
    }
    // the presence of the ball should change dangerscores
    ASSERT_NE(danger.scores.at(0), oldDangerScore);

    // move one robot forward with the ball
    // and point him towards the goal
    worldMsg.them.at(0).pos = rtt::Vector2(200, 0);
    worldMsg.them.at(0).angle = 28 * M_PI;
    worldMsg.ball.pos = rtt::Vector2(250, 0);

    // set the global world object again
    rtt::ai::World::set_world(worldMsg);
    // force recalculate
      danger = df::DangerFinder::instance().calculateDataNow();

    // his dangerscore should be sky high (because of canshoot)
    // the canshoot module adds a score of 999, so it should be at least that value.
    ASSERT_GE(danger.scores[danger.getByDangerRank(0)->id], 999);
}

TEST(DangerFinderTest, it_stays_within_limits) {
  setFieldtoWorld();
  for (int i = 0; i < 1000; i++) {
    roboteam_msgs::World tempWorldMsg;
    // set robots at random locations
    int amountOfRobots = 11;
    for (int i = 0; i < amountOfRobots; i++) {
      roboteam_msgs::WorldRobot robot;

      // set robots at random locations in the field
      int halfFieldLength = (int) fieldMsg.field_length/2;
      int halfFieldWidth = (int) fieldMsg.field_width/2;
      int randomX = rand()%(halfFieldLength*2 + 1) + halfFieldLength;
      int randomY = rand()%(halfFieldWidth*2 + 1) + halfFieldWidth;
      robot.pos = rtt::Vector2(randomX, randomY);
      robot.id = (unsigned int) i; // this is important for now. TODO let the dangerfinder use push_back (?)
      tempWorldMsg.us.push_back(robot);
      tempWorldMsg.them.push_back(robot);
    }

    // set the global world object.
    rtt::ai::World::set_world(tempWorldMsg);

    // start the dangerfinder with the worldMsg and receive dangerdata
    df::DangerData danger = df::DangerFinder::instance().calculateDataNow();

    // values can never be smaller than 0
    for (int j = 0; j < danger.scores.size(); j++) {
      ASSERT_GE(danger.scores.at(j), 0);
    }
  }
}
