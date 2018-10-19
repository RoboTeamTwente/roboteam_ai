//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "../src/utilities/World.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

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


