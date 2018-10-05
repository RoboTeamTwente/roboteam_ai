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
  EXPECT_EQ(rtt::ai::World::get_world().ball.z, 42);
  EXPECT_EQ(rtt::ai::World::get_world().ball.z_vel, 100);
}

TEST(WorldTest, it_gets_and_sets_the_field) {
  roboteam_msgs::GeometryFieldSize field;
  field.boundary_width = 42;

  rtt::ai::World::set_field(field);
  EXPECT_EQ(rtt::ai::World::get_field().boundary_width, 42);
}
