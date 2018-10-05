//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "../src/DangerFinder/DangerFinder.h"
#include "../src/utilities/World.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"

namespace df = rtt::ai::dangerfinder;

TEST(DangerFinderTest, it_logs_dangerdata) {
  roboteam_msgs::World worldMsg;
  int amountOfRobots = 11;

  // set some opponent robots
  for (int i = 0; i < amountOfRobots; i++) {
    roboteam_msgs::WorldRobot robot;
    worldMsg.them.push_back(robot);
  }

  // set the global world object.
  rtt::ai::World::set_world(worldMsg);

  // start the dangerfinder with the worldMsg and receive dangerdata
  df::DangerData danger = df::DangerFinder::instance().getMostRecentData();

  // All robots should have dangerscores
  ASSERT_EQ(danger.dangerList.size(), amountOfRobots);
}
