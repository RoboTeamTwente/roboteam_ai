/*
 * Creates an instance of StrategyNode
 * The strategynode decides which strategy to use based upon the following game parameters:
 *  1) The world state, which contains:
 *    a) The refcommands
 *    b) The location of all robots (both us and them)
 *    c) the field geometry
 *
 *  2) A danger list which is generated based upon the world state
 *
 * RoboTeamTwente, september 2018
 */

#include "ros/ros.h"
#include "io/StrategyIOManager.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "StrategyNode");

  StrategyIOManager strategyIOManager;
  strategyIOManager.subscribeToWorldState();
  strategyIOManager.subscribeToRoleFeedback();

  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();

    strategyIOManager.getWorldState();
    strategyIOManager.getRoleFeedback();

    rate.sleep();
  }
  return 0;
}
