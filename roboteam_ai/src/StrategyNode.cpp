//
// Created by mrlukasbos on 21-9-18.
//
// Start a strategy node

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

  // Terminate if needed
//  if (strategy->getStatus()==bt::Node::Status::Running) {
//    strategy->Terminate(bt::Node::Status::Running);
//  }

  return 0;
}
