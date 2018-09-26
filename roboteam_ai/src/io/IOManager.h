//
// Created by mrlukasbos on 19-9-18.
//
// Listen to rostopic World_state

#ifndef PROJECT_ROS_HANDLER_H
#define PROJECT_ROS_HANDLER_H

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/LastWorld.h"
#include <iostream>

class IOManager {
 private:
  roboteam_msgs::World world;

 protected:
  ros::NodeHandle nodeHandle;
  void handleWorldState(const roboteam_msgs::WorldConstPtr &str);

 public:
  IOManager() =default;
  void subscribeToWorldState();
  const roboteam_msgs::World &getWorldState();
};

#endif //PROJECT_ROS_HANDLER_H
