//
// Created by mrlukasbos on 19-9-18.
//
// Listen to rostopic World_state

#ifndef PROJECT_ROS_HANDLER_H
#define PROJECT_ROS_HANDLER_H

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/RoleFeedback.h"
#include "roboteam_utils/constants.h"
#include <iostream>

class IO_Node {
 public:
  IO_Node() = default;
  virtual void publishAll();
  virtual void getWorldState();

 protected:
  ros::NodeHandle nodeHandle;
  ros::NodeHandle debugHandle;

  void IO_Node::subscribeToWorldState();
  void handleWorldState(const roboteam_msgs::WorldConstPtr &str);
};

#endif //PROJECT_ROS_HANDLER_H
