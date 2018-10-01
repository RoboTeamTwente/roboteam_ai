//
// Created by mrlukasbos on 19-9-18.
//
// Listen to rostopic World_state

#ifndef ROBOTEAM_AI_IO_MANAGER_H
#define ROBOTEAM_AI_IO_MANAGER_H

#include "ros/ros.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/constants.h"
#include "roboteam_utils/LastWorld.h"
#include <iostream>

namespace rtt {
namespace ai {
namespace io {

class IOManager {
 private:
  roboteam_msgs::World world;
  ros::Subscriber worldSubscriber;
 protected:
  ros::NodeHandle nodeHandle;
  void handleWorldState(const roboteam_msgs::WorldConstPtr &world);

 public:
  IOManager() =default;
  void subscribeToWorldState();
  const roboteam_msgs::World &getWorldState();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H
