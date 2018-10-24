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
#include <roboteam_msgs/GeometryData.h>
#include "roboteam_msgs/RefereeData.h"

namespace rtt {
namespace ai {
namespace io {

class IOManager {
 private:
  roboteam_msgs::World world;
  roboteam_msgs::GeometryData geometry;
  roboteam_msgs::RefereeData refData;
  ros::Subscriber worldSubscriber;
  ros::Subscriber geometrySubscriber;
  ros::Subscriber refereeSubscriber;

 protected:
  ros::NodeHandle nodeHandle;
  void handleWorldState(const roboteam_msgs::WorldConstPtr &world);
  void handleGeometryData(const roboteam_msgs::GeometryDataConstPtr &geometry);
  void handleRefereeData(const roboteam_msgs::RefereeDataConstPtr &refData);

 public:
  IOManager() =default;
  void subscribeToWorldState();
  void subscribeToGeometryData();
  void subscribeToRefereeData();
  const roboteam_msgs::World &getWorldState();
  const roboteam_msgs::GeometryData &getGeometryData();
  const roboteam_msgs::RefereeData &getRefereeData();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_IO_MANAGER_H