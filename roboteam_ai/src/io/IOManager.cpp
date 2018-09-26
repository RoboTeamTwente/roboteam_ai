//
// Created by mrlukasbos on 19-9-18.
//

#include "IOManager.h"

void IOManager::subscribeToWorldState() {
  // previously also ros::TransportHints().tcpNoDelay() has been used as argument.
  nodeHandle.subscribe<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1, &IOManager::handleWorldState, this);
}

void IOManager::handleWorldState(const roboteam_msgs::WorldConstPtr& world) {
  this->world = * world;
}

const roboteam_msgs::World& IOManager::getWorldState() {
  return this->world;
}




