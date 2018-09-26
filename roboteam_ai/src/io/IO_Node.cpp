//
// Created by mrlukasbos on 19-9-18.
//

#include "IO_Node.h"

void IO_Node::subscribeToWorldState() {
  nodeHandle.subscribe<roboteam_msgs::World>(rtt::TOPIC_WORLD_STATE, 1000, &IO_Node::handleWorldState, this);
}

void IO_Node::handleWorldState(const roboteam_msgs::WorldConstPtr &str) {
  std::cout << str << std::endl;
}

void IO_Node::getWorldState() {

}


