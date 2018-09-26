//
// Created by mrlukasbos on 24-9-18.
//

#include "Strategy_IO_Node.h"

Strategy_IO_Node::Strategy_IO_Node() {
  subscribeToWorldState();
  subscribeToRoleFeedback();
}

void Strategy_IO_Node::subscribeToRoleFeedback() {
  nodeHandle.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_WORLD_STATE, 1000, &IO_Node::handleRobotFeedback, this);
}

void Strategy_IO_Node::publishRoleDirective() {

}
