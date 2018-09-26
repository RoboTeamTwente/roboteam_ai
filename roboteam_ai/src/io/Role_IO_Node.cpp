//
// Created by mrlukasbos on 24-9-18.
//
// Subscribes to world

#include <roboteam_msgs/RoleFeedback.h>
#include "Role_IO_Node.h"

Role_IO_Node::Role_IO_Node() {

}

void Role_IO_Node::handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &str) {
  std::cout << str << std::endl;
}
