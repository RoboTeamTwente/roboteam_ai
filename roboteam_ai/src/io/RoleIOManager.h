/*
 * Receives and handles the world state.
 * RoboTeamTwente, september 2018
 */

#ifndef ROBOTEAM_AI_ROLE_IO_NODE_H
#define ROBOTEAM_AI_ROLE_IO_NODE_H

#include "IOManager.h"
#include "roboteam_msgs/RoleDirective.h"
#include "roboteam_msgs/RobotCommand.h"
#include <roboteam_msgs/RoleFeedback.h>

namespace rtt {
namespace ai {
namespace io {

// the publisher is globally accessible (so the skills can publish)
ros::Publisher robotCommandPublisher;

class RoleIOManager : public IOManager {
 private:
  roboteam_msgs::RoleDirective roleDirective;
  void handleRoleDirective(const roboteam_msgs::RoleDirectiveConstPtr &roleDirective);
  ros::Subscriber roleDirectiveSubscriber;
 public:
  RoleIOManager();
  void subscribeToRoleDirective();
  roboteam_msgs::RoleDirective &getRoleDirective();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_ROLE_IO_NODE_H
