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

class RoleIOManager : public IOManager {
 private:
  roboteam_msgs::RoleDirective roleDirective;
  void handleRoleDirective(const roboteam_msgs::RoleDirectiveConstPtr &roleDirective);
  ros::Subscriber roleDirectiveSubscriber;
  ros::Publisher robotCommandPublisher;
 public:
  RoleIOManager();
  void subscribeToRoleDirective();
  roboteam_msgs::RoleDirective &getRoleDirective();
  void publishRobotCommand(roboteam_msgs::RobotCommand cmd);
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_ROLE_IO_NODE_H
