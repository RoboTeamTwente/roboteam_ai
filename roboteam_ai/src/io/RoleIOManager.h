/*
 * Receives and handles the world state.
 * RoboTeamTwente, september 2018
 */

#ifndef ROBOTEAM_AI_ROLE_IO_NODE_H
#define ROBOTEAM_AI_ROLE_IO_NODE_H

#include "IOManager.h"
#include "roboteam_msgs/RoleDirective.h"

class RoleIOManager : public IOManager {
 private:
  roboteam_msgs::RoleDirective roleDirective;
  void handleRoleDirective(const roboteam_msgs::RoleDirectiveConstPtr &roleDirective);

 public:
  RoleIOManager();
  void subscribeToRoleDirective();
  roboteam_msgs::RoleDirective &getRoleDirective();

};

#endif //ROBOTEAM_AI_ROLE_IO_NODE_H
