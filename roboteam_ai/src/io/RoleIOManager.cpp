//
// Created by mrlukasbos on 24-9-18.
//
// Subscribes to world

#include <roboteam_msgs/RoleFeedback.h>
#include "RoleIOManager.h"

void RoleIOManager::subscribeToRoleDirective() {
  nodeHandle.subscribe<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 1, &RoleIOManager::handleRoleDirective, this);
}

void RoleIOManager::handleRoleDirective(const roboteam_msgs::RoleDirectiveConstPtr &roleDirective) {
  this->roleDirective = * roleDirective;
}

roboteam_msgs::RoleDirective &RoleIOManager::getRoleDirective() {
  return this->roleDirective;
}
