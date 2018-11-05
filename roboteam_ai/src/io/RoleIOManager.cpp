#include "RoleIOManager.h"

namespace rtt {
namespace ai {
namespace io {

RoleIOManager::RoleIOManager() {
    // set up advertisement to publish robotcommands
    robotCommandPublisher = nodeHandle.advertise<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 100);
}

void RoleIOManager::subscribeToRoleDirective() {
    roleDirectiveSubscriber = nodeHandle.subscribe<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE,
            1,
            &RoleIOManager::handleRoleDirective,
            this);
}

void RoleIOManager::handleRoleDirective(const roboteam_msgs::RoleDirectiveConstPtr &roleDirective) {
    this->roleDirective = *roleDirective;
}

roboteam_msgs::RoleDirective &RoleIOManager::getRoleDirective() {
    return this->roleDirective;
}

void RoleIOManager::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    robotCommandPublisher.publish(cmd);
}

} // io
} // ai
} // rtt