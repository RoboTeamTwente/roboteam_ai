#include <roboteam_msgs/RoleDirective.h>
#include "StrategyIOManager.h"

namespace rtt {
namespace ai {
namespace io {

StrategyIOManager::StrategyIOManager() {
  this->subscribeToWorldState();
  this->subscribeToGeometryData();
  this->subscribeToRoleFeedback();

  // set up advertisement to publish roledirectives
  roleDirectivePublisher = nodeHandle.advertise<roboteam_msgs::RoleDirective>(rtt::TOPIC_ROLE_DIRECTIVE, 1000);
}

void StrategyIOManager::subscribeToRoleFeedback() {
  roleFeedbackSubscriber = nodeHandle.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 1, &StrategyIOManager::handleRobotFeedback, this);
}

void StrategyIOManager::handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback) {
  this->roleFeedback = *rolefeedback;
}

roboteam_msgs::RoleFeedback &StrategyIOManager::getRoleFeedback() {
  return roleFeedback;
}

} // io
} // ai
} // rtt
