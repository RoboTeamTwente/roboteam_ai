#include "StrategyIOManager.h"

namespace rtt {
namespace ai {
namespace io {

StrategyIOManager::StrategyIOManager() {
  this->subscribeToWorldState();
  this->subscribeToRoleFeedback();
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
