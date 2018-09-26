//
// Created by mrlukasbos on 24-9-18.
//

#include "StrategyIOManager.h"

void StrategyIOManager::subscribeToRoleFeedback() {
  nodeHandle.subscribe<roboteam_msgs::RoleFeedback>(rtt::TOPIC_ROLE_FEEDBACK, 1, &StrategyIOManager::handleRobotFeedback, this);
}

void StrategyIOManager::handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback) {
  this->roleFeedback = *rolefeedback;
}

roboteam_msgs::RoleFeedback &StrategyIOManager::getRoleFeedback() {
  return roleFeedback;
}


