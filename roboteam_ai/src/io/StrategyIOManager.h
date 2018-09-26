//
// Created by mrlukasbos on 24-9-18.
//

#ifndef ROBOTEAM_AI_STRATEGY_IO_NODE_H
#define ROBOTEAM_AI_STRATEGY_IO_NODE_H

#include "IOManager.h"
#include "roboteam_msgs/RoleFeedback.h"

class StrategyIOManager : public IOManager {
 private:
  roboteam_msgs::RoleFeedback roleFeedback;
  void handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback);
 public:
  StrategyIOManager() = default;
  void subscribeToRoleFeedback();
  roboteam_msgs::RoleFeedback &getRoleFeedback();
};

#endif //ROBOTEAM_AI_STRATEGY_IO_NODE_H
