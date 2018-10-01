/*
 * Receives and handles the world state, as well as rolefeedback
 * RoboTeamTwente, september 2018
 */

#ifndef ROBOTEAM_AI_STRATEGY_IO_NODE_H
#define ROBOTEAM_AI_STRATEGY_IO_NODE_H

#include "IOManager.h"
#include "roboteam_msgs/RoleFeedback.h"

namespace rtt {
namespace ai {
namespace io {

class StrategyIOManager : public IOManager {
 private:
  roboteam_msgs::RoleFeedback roleFeedback;
  void handleRobotFeedback(const roboteam_msgs::RoleFeedbackConstPtr &rolefeedback);
 public:
  StrategyIOManager();
  void subscribeToRoleFeedback();
  roboteam_msgs::RoleFeedback &getRoleFeedback();
};

} // io
} // ai
} // rtt

#endif //ROBOTEAM_AI_STRATEGY_IO_NODE_H
