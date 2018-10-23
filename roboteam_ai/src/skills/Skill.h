#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"
#include "ros/ros.h"
#include "roboteam_msgs/WorldRobot.h"
#include "../io/RoleIOManager.h"
#include "../utilities/Constants.h"
#include <roboteam_msgs/RobotCommand.h>

namespace rtt {
namespace ai {

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
 class Skill : public bt::Leaf {
  protected:
   roboteam_msgs::WorldRobot robot;
  public:
    explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    Status Update() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
