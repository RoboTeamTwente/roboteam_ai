#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include "../bt/Leaf.hpp"
#include "ros/ros.h"
#include "../io/IOManager.h"
#include "roboteam_msgs/WorldRobot.h"
#include <roboteam_msgs/RobotCommand.h>
#include "../../src/control/ControlUtils.h"
#include "../utilities/Constants.h"
#include "../utilities/Coach.h"
#include "roboteam_utils/Vector2.h"



namespace rtt {
namespace ai {

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    protected:

        io::IOManager ioManager;
        using coach = coach::Coach;
        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);
        void terminate(Status s) override;
public:
        using Control = control::ControlUtils;
        using Status = bt::Node::Status;
        explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SKILL_H
