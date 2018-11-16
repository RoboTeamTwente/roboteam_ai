#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"
#include "ros/ros.h"
#include "roboteam_msgs/WorldRobot.h"
#include "../io/IOManager.h"
#include "../utilities/Constants.h"
#include <roboteam_msgs/RobotCommand.h>
#include "../../src/control/ControlUtils.h"

namespace rtt {
namespace ai {

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    protected:
        roboteam_msgs::WorldRobot robot;
        io::IOManager ioManager;

        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);

    public:
        using Control = control::ControlUtils;
        using Status = bt::Node::Status;


        explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
