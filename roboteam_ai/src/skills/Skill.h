#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include "../bt/Leaf.hpp"
#include <roboteam_msgs/RobotCommand.h>
#include "ros/ros.h"
#include "../io/IOManager.h"
#include "roboteam_ai/src/control/PositionController.h"
#include "../control/positionControllers/PosVelAngle.h"

namespace rtt {
namespace ai {

// forward declare control Utils
namespace control {
    class ControlUtils;
    class PosVelAngle;
}



/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    private:
        roboteam_msgs::RobotCommand rotateRobotCommand(roboteam_msgs::RobotCommand &cmd);
protected:
        io::IOManager ioManager = io::IOManager(false,true);

        using GoToType = control::PosControlType;
        void publishRobotCommand(roboteam_msgs::RobotCommand cmd);
    public:

        using Control = control::ControlUtils;
        using Status = bt::Node::Status;
        explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        std::string node_name() override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;

        virtual void onInitialize() { };
        virtual Status onUpdate() = 0;
        virtual void onTerminate(Status s) { };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SKILL_H
