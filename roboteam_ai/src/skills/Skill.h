#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include "../bt/Leaf.hpp"
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_ai/src/utilities/Coach.h>
#include "ros/ros.h"
#include "../io/IOManager.h"
#include "../control/ControlGoToPos.h"
namespace rtt {
namespace ai {

// forward declare control Utils
namespace control {
    class ControlUtils;
}



/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    private:
        void rotateRobotCommand(roboteam_msgs::RobotCommand &cmd);
protected:
        io::IOManager ioManager = io::IOManager(false,true);

        using Coach = coach::Coach;
        using GoToType = control::GoToType;
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
