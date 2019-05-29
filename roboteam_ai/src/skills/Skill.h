#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include "../bt/Leaf.hpp"
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_utils/Angle.h>
#include "ros/ros.h"
#include "../io/IOManager.h"
#include "roboteam_ai/src/control/positionControllers/RobotCommand.h"

namespace rtt {
namespace ai {

// forward declare control Utils
namespace control {
class ControlUtils;
}

namespace world {
    class Robot;
    class Ball;
    class WorldData;
}

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    private:
        roboteam_msgs::RobotCommand rotateRobotCommand(roboteam_msgs::RobotCommand &cmd);
    protected:
        using Robot = world::Robot;
        using Ball = world::Ball;
        using WorldData = world::WorldData;

        io::IOManager ioManager = io::IOManager(false, true);
        void publishRobotCommand();
    void refreshRobotCommand();
    roboteam_msgs::RobotCommand command;

        using Control = control::ControlUtils;
        using Status = bt::Node::Status;
    void limitRobotCommand();

    public:
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
