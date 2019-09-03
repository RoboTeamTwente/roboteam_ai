#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include "bt/Leaf.hpp"
#include "io/IOManager.h"
#include "roboteam_proto/RobotCommand.pb.h"
#include <roboteam_utils/Angle.h>

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

using namespace std;

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
    private:
        roboteam_proto::RobotCommand rotateRobotCommand(roboteam_proto::RobotCommand &cmd);
    protected:
        using Robot = world::Robot;
        using Ball = world::Ball;
        using RobotPtr = std::shared_ptr<world::Robot>;
        using BallPtr = std::shared_ptr<world::Ball>;
        using WorldData = world::WorldData;

        void publishRobotCommand();
        void refreshRobotCommand();
        roboteam_proto::RobotCommand command;

        using Control = control::ControlUtils;
        using Status = bt::Node::Status;
        void limitRobotCommand();

    public:
        explicit Skill(string name, bt::Blackboard::Ptr blackboard = nullptr);
        std::string node_name() override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;
        virtual void onInitialize() { };
        virtual Status onUpdate() = 0;
        virtual void onTerminate(Status s) { };
        void refreshRobotPositionControllers();
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SKILL_H
