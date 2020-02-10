#ifndef ROBOTEAM_AI_SKILL_H
#define ROBOTEAM_AI_SKILL_H

#include <roboteam_utils/Angle.h>
#include "include/roboteam_ai/utilities/IOManager.h"
#include "roboteam_proto/RobotCommand.pb.h"
#include "treeinterp/Leaf.h"

namespace rtt::ai {

// forward declare control Utils
namespace control {
class ControlUtils;
}

namespace world {
class Robot;
class Ball;
class WorldData;
}  // namespace world

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
   private:
    proto::RobotCommand rotateRobotCommand(proto::RobotCommand &cmd);

   protected:
    using Robot = world::Robot;
    using Ball = world::Ball;
    using RobotPtr = std::shared_ptr<world::Robot>;
    using BallPtr = std::shared_ptr<world::Ball>;
    using WorldData = world::WorldData;

    void publishRobotCommand();
    void refreshRobotCommand();
    proto::RobotCommand command;

    using Control = control::ControlUtils;
    using Status = bt::Node::Status;
    void limitRobotCommand();

   public:
    explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    std::string node_name() override;
    void initialize() override;
    Status update() override;
    void terminate(Status s) override;
    virtual void onInitialize(){};
    virtual Status onUpdate() = 0;
    virtual void onTerminate(Status s){};
    void refreshRobotPositionControllers();
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_SKILL_H
