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

/**
 * \class Skill
 * \brief Base class for all skills. Provides no additional functionality.
 */
class Skill : public bt::Leaf {
   private:
    /** Flips the command around, to work with a mirrored field */
    proto::RobotCommand rotateRobotCommand(proto::RobotCommand &cmd);

   protected:
    proto::RobotCommand command;

    /** Flips the command if needed. Limits speed of the robots. Publishes the command. */
    void publishRobotCommand();
    /** Empties the robot command, and sets its robotId */
    void resetRobotCommand();
    /** Limits the velocities of the robot command based on the current state of the game */
    void limitRobotCommand();

    using Control = control::ControlUtils;
    using Status = bt::Node::Status;

   public:
    explicit Skill(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    std::string node_name() override;

    /** Sets the ball. Sets the correct robot based on the blackboard. Calls onInitialize() */
    void initialize() override;
    /** Failure if the robot isn't present. Waiting if the ball isn't present. Calls onUpdate() */
    Status update() override;
    /** Resets the robot controllers and command. Calls onTerminate() */
    void terminate(Status s) override;

    /** These functions have to be implemented by the deriving class */
    virtual void onInitialize(){};
    virtual Status onUpdate() = 0;
    virtual void onTerminate(Status s){};

    /** Resets the controllers of the robot TODO implement with new_world */
    void refreshRobotPositionControllers();
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_SKILL_H
